
#include <rclcpp/rclcpp.hpp>
#include <filters/filter_base.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/circular_buffer.hpp>
#include <condition_variable>

struct Frames {
  Eigen::Vector3d b;
  Eigen::Vector3d w;
};

struct Velocity {
  Frames lin;
  Frames ang;
};

struct Pose {
  Eigen::Vector3d p; // position in world frame
  Eigen::Quaterniond q; // orientation in world frame
  Eigen::Vector3d v;// linear velocity in world frame
};

struct ImuBias {
  Eigen::Vector3d gyro;
  Eigen::Vector3d accel;
};

struct ImuMeas {
  uint64_t stamp;
  uint32_t dt; // defined as the difference between the current and the previous measurement
  Eigen::Vector3d ang_vel;
  Eigen::Vector3d lin_accel;
};

struct State {
  Eigen::Vector3d p; // position in world frame
  Eigen::Quaterniond q; // orientation in world frame
  Velocity v; // velocity in world frame
  ImuBias b; // imu biases in body frame
};

namespace pointcloud2_filters
{
  class DeskewImu : public filters::FilterBase<sensor_msgs::msg::PointCloud2>
  {
    protected:

      std::unique_ptr<tf2_ros::Buffer> m_buffer;
      std::shared_ptr<tf2_ros::TransformListener> m_listener{nullptr};
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pubIntermediate;
      
      rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_subImu;

      std::atomic<bool> m_isReceivedImu;
      std::atomic<bool> m_isReceivedLidar;
      std::atomic<bool> m_isCalibratedImu;
      State m_state;

      uint64_t m_stampImuFirst;
      uint64_t m_stampImuPrevious;

      uint64_t m_stampScanPrevious;
      uint64_t m_stampScanCurrent;
      
      Pose m_posePrevious;
      Pose m_poseCurrent;
      ImuMeas m_imuMeasure;
      Eigen::Matrix3d m_imuAcceleration;

      std::mutex m_mutexImu;
      std::condition_variable m_cvImuStamp;
      
      Eigen::Affine3d m_map2Baselink;
      Eigen::Affine3d m_imu2Baselink;
      Eigen::Affine3d m_lidar2Baselink;
      boost::circular_buffer<ImuMeas> m_imuBuffer;
      int m_imuBufferSize;

      std::string imu_topic_;

      std::string m_mapFrameId;
      std::string m_robotFrameId;
      std::string m_field;
      double m_imuCalibTime;
      double m_waitForTfDelay;      
      double m_conversion;
      double m_frequency;
      double m_gravity;

      bool m_calibrateGyro;
      bool m_calibrateAccel;


      public:
      DeskewImu() { }

      bool configure()
      {
        RCLCPP_INFO(this->logging_interface_->get_logger(),"Pointcloud2DeskewImu configuring");

        // Setup tf2
        if(this->get_node() != nullptr)
        {
          m_buffer = std::make_unique<tf2_ros::Buffer>(this->get_node()->get_clock());
          m_listener = std::make_shared<tf2_ros::TransformListener>(*m_buffer);
        }

        getParam("map_frame_id", m_mapFrameId, "odom");
        getParam("robot_frame_id", m_mapFrameId, "base_link");
        getParam("field", m_field, "time");
        getParam("conversion", m_conversion, 1.0);
        getParam("frequency", m_frequency, 10.0);
        getParam("wait_for_tf_delay", m_waitForTfDelay, 0.1);

        getParam("imu_topic", imu_topic_, "imu");
        getParam("imu_calib_time", m_imuCalibTime, 3.0);
        getParam("imu_buffer_size", m_imuBufferSize, 2000);
        getParam("gravity", m_gravity, 9.80665);

        getParam("calibrate_gyro", m_calibrateGyro, true);
        getParam("calibrate_accel", m_calibrateAccel, true);

        RCLCPP_INFO(this->logging_interface_->get_logger(),"  map frame id is: %s", m_mapFrameId.c_str());
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  robot frame id is: %s", m_robotFrameId.c_str());
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  field is: %s", m_field.c_str());
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  conversion factor is: %f", m_conversion);
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  frequency is: %f", m_frequency);
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  wait_for_tf_delay: %f", m_waitForTfDelay);

        m_map2Baselink.setIdentity();
        m_imu2Baselink.setIdentity();
        m_lidar2Baselink.setIdentity();
        m_imuBuffer.set_capacity(m_imuBufferSize);

        m_isReceivedImu = false;
        m_isReceivedLidar = false;
        m_isCalibratedImu =false;
        
        m_stampImuFirst = 0;
        m_stampImuPrevious = 0;
        m_stampScanPrevious = 0;

        m_state.b.accel = Eigen::Vector3d(0., 0., 0.);
        m_state.b.gyro = Eigen::Vector3d(0., 0., 0.);
        m_imuAcceleration = Eigen::Matrix3d::Identity();

        m_posePrevious.p = Eigen::Vector3d(0., 0., 0.);
        m_posePrevious.q = Eigen::Quaterniond(1., 0., 0., 0.);
        m_posePrevious.v = Eigen::Vector3d(0., 0., 0.);

        m_poseCurrent.p = Eigen::Vector3d(0., 0., 0.);
        m_poseCurrent.q = Eigen::Quaterniond(1., 0., 0., 0.);
        m_poseCurrent.v = Eigen::Vector3d(0., 0., 0.);

        m_imuMeasure.stamp = 0;
        m_imuMeasure.ang_vel[0] = 0.;
        m_imuMeasure.ang_vel[1] = 0.;
        m_imuMeasure.ang_vel[2] = 0.;
        m_imuMeasure.lin_accel[0] = 0.;
        m_imuMeasure.lin_accel[1] = 0.;
        m_imuMeasure.lin_accel[2] = 0.;

        // Create subscriber
        m_subImu = this->get_node()->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, 1000, std::bind(&DeskewImu::callbackImu, this, std::placeholders::_1));

        // Create publisher
        if(this->get_node() != nullptr)
            m_pubIntermediate = this->get_node()->create_publisher<sensor_msgs::msg::PointCloud2>("output", 10);

        return true;
      }

      bool update(
        const sensor_msgs::msg::PointCloud2& input_msg, sensor_msgs::msg::PointCloud2& output_msg)
      {
        output_msg = input_msg;

        // Check message for data
        if (input_msg.data.size() == 0)
        {
          if(this->get_node() != nullptr) 
            RCLCPP_WARN_STREAM_THROTTLE(this->logging_interface_->get_logger(), *this->get_node()->get_clock(), 1000, "No data in pointcloud!");
          else
            RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "No data in pointcloud!");
          output_msg = input_msg;
          return false;
        }

        // Exit if imu buffer is empty
        rclcpp::Time inputTime(input_msg.header.stamp);
        rclcpp::Time imuTime(input_msg.header.stamp);
        if (m_imuBuffer.empty() || inputTime <= imuTime) 
        {
          RCLCPP_WARN_STREAM_THROTTLE(this->logging_interface_->get_logger(), *this->get_node()->get_clock(), 1000, "Imu buffer does not contain data! " << inputTime.seconds());
          return false;
        }

        // Store lidar to baselink transform on first call
        if(!m_isReceivedLidar)
        {
          // Check to see if we can do transforms
          if(m_buffer == nullptr)
          {
            RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "filters_pointcloud2_erdc/DescewImu does not have access to tf data!");
            return false;
          } else {   
            try
            { 
              if(m_buffer->canTransform(m_robotFrameId, input_msg.header.frame_id, input_msg.header.stamp, rclcpp::Duration::from_seconds(m_waitForTfDelay)))
              {
                geometry_msgs::msg::TransformStamped transform;
                transform = m_buffer->lookupTransform(m_robotFrameId, input_msg.header.frame_id, input_msg.header.stamp);
                m_lidar2Baselink = tf2::transformToEigen(transform);
              }
              else
              {
                RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "  DeskewImu is waiting for transform from " << input_msg.header.frame_id << " to " << m_robotFrameId << " to become available.");
                return false;
              }
            }
            catch (tf2::TransformException &ex) 
            {
              RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "  DeskewImu Thread: " << ex.what());
              return false;
            }
          }
          
          m_isReceivedLidar = true;
        }

        // create map of field offsets for easy access
        std::map<std::string, uint32_t> offset;
        for (int i=0; i < input_msg.fields.size(); i++)
        {
          offset.emplace(input_msg.fields[i].name, input_msg.fields[i].offset);
        } 

        // Create sorted list of timestamps
        std::map<uint64_t, uint32_t> timestamps;
        for(int i=0; i < input_msg.height * input_msg.width; i++)
        {
          uint32_t timeInput;
          uint64_t point_index = i * input_msg.point_step;

          std::memcpy(&timeInput, &input_msg.data[point_index + offset[m_field]], sizeof(uint32_t));
          
          rclcpp::Duration timeShift = rclcpp::Duration::from_nanoseconds((uint32_t)(m_conversion * (double)timeInput));
          rclcpp::Time timePoint(output_msg.header.stamp);
          timestamps[timePoint.nanoseconds()] = 0; 
        }

        // Get current lidar position in map frame
        if(m_stampScanPrevious == 0) m_stampScanPrevious = rclcpp::Time(input_msg.header.stamp).nanoseconds() - (1 / m_frequency * 1e9);               
        if(m_buffer == nullptr)
        {
          RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "filters_pointcloud2_erdc/DescewImu does not have access to tf data!");
          return false;            
        } else {  
          try
          { 
            if(m_buffer->canTransform(m_mapFrameId, input_msg.header.frame_id, input_msg.header.stamp, rclcpp::Duration::from_seconds(m_waitForTfDelay)))
            {
              // Treat current position as zero so all corections are relative to current position
              geometry_msgs::msg::TransformStamped transform;
              transform = m_buffer->lookupTransform(m_mapFrameId, input_msg.header.frame_id, input_msg.header.stamp);
              Eigen::Affine3d affine = tf2::transformToEigen(transform);
              double dt = (rclcpp::Time(input_msg.header.stamp).seconds() - m_stampScanPrevious) * 1e-9f;
              m_stampScanCurrent = rclcpp::Time(input_msg.header.stamp).seconds(); 
              m_poseCurrent.q = Eigen::Quaterniond(1., 0., 0., 0.);
              m_poseCurrent.p = Eigen::Vector3d(0., 0., 0.);
              m_poseCurrent.v = affine.rotation().inverse() * ((affine.translation() - m_posePrevious.p) / dt);            
            }
            else
            {
              RCLCPP_WARN_STREAM_THROTTLE(this->logging_interface_->get_logger(), *this->get_node()->get_clock(), 1000, "  DeskewImu is waiting for transform from " << input_msg.header.frame_id << " to " << m_mapFrameId << " to become available.");
              return false;
            }
          }
          catch (tf2::TransformException &ex) 
          {
            RCLCPP_WARN_STREAM_THROTTLE(this->logging_interface_->get_logger(), *this->get_node()->get_clock(), 1000, "  DeskewImu Thread: " << ex.what());
            return false;
          }
        }

        // Create poses for each timestamp from IMU priors
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> frames;
        frames = integrateImu(m_stampScanCurrent, m_poseCurrent.q, m_poseCurrent.p, m_poseCurrent.v, timestamps);

        // if there are no frames between the start and end of the sweep
        // that probably means that there's a sync issue
        if (frames.size() != timestamps.size()) {
          RCLCPP_FATAL(this->logging_interface_->get_logger(), "Bad time sync between LiDAR and IMU!");

          m_stampScanPrevious = rclcpp::Time(input_msg.header.stamp).nanoseconds();          
          m_posePrevious.p = Eigen::Vector3d(0., 0., 0.);
          m_posePrevious.q = Eigen::Quaterniond(1., 0., 0., 0.);
          m_posePrevious.v = Eigen::Vector3d(0., 0., 0.);    
          return false;
        }

        // setup iterators
        sensor_msgs::PointCloud2Iterator<float> it_x(output_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(output_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(output_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint32_t> it_t(output_msg, m_field);

        // transform at time for each point
        for (; it_x != it_x.end() && it_y != it_y.end() && it_z != it_z.end() && it_t != it_t.end(); ++it_x, ++it_y, ++it_z, ++it_t)
        {
          uint32_t time;
          Eigen::Vector3f point;

          point[0] = *it_x;
          point[1] = *it_y;
          point[2] = *it_z;
          time = *it_t;

          // Get Transform
          rclcpp::Duration timeShift = rclcpp::Duration::from_nanoseconds((uint32_t)(m_conversion * (double)time));
          rclcpp::Time timePoint(output_msg.header.stamp);
          timePoint += timeShift;
          uint32_t index = timestamps[timePoint.nanoseconds()];          

          Eigen::Affine3d frame(frames[index]);
          Eigen::Affine3d T = frame * m_lidar2Baselink;
          Eigen::Affine3f affine(T.cast<float>());
          point = affine * point;

          *it_x = point[0];
          *it_y = point[1];
          *it_z = point[2];
          *it_t = 0.0;
        }

        // publish here for debug purposes
        if(m_pubIntermediate!= nullptr && m_pubIntermediate->get_subscription_count() > 0) 
          m_pubIntermediate->publish(output_msg);

        if(m_buffer == nullptr)
        {
          RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "filters_pointcloud2_erdc/DescewImu does not have access to tf data!");
          return false;            
        } else {  
          // Get current lidar position in map frame
          try
          { 
            if(m_buffer->canTransform(m_mapFrameId, input_msg.header.frame_id, input_msg.header.stamp, rclcpp::Duration::from_seconds(m_waitForTfDelay)))
            {
              geometry_msgs::msg::TransformStamped transform;
              transform = m_buffer->lookupTransform(m_mapFrameId, input_msg.header.frame_id, input_msg.header.stamp);
              Eigen::Affine3d affine = tf2::transformToEigen(transform);
              double dt = (rclcpp::Time(input_msg.header.stamp).nanoseconds() - m_stampScanPrevious) * 1e-9f;
              m_posePrevious.v = (affine.translation() - m_posePrevious.p) / dt;
              m_posePrevious.p = affine.translation();
              m_posePrevious.q = affine.rotation();
              m_stampScanPrevious = rclcpp::Time(input_msg.header.stamp).nanoseconds(); 
            }
            else
            {
              RCLCPP_WARN_STREAM_THROTTLE(this->logging_interface_->get_logger(), *this->get_node()->get_clock(), 1000, "  DeskewImu is waiting for transform from " << input_msg.header.frame_id << " to " << m_mapFrameId << " to become available.");
              return false;
            }
          }
          catch (tf2::TransformException &ex) 
          {
            RCLCPP_WARN_STREAM_THROTTLE(this->logging_interface_->get_logger(), *this->get_node()->get_clock(), 1000, "  DeskewImu Thread: " << ex.what());
            return false;
          }
        }

        return true;
      }

      // transform IMU values to reference frame
      sensor_msgs::msg::Imu::Ptr transformImu(const sensor_msgs::msg::Imu::ConstPtr& imu_raw) 
      {
        sensor_msgs::msg::Imu::Ptr imu(new sensor_msgs::msg::Imu);

        // Copy header
        // RCLCPP_INFO_STREAM(this->logging_interface_->get_logger(),"Copy header");
        imu->header = imu_raw->header;

        // Transform angular velocity (will be the same on a rigid body, so just rotate to ROS convention)
        // RCLCPP_INFO_STREAM(this->logging_interface_->get_logger(),"angular velocity");          
        Eigen::Vector3d ang_vel(imu_raw->angular_velocity.x,
                                imu_raw->angular_velocity.y,
                                imu_raw->angular_velocity.z);

        Eigen::Vector3d ang_vel_cg = m_imu2Baselink.rotation() * ang_vel;

        imu->angular_velocity.x = ang_vel_cg[0];
        imu->angular_velocity.y = ang_vel_cg[1];
        imu->angular_velocity.z = ang_vel_cg[2];

        // Transform linear acceleration (need to account for component due to translational difference)
        // RCLCPP_INFO_STREAM(this->logging_interface_->get_logger(),"linear acceleration");        
        Eigen::Vector3d lin_accel(imu_raw->linear_acceleration.x,
                                  imu_raw->linear_acceleration.y,
                                  imu_raw->linear_acceleration.z);                                

        Eigen::Vector3d lin_accel_cg = m_imu2Baselink.rotation() * lin_accel;      

        static Eigen::Vector3d ang_vel_cg_prev = ang_vel_cg;

        lin_accel_cg = lin_accel_cg
        // + ((ang_vel_cg - ang_vel_cg_prev) / dt).cross(-this->extrinsics.baselink2imu.t)
        + ang_vel_cg.cross(ang_vel_cg.cross(-this->m_imu2Baselink.translation()));

        imu->linear_acceleration.x = lin_accel_cg[0];
        imu->linear_acceleration.y = lin_accel_cg[1];
        imu->linear_acceleration.z = lin_accel_cg[2];

        return imu;
      }

      void callbackImu(const sensor_msgs::msg::Imu::ConstPtr& imu_raw) 
      {
        if(imu_raw == nullptr) return;

        // Store imu to baselink transform on first call
        if(!m_isReceivedImu)
        {
           // Check to see if we can do transforms
          if(m_buffer == nullptr)
          {
            RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "filters_pointcloud2_erdc/BoxFilter does not have access to tf data!");
          } else {   
            try
            { 
              if(m_buffer->canTransform(m_robotFrameId, imu_raw->header.frame_id, imu_raw->header.stamp, rclcpp::Duration::from_seconds(m_waitForTfDelay)))
              {
                geometry_msgs::msg::TransformStamped transform;
                transform = m_buffer->lookupTransform(m_robotFrameId, imu_raw->header.frame_id, imu_raw->header.stamp);
                m_imu2Baselink = tf2::transformToEigen(transform);
              }
              else
              {
                RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "  DeskewImu is waiting for transform from " << imu_raw->header.frame_id << " to " << m_robotFrameId << " to become available.");
                return;
              }
            }
            catch (tf2::TransformException &ex) 
            {
              RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "  DeskewImu Thread: " << ex.what());
              return;
            }

            try
            { 
              if(m_buffer->canTransform(m_mapFrameId, m_robotFrameId, imu_raw->header.stamp, rclcpp::Duration::from_seconds(m_waitForTfDelay)))
              {
                geometry_msgs::msg::TransformStamped transform;
                transform = m_buffer->lookupTransform(m_mapFrameId, m_robotFrameId, imu_raw->header.stamp);
                m_map2Baselink = tf2::transformToEigen(transform);
              }
              else
              {
                RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "  DeskewImu is waiting for transform from " << m_robotFrameId << " to " << m_mapFrameId << " to become available.");
                return;
              }
            }
            catch (tf2::TransformException &ex) 
            {
              RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "  DeskewImu Thread: " << ex.what());
              return;
            }
          }
          
          m_isReceivedImu = true;
        }

        sensor_msgs::msg::Imu::Ptr imu = transformImu( imu_raw );

        Eigen::Vector3d lin_accel;
        Eigen::Vector3d ang_vel;

        // Get IMU samples
        ang_vel[0] = (double)imu->angular_velocity.x;
        ang_vel[1] = (double)imu->angular_velocity.y;
        ang_vel[2] = (double)imu->angular_velocity.z;

        lin_accel[0] = (double)imu->linear_acceleration.x;
        lin_accel[1] = (double)imu->linear_acceleration.y;
        lin_accel[2] = (double)imu->linear_acceleration.z;

        if (m_stampImuFirst == 0) {
          m_stampImuFirst = rclcpp::Time(imu->header.stamp).nanoseconds();
        }

        // IMU calibration procedure - do for three seconds
        if (!m_isCalibratedImu) {

          static int num_samples = 0;
          static Eigen::Vector3d gyro_avg (0., 0., 0.);
          static Eigen::Vector3d accel_avg (0., 0., 0.);
          static bool print = true;

          // Store data for calibration
          if ((rclcpp::Time(imu->header.stamp).nanoseconds() - m_stampImuFirst) < (uint64_t)(m_imuCalibTime * 1e+9f)) {

            num_samples++;

            RCLCPP_INFO_STREAM(this->logging_interface_->get_logger(),"Get imu sample: " << (rclcpp::Time(imu->header.stamp).nanoseconds() - m_stampImuFirst));   
            RCLCPP_INFO_STREAM(this->logging_interface_->get_logger(),"  Angular velocity: " << (float)ang_vel[0] << ":" << (float)ang_vel[1] << ":" << (float)ang_vel[2]);  
            RCLCPP_INFO_STREAM(this->logging_interface_->get_logger(),"  Linear_acceleration: " << (float)lin_accel[0] << ":" << (float)lin_accel[1] << ":" << (float)lin_accel[2]);  
            
            gyro_avg[0] += ang_vel[0];
            gyro_avg[1] += ang_vel[1];
            gyro_avg[2] += ang_vel[2];

            accel_avg[0] += lin_accel[0];
            accel_avg[1] += lin_accel[1];
            accel_avg[2] += lin_accel[2];
            if(print) {
              RCLCPP_INFO_STREAM(this->logging_interface_->get_logger()," Calibrating IMU for " << m_imuCalibTime << " seconds... ");
              print = false;
            }
          // Calibrate
          } else {

            RCLCPP_INFO_STREAM(this->logging_interface_->get_logger(),"  Done calibrating IMU" << std::endl);

            gyro_avg /= num_samples;
            accel_avg /= num_samples;

            Eigen::Vector3d grav_vec (0., 0., m_gravity);
            grav_vec = m_map2Baselink.rotation().inverse() * grav_vec;

            if (m_calibrateAccel) {

              // subtract gravity from avg accel to get bias
              m_state.b.accel = accel_avg - grav_vec;

              RCLCPP_INFO_STREAM(this->logging_interface_->get_logger()," Accel biases [xyz]: " << (float)m_state.b.accel[0] << ", "
                                                      << (float)m_state.b.accel[1] << ", "
                                                      << (float)m_state.b.accel[2] << std::endl);
            }

            if (m_calibrateGyro) {

              m_state.b.gyro = gyro_avg;

              RCLCPP_INFO_STREAM(this->logging_interface_->get_logger()," Gyro biases  [xyz]: " << (float)m_state.b.gyro[0] << ", "
                                                      << (float)m_state.b.gyro[1] << ", "
                                                      << (float)m_state.b.gyro[2] << std::endl);
            }

            m_isCalibratedImu = true;

          }

        } else {

          uint32_t dt = (rclcpp::Time(imu->header.stamp).nanoseconds() - m_stampImuPrevious);
          if (dt == 0) { dt = 1e9/200; }
          // this->imu_rates.push_back( 1./dt );

          // Apply the calibrated bias to the new IMU measurements
          m_imuMeasure.stamp = rclcpp::Time(imu->header.stamp).nanoseconds();
          m_imuMeasure.dt = dt;
          m_stampImuPrevious = m_imuMeasure.stamp;

          Eigen::Vector3d lin_accel_corrected = (m_imuAcceleration * lin_accel) - m_state.b.accel;
          Eigen::Vector3d ang_vel_corrected = ang_vel - m_state.b.gyro;

          m_imuMeasure.lin_accel = lin_accel_corrected;
          m_imuMeasure.ang_vel = ang_vel_corrected;

          // // Store calibrated IMU measurements into imu buffer for manual integration later.
          this->m_mutexImu.lock();
          m_imuBuffer.push_front(m_imuMeasure);
          this->m_mutexImu.unlock();

          // Notify the callbackPointCloud thread that IMU data exists for this time
          m_cvImuStamp.notify_one();

          // propagateState();
        }

      }

      std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>
      integrateImu(uint64_t start_time, Eigen::Quaterniond q_init, Eigen::Vector3d p_init,
                                  Eigen::Vector3d v_init, std::map<uint64_t, uint32_t>& sorted_timestamps) {

        const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> empty;

        if (sorted_timestamps.empty() || start_time > sorted_timestamps.begin()->first) {
          // invalid input, return empty vector
          return empty;
        }

        boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it;
        boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it;
        if (imuMeasFromTimeRange(start_time, sorted_timestamps.rbegin()->first, begin_imu_it, end_imu_it) == false) {
          // not enough IMU measurements, return empty vector
          return empty;
        }

        // Backwards integration to find pose at first IMU sample
        const ImuMeas& f1 = *begin_imu_it;
        const ImuMeas& f2 = *(begin_imu_it+1);

        // Time between first two IMU samples
        double dt = (double)f2.dt * 1e-9;

        // Time between first IMU sample and start_time
        double idt = (double)(start_time - f1.stamp) * 1e-9;

        // Angular acceleration between first two IMU samples
        Eigen::Vector3d alpha_dt = f2.ang_vel - f1.ang_vel;
        Eigen::Vector3d alpha = alpha_dt / dt;

        // Average angular velocity (reversed) between first IMU sample and start_time
        Eigen::Vector3d omega_i = -(f1.ang_vel + 0.5*alpha*idt);

        // Set q_init to orientation at first IMU sample
        q_init = Eigen::Quaterniond (
          q_init.w() - 0.5*( q_init.x()*omega_i[0] + q_init.y()*omega_i[1] + q_init.z()*omega_i[2] ) * idt,
          q_init.x() + 0.5*( q_init.w()*omega_i[0] - q_init.z()*omega_i[1] + q_init.y()*omega_i[2] ) * idt,
          q_init.y() + 0.5*( q_init.z()*omega_i[0] + q_init.w()*omega_i[1] - q_init.x()*omega_i[2] ) * idt,
          q_init.z() + 0.5*( q_init.x()*omega_i[1] - q_init.y()*omega_i[0] + q_init.w()*omega_i[2] ) * idt
        );
        q_init.normalize();

        // Average angular velocity between first two IMU samples
        Eigen::Vector3d omega = f1.ang_vel + 0.5*alpha_dt;

        // Orientation at second IMU sample
        Eigen::Quaterniond q2 (
          q_init.w() - 0.5*( q_init.x()*omega[0] + q_init.y()*omega[1] + q_init.z()*omega[2] ) * dt,
          q_init.x() + 0.5*( q_init.w()*omega[0] - q_init.z()*omega[1] + q_init.y()*omega[2] ) * dt,
          q_init.y() + 0.5*( q_init.z()*omega[0] + q_init.w()*omega[1] - q_init.x()*omega[2] ) * dt,
          q_init.z() + 0.5*( q_init.x()*omega[1] - q_init.y()*omega[0] + q_init.w()*omega[2] ) * dt
        );
        q2.normalize();

        // Acceleration at first IMU sample
        Eigen::Vector3d a1 = q_init._transformVector(f1.lin_accel);
        a1[2] -= m_gravity;

        // Acceleration at second IMU sample
        Eigen::Vector3d a2 = q2._transformVector(f2.lin_accel);
        a2[2] -= m_gravity;

        // Jerk between first two IMU samples
        Eigen::Vector3d j = (a2 - a1) / dt;

        // Set v_init to velocity at first IMU sample (go backwards from start_time)
        v_init -= a1*idt + 0.5*j*idt*idt;

        // Set p_init to position at first IMU sample (go backwards from start_time)
        p_init -= v_init*idt + 0.5*a1*idt*idt + (1/6.)*j*idt*idt*idt;

        return this->integrateImuInternal(q_init, p_init, v_init, sorted_timestamps, begin_imu_it, end_imu_it);
      }

      std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>
      integrateImuInternal(Eigen::Quaterniond q_init, Eigen::Vector3d p_init, Eigen::Vector3d v_init,
                                          std::map<uint64_t, uint32_t>& sorted_timestamps,
                                          boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
                                          boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it) {

        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> imu_se3;

        // Initialization
        Eigen::Quaterniond q = q_init;
        Eigen::Vector3d p = p_init;
        Eigen::Vector3d v = v_init;
        Eigen::Vector3d a = q._transformVector(begin_imu_it->lin_accel);
        a[2] -= m_gravity;;

        // Iterate over IMU measurements and timestamps
        auto prev_imu_it = begin_imu_it;
        auto imu_it = prev_imu_it + 1;

        auto stamp_it = sorted_timestamps.begin();

        for (; imu_it != end_imu_it; imu_it++) {

          const ImuMeas& f0 = *prev_imu_it;
          const ImuMeas& f = *imu_it;

          // Time between IMU samples
          double dt = f.dt * 1e-9;

          // Angular acceleration
          Eigen::Vector3d alpha_dt = f.ang_vel - f0.ang_vel;
          Eigen::Vector3d alpha = alpha_dt / dt;

          // Average angular velocity
          Eigen::Vector3d omega = f0.ang_vel + 0.5*alpha_dt;

          // Orientation
          q = Eigen::Quaterniond (
            q.w() - 0.5*( q.x()*omega[0] + q.y()*omega[1] + q.z()*omega[2] ) * dt,
            q.x() + 0.5*( q.w()*omega[0] - q.z()*omega[1] + q.y()*omega[2] ) * dt,
            q.y() + 0.5*( q.z()*omega[0] + q.w()*omega[1] - q.x()*omega[2] ) * dt,
            q.z() + 0.5*( q.x()*omega[1] - q.y()*omega[0] + q.w()*omega[2] ) * dt
          );
          q.normalize();

          // Acceleration
          Eigen::Vector3d a0 = a;
          a = q._transformVector(f.lin_accel);
          a[2] -= m_gravity;;

          // Jerk
          Eigen::Vector3d j_dt = a - a0;
          Eigen::Vector3d j = j_dt / dt;

          // Interpolate for given timestamps
          while (stamp_it != sorted_timestamps.end() && stamp_it->first <= f.stamp) {
            // Time between previous IMU sample and given timestamp
            double idt = (double)(stamp_it->first - f0.stamp) * 1e-9f;

            // Average angular velocity
            Eigen::Vector3d omega_i = f0.ang_vel + 0.5*alpha*idt;

            // Orientation
            Eigen::Quaterniond q_i (
              q.w() - 0.5*( q.x()*omega_i[0] + q.y()*omega_i[1] + q.z()*omega_i[2] ) * idt,
              q.x() + 0.5*( q.w()*omega_i[0] - q.z()*omega_i[1] + q.y()*omega_i[2] ) * idt,
              q.y() + 0.5*( q.z()*omega_i[0] + q.w()*omega_i[1] - q.x()*omega_i[2] ) * idt,
              q.z() + 0.5*( q.x()*omega_i[1] - q.y()*omega_i[0] + q.w()*omega_i[2] ) * idt
            );
            q_i.normalize();

            // Position
            Eigen::Vector3d p_i = p + v*idt + 0.5*a0*idt*idt + (1/6.)*j*idt*idt*idt;

            // Transformation
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block(0, 0, 3, 3) = q_i.toRotationMatrix();
            T.block(0, 3, 3, 1) = p_i;
            // RCLCPP_INFO_STREAM(this->logging_interface_->get_logger(),"p_i: " << p_i);

            stamp_it->second = imu_se3.size();
            imu_se3.push_back(T);

            stamp_it++;
          }

          // Position
          p += v*dt + 0.5*a0*dt*dt + (1/6.)*j_dt*dt*dt;

          // Velocity
          v += a0*dt + 0.5*j_dt*dt;

          prev_imu_it = imu_it;

        }

        return imu_se3;

      }

      bool imuMeasFromTimeRange(uint64_t start_time, uint64_t end_time,
                                          boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
                                          boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it) 
      {

        if (m_imuBuffer.empty() || m_imuBuffer.front().stamp < end_time) {
          // Wait for the latest IMU data
          std::unique_lock<decltype(this->m_mutexImu)> lock(this->m_mutexImu);
          m_cvImuStamp.wait(lock, [this, &end_time]{ return m_imuBuffer.front().stamp >= end_time; });
        }

        auto imu_it = m_imuBuffer.begin();

        // Find last imu value in range
        auto last_imu_it = imu_it;
        imu_it++;
        while (imu_it != m_imuBuffer.end() && imu_it->stamp >= end_time) {
          last_imu_it = imu_it;
          imu_it++;
        }

        // Find first imu value in range
        while (imu_it != m_imuBuffer.end() && imu_it->stamp >= start_time) {
          imu_it++;
        }

        if (imu_it == m_imuBuffer.end()) {
          // not enough IMU measurements, return false
          return false;
        }
        imu_it++;

        // Set reverse iterators (to iterate forward in time)
        end_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(last_imu_it);
        begin_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(imu_it);

        return true;
      }

  };

}

PLUGINLIB_EXPORT_CLASS(pointcloud2_filters::DeskewImu, filters::FilterBase<sensor_msgs::msg::PointCloud2>)
