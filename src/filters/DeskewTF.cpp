
#include <rclcpp/rclcpp.hpp>
#include <filters/filter_base.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>


namespace pointcloud2_filters
{
  class DeskewTF : public filters::FilterBase<sensor_msgs::msg::PointCloud2>
  {
    protected:

      std::unique_ptr<tf2_ros::Buffer> m_buffer;
      std::shared_ptr<tf2_ros::TransformListener> m_listener{nullptr};
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pubIntermediate;

      std::string m_mapFrameId;
      std::string m_field;
      double m_waitForTfDelay;      
      double m_conversion;
      double m_frequency;

      
    public:
      DeskewTF() { }

      bool configure()
      {
        RCLCPP_INFO(this->logging_interface_->get_logger(),"Pointcloud2DeskewTF configuring");

        // Setup tf2
        if(this->get_node() != nullptr)
        {
          m_buffer = std::make_unique<tf2_ros::Buffer>(this->get_node()->get_clock());
          m_listener = std::make_shared<tf2_ros::TransformListener>(*m_buffer);
        }

        getParam("map_frame_id", m_mapFrameId, "odom");
        getParam("field", m_field, "time");
        getParam("conversion", m_conversion, 1.0);
        getParam("frequency", m_frequency, 10.0);
        getParam("wait_for_tf_delay", m_waitForTfDelay, 0.1);

        RCLCPP_INFO(this->logging_interface_->get_logger(),"  map_frame_id is: %s", m_mapFrameId.c_str());
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  field is: %s", m_field.c_str());
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  conversion factor is: %f", m_conversion);
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  frequency is: %f", m_frequency);
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  wait_for_tf_delay: %f", m_waitForTfDelay);

        // Create publisher
        if(this->get_node() != nullptr)
            m_pubIntermediate = this->get_node()->create_publisher<sensor_msgs::msg::PointCloud2>("output", 10);

        return true;
      }

      bool update(
        const sensor_msgs::msg::PointCloud2& input_msg,
        sensor_msgs::msg::PointCloud2& output_msg)
      {
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

        output_msg = input_msg;

        // Check to see if we can do transforms
        if(m_buffer == nullptr)
        {
          RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "filters_pointcloud2_erdc/DeskewTf does not have access to tf data!");
        } else {         
          //transform
          try
          { 
            rclcpp::Time startTime = output_msg.header.stamp;
            rclcpp::Time endTime = output_msg.header.stamp + rclcpp::Duration::from_seconds(1 / m_frequency);
            if(m_buffer->canTransform(m_mapFrameId, output_msg.header.frame_id, endTime, rclcpp::Duration::from_seconds(m_waitForTfDelay)))
            {
              geometry_msgs::msg::TransformStamped startTransform, endTransform;
              startTransform = m_buffer->lookupTransform(m_mapFrameId, output_msg.header.frame_id, startTime);
              endTransform = m_buffer->lookupTransform(m_mapFrameId, output_msg.header.frame_id, endTime);
              
              Eigen::Affine3d startPose = tf2::transformToEigen(startTransform);
              Eigen::Affine3d endPose = tf2::transformToEigen(endTransform);

              // setup iterators
              sensor_msgs::PointCloud2Iterator<float> it_x(output_msg, "x");
              sensor_msgs::PointCloud2Iterator<float> it_y(output_msg, "y");
              sensor_msgs::PointCloud2Iterator<float> it_z(output_msg, "z");
              sensor_msgs::PointCloud2Iterator<uint32_t> it_t(output_msg, m_field);

              // transform to static frame at time for each point
              for (; it_x != it_x.end() && it_y != it_y.end() && it_z != it_z.end() && it_t != it_t.end(); ++it_x, ++it_y, ++it_z, ++it_t)
              {
                Eigen::Vector3f point;
                point[0] = *it_x;
                point[1] = *it_y;
                point[2] = *it_z;
                uint32_t time = *it_t;
                
                // Get Transform
                rclcpp::Duration timeShift = rclcpp::Duration::from_nanoseconds((uint32_t)(m_conversion * (double)time));
                rclcpp::Time timePoint(output_msg.header.stamp);
                timePoint += timeShift;
                Eigen::Matrix4d transform = interpolateTransform(startPose, endPose, startTime.nanoseconds(), endTime.nanoseconds(), timePoint.nanoseconds());
                Eigen::Affine3f affine(transform.cast<float>());
                point = affine * point;

                *it_x = point[0];
                *it_y = point[1];
                *it_z = point[2];
                *it_t = 0.0;
              }
              //transform back to sensor frame at time in header
              geometry_msgs::msg::TransformStamped transform;
              transform = m_buffer->lookupTransform(output_msg.header.frame_id, m_mapFrameId, output_msg.header.stamp);
              
              tf2::doTransform(output_msg, output_msg, transform);
            }
            else
            {
              RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "  DeskewTF is waiting for transform from " << input_msg.header.frame_id << " to " << m_mapFrameId << " to become available.");
              return false;
            }
          }
          catch (tf2::TransformException &ex) 
          {
            RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "  DeskewTF Thread: " << ex.what());
            output_msg = input_msg;
            return false;
          }
        }

        // publish here for debug purposes
        if(m_pubIntermediate!= nullptr && m_pubIntermediate->get_subscription_count() > 0) 
          m_pubIntermediate->publish(output_msg);

        return true;
      }

      geometry_msgs::msg::TransformStamped interpolateTransform(const geometry_msgs::msg::TransformStamped &startTransform, const geometry_msgs::msg::TransformStamped &endTransform, const rclcpp::Time time)
      {
        geometry_msgs::msg::TransformStamped output;

        //Calculate the ratio
        rclcpp::Time startTime(startTransform.header.stamp);
        rclcpp::Time endTime(endTransform.header.stamp);
        tf2Scalar ratio = (time - startTime).seconds() / (endTime - startTime).seconds();        

        //Interpolate rotation
        tf2::Quaternion rotation, rotationStart, rotationEnd;
        tf2::convert(startTransform.transform.rotation, rotationStart);
        tf2::convert(endTransform.transform.rotation, rotationEnd);
        rotation = tf2::slerp(rotationStart, rotationEnd, ratio);
        tf2::convert(rotation, output.transform.rotation);

        //Interpolate translation
        tf2::Vector3 translation, translationStart, translationEnd;
        tf2::convert(startTransform.transform.translation, translationStart);
        tf2::convert(endTransform.transform.translation, translationEnd);
        translation.setInterpolate3(translationStart, translationEnd, ratio);
        tf2::convert(translation, output.transform.translation);

        output.header.stamp = time;
        output.header.frame_id = startTransform.header.frame_id;
        output.child_frame_id = startTransform.child_frame_id;

        return output;
      }

      Eigen::Matrix4d interpolateTransform(const Eigen::Affine3d &startPose, const Eigen::Affine3d &endPose, rcl_time_point_value_t startTime, rcl_time_point_value_t endTime, rcl_time_point_value_t time)
      {
        //Calculate the ratio
        double ratio1 = (double)(time - startTime) / (double)(endTime - startTime);
        double ratio2 = 1.0 - ratio1;

        // Interpolate
        Eigen::Quaterniond q1(startPose.rotation()); 
        Eigen::Quaterniond q2(endPose.rotation());
        Eigen::Quaterniond q = q1.slerp(ratio1, q2);
    
        Eigen::Vector3d p = startPose.translation() * ratio2 + endPose.translation() * ratio1;
        
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block(0, 0, 3, 3) = q.toRotationMatrix();
        T.block(0, 3, 3, 1) = p;

        return T;
      }

  };

}

PLUGINLIB_EXPORT_CLASS(pointcloud2_filters::DeskewTF, filters::FilterBase<sensor_msgs::msg::PointCloud2>)
