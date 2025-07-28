
#include <rclcpp/rclcpp.hpp>
#include <filters/filter_base.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud2_iterator.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>


namespace pointcloud2_filters_erdc
{

  struct hasField
  {
    std::string key;
    hasField(const sensor_msgs::PointField& item): key(item.name) {}

    bool operator()(const sensor_msgs::PointField& field) {
        return (field.name == key);
    }

    hasField(const pcl::PCLPointField& item): key(item.name) {}

    bool operator()(const pcl::PCLPointField& field) {
        return (field.name == key);
    }
  };

  class ConvertToImage : public filters::FilterBase<sensor_msgs::msg::PointCloud2>
  {
    protected:

      std::unique_ptr<tf2_ros::Buffer> m_buffer;
      std::shared_ptr<tf2_ros::TransformListener> m_listener{nullptr};

      std::string camera_info_topic_;
      std::string color_image_topic_;
      std::string depth_image_topic_;
      std::string user_image_topic_;

      ros::Publisher info_pub_;
      ros::Publisher color_pub_;
      ros::Publisher depth_pub_;
      ros::Publisher user_pub_;
      
    
    public:
      ConvertToImage()
      : tfBuffer_()
      , tfListener_(tfBuffer_)
      {
        ConvertToImageConfig config_ = ConvertToImageConfig::__getDefault__();
      }

      bool configure()
      {
        RCLCPP_INFO(this->logging_interface_->get_logger(),"Pointcloud2ConvertToImage started");

        ros::NodeHandle pnh("~" + getName());
        dyn_server_.reset(new dynamic_reconfigure::Server<ConvertToImageConfig>(own_mutex_, pnh));
        dynamic_reconfigure::Server<ConvertToImageConfig>::CallbackType f;
        f = [this](auto& config, auto level){ reconfigureCB(config, level); };
        dyn_server_->setCallback(f);

        if (!getParam("frame_id", config_.frame_id)) 
        {
          RCLCPP_ERROR(this->logging_interface_->get_logger(),"Transform did not find parameter 'frame_id'.");
          return false;
        }

        getParam("wait_for_tf_delay", config_.wait_for_tf_delay);
        getParam("field", config_.field);
        getParam("height", config_.height);
        getParam("width", config_.width); 
        getParam("pixel_size", config_.pixel_size);   
        getParam("focal_length", config_.focal_length);       
        dyn_server_->updateConfig(config_);
        
        camera_info_topic_ = "camera_info";
        color_image_topic_ = "color_image";
        depth_image_topic_ = "depth_image";
        user_image_topic_ = "user_image";         

        getParam("camera_info_topic", camera_info_topic_);
        getParam("color_image_topic", color_image_topic_);
        getParam("depth_image_topic", depth_image_topic_);
        getParam("user_image_topic", user_image_topic_); 

        info_pub_ = pnh.template advertise<sensor_msgs::CameraInfo>(camera_info_topic_, 10);
        color_pub_ = pnh.template advertise<sensor_msgs::Image>(color_image_topic_, 10);
        depth_pub_ = pnh.template advertise<sensor_msgs::Image>(depth_image_topic_, 0);
        user_pub_ = pnh.template advertise<sensor_msgs::Image>(user_image_topic_, 10);
        
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  Frame_id is: %s", config_.frame_id.c_str());
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  wait for tf delay: %f", config_.wait_for_tf_delay);
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  field: %s", config_.field.c_str());
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  height: %d", config_.height);
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  width: %d", config_.width);
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  pixel_size: %d", config_.pixel_size);
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  focal_length: %f", config_.focal_length);

        return true;
      }

      bool update(
        const sensor_msgs::msg::PointCloud2& input_msg,
        sensor_msgs::msg::PointCloud2& output_msg)
      {
        ROS_DEBUG("Update");
        
        if (input_msg.data.size() == 0)
        {
          RCLCPP_WARN_STREAM_THROTTLE(this->logging_interface_->get_logger(), *this->logging_interface_->get_clock(), 1000, "No data in pointcloud!");
          output_msg = input_msg;
          return false;
        }

        ros::WallTime beginTime = ros::WallTime::now();

        //transform
        ROS_DEBUG("  Transform");
        geometry_msgs::msg::TransformStamped transform;
        try
        {
          if(tfBuffer_.canTransform(config_.frame_id, input_msg.header.frame_id, input_msg.header.stamp, ros::Duration(config_.wait_for_tf_delay)))
          {
            transform = tfBuffer_.lookupTransform(config_.frame_id, input_msg.header.frame_id, input_msg.header.stamp);
            
            tf2::doTransform(input_msg, output_msg, transform);
          }
          else
          {
            ROS_WARN_STREAM("  Transform filter is waiting for transform from " << input_msg.header.frame_id << " to " << config_.frame_id << " to become available.");
            output_msg = input_msg;
            return false;
          }
        }
        catch (tf2::TransformException &ex) 
        {
          ROS_WARN_STREAM("  Publish Thread: " << ex.what());
          output_msg = input_msg;
          return false;
        }

        //Sort input cloud fields by field offset
        ROS_DEBUG("  Sort Fields");
        std::vector<sensor_msgs::PointField> sortedFields(input_msg.fields);
        std::sort(sortedFields.begin(), sortedFields.end(), compareFieldsOffset);

        // Find fields we need in the cloud
        sensor_msgs::PointField xField, yField, zField, rgbField, userField;
        for (int i=0; i < sortedFields.size(); i++)
        {
          sensor_msgs::PointField currentField = sortedFields[i];
          
          if(currentField.name == "x")
          {
            xField = currentField;
            ROS_DEBUG("  Found X Field");
          }
          else if(currentField.name == "y")
          {
            yField = currentField;
            ROS_DEBUG("  Found Y Field");            
          }
          else if(currentField.name == "z")
          {
            zField = currentField;
            ROS_DEBUG("  Found Z Field");            
          }
          else if(currentField.name == "rgb" || currentField.name == "rgba")
          {
            rgbField = currentField;
            RCLCPP_DEBUG_STREAM(g_node->get_logger(),"  Found " << currentField.name);            
          }
          if(currentField.name == config_.field)
          {
            userField = currentField;
            RCLCPP_DEBUG_STREAM(g_node->get_logger(),"  Found " << currentField.name);             
          }
        }

        ROS_DEBUG("  Setup output msgs");
        // Setup camera_info
        ROS_DEBUG("    Camera Info");        
        sensor_msgs::CameraInfo info;
        info.header = output_msg.header;
        info.header.frame_id = config_.frame_id;        
        info.height = config_.height;
        info.width = config_.width;
        info.distortion_model =  "plumb_bob";
        cv::Mat distortionCoefficients = cv::Mat::zeros(1, 5, CV_64F);
        info.D = distortionCoefficients;

        // Intrinsic Matrix
        cv::Mat intrinsicMatrix = cv::Mat::zeros(3, 3, CV_64F);
        intrinsicMatrix.at<double>(0,0) = config_.focal_length;
        intrinsicMatrix.at<double>(1,1) = config_.focal_length;
        intrinsicMatrix.at<double>(0,2) = config_.width/2;
        intrinsicMatrix.at<double>(1,2) = config_.height/2;
        intrinsicMatrix.at<double>(2,2) = 1.0;
        for (int row = 0; row < 3; row++)
        {
          for (int col = 0; col < 3; col++)
          {
            info.K[row * 3 + col] = intrinsicMatrix.at<double>(row, col);
          }
        }

        // Rectification Matrix
        cv::Mat rectificationMatrix = cv::Mat::zeros(3, 3, CV_64F);
        rectificationMatrix.at<double>(0,0) = 1.0;
        rectificationMatrix.at<double>(1,1) = 1.0;
        rectificationMatrix.at<double>(2,2) = 1.0;        
        for (int row = 0; row < 3; row++)
        {
          for (int col = 0; col < 3; col++)
          {
            info.R[row * 3 + col] = rectificationMatrix.at<double>(row, col);
          }
        }        

        // Projection Matrix
        cv::Mat projectionMatrix = cv::Mat::zeros(3, 4, CV_64F);        
        projectionMatrix.at<double>(0,0) = config_.focal_length;
        projectionMatrix.at<double>(1,1) = config_.focal_length;
        projectionMatrix.at<double>(0,2) = config_.width/2;
        projectionMatrix.at<double>(1,2) = config_.height/2;
        projectionMatrix.at<double>(2,2) = 1.0;        
        for (int row = 0; row < 3; row++)
        {
          for (int col = 0; col < 4; col++)
          {
            info.P[row * 4 + col] = projectionMatrix.at<double>(row, col);
          }
        }

        // Setup color image
        ROS_DEBUG("    Color");        
        cv_bridge::CvImage colorImage;
        colorImage.header = output_msg.header;
        colorImage.header.frame_id = config_.frame_id;
        colorImage.encoding = sensor_msgs::image_encodings::BGR8;
        colorImage.image = cv::Mat::zeros(config_.height, config_.width, CV_8UC3);

        // Setup depth image
        ROS_DEBUG("    Depth");           
        cv_bridge::CvImage depthImage;
        depthImage.header = output_msg.header;
        depthImage.header.frame_id = config_.frame_id;
        depthImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        depthImage.image = cv::Mat::zeros(config_.height, config_.width, CV_32FC1);

        // Setup user image
        ROS_DEBUG("    User");         
        RCLCPP_DEBUG_STREAM(g_node->get_logger(),"    User-Datatype: " << +userField.datatype);  
        cv_bridge::CvImage userImage;
        userImage.header = output_msg.header;
        userImage.header.frame_id = config_.frame_id;
        if((userField.datatype == sensor_msgs::PointField::UINT8   && userField.count == 4) ||
           (userField.datatype == sensor_msgs::PointField::UINT32  && (userField.name == "rgb" || userField.name == "rgba")) ||
           (userField.datatype == sensor_msgs::PointField::FLOAT32 && (userField.name == "rgb" || userField.name == "rgba"))
          )
        {
          ROS_DEBUG("      4 8-bit unsigned integers");       
          userImage.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
          userImage.image = cv::Mat::zeros(config_.height, config_.width, CV_8UC3);
        }
        else if((userField.datatype == sensor_msgs::PointField::UINT16  && userField.count == 4) ||
                (userField.datatype == sensor_msgs::PointField::FLOAT64 && (userField.name == "rgb" || userField.name == "rgba"))
               )
        {
          ROS_DEBUG("      4 16-bit unsigned integers");    
          userImage.encoding = sensor_msgs::image_encodings::TYPE_16UC3;
          userImage.image = cv::Mat::zeros(config_.height, config_.width, CV_16UC3);
        }
        else if((userField.datatype == sensor_msgs::PointField::UINT8))
        {
          ROS_DEBUG("      1 8-bit unsigned integer");    
          userImage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
          userImage.image = cv::Mat::zeros(config_.height, config_.width, CV_8UC1);          
        }
        else if((userField.datatype == sensor_msgs::PointField::INT8))
        {
          ROS_DEBUG("      1 8-bit signed integer");             
          userImage.encoding = sensor_msgs::image_encodings::TYPE_8SC1;
          userImage.image = cv::Mat::zeros(config_.height, config_.width, CV_8SC1);            
        }        
        else if((userField.datatype == sensor_msgs::PointField::UINT16))
        {
          ROS_DEBUG("      1 16-bit unsigned integer");               
          userImage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
          userImage.image = cv::Mat::zeros(config_.height, config_.width, CV_16UC1);            
        }
        else if((userField.datatype == sensor_msgs::PointField::INT16))
        {
          ROS_DEBUG("      1 16-bit signed integer");               
          userImage.encoding = sensor_msgs::image_encodings::TYPE_16SC1;
          userImage.image = cv::Mat::zeros(config_.height, config_.width, CV_16SC1);  
        }
        else if((userField.datatype == sensor_msgs::PointField::FLOAT32))
        {
          ROS_DEBUG("      1 32-bit float");     
          userImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
          userImage.image = cv::Mat::zeros(config_.height, config_.width, CV_32FC1);            
        }        
        else if((userField.datatype == sensor_msgs::PointField::FLOAT64))
        {
          ROS_DEBUG("      1 64-bit float");               
          userImage.encoding = sensor_msgs::image_encodings::TYPE_64FC1;
          userImage.image = cv::Mat::zeros(config_.height, config_.width, CV_64FC1);  
        }        
        else
        {
          return false;
        }

        if(info_pub_.getNumSubscribers()  > 0 || 
           color_pub_.getNumSubscribers() > 0 ||
           depth_pub_.getNumSubscribers() > 0 ||
           user_pub_.getNumSubscribers()  > 0)
        {
          createImages(output_msg, depthImage, 
                       colorImage, userImage, 
                       info, intrinsicMatrix, distortionCoefficients,
                       xField, yField, zField, rgbField, userField                        
                      );
        }

        ROS_DEBUG("  Publish");
        if(info_pub_.getNumSubscribers() > 0) info_pub_.publish(info);
        if(color_pub_.getNumSubscribers() > 0) color_pub_.publish(colorImage);
        if(depth_pub_.getNumSubscribers() > 0) depth_pub_.publish(depthImage);
        if(user_pub_.getNumSubscribers() > 0) user_pub_.publish(userImage);

        // Check timer
        ros::WallTime endTime = ros::WallTime::now();
        ros::WallDuration timer = beginTime - endTime;
        RCLCPP_DEBUG_STREAM(g_node->get_logger(),getName() << " timer: " << timer);

        return true;
      }

      void createImages(sensor_msgs::msg::PointCloud2& output_msg, cv_bridge::CvImage& depthImage, 
                        cv_bridge::CvImage& colorImage, cv_bridge::CvImage& userImage, 
                        sensor_msgs::CameraInfo& info, cv::Mat& intrinsicMatrix, cv::Mat& distortionCoefficients,
                        sensor_msgs::PointField& xField, sensor_msgs::PointField& yField, sensor_msgs::PointField& zField,
                        sensor_msgs::PointField& rgbField, sensor_msgs::PointField& userField                  
                       )
      {
        // Setup OpenCV matrixes
        ROS_DEBUG("  Setup opencv matrixes");        
        cv::Mat rvec = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
        cv::Mat tvec = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
        std::vector<cv::Point3f> obj_pts;
        std::vector<cv::Point2f> img_pts;

        // #pragma omp parallel for
        int size = output_msg.height * output_msg.width;
        RCLCPP_DEBUG_STREAM(g_node->get_logger(),"  Create depth image for opencv: " << size);
        for (int i = 0; i < size; ++i)
        {
          float X,Y,Z;
          std::memcpy(&X, &output_msg.data[i * output_msg.point_step + xField.offset],sizeof(sizeOfPointField(yField.datatype)));
          std::memcpy(&Y, &output_msg.data[i * output_msg.point_step + yField.offset],sizeof(sizeOfPointField(yField.datatype)));
          std::memcpy(&Z, &output_msg.data[i * output_msg.point_step + zField.offset],sizeof(sizeOfPointField(zField.datatype)));
          obj_pts.push_back(cv::Point3f(X, Y, Z));
        }
        
        cv::projectPoints(obj_pts, rvec, tvec, intrinsicMatrix, distortionCoefficients, img_pts);

        // #pragma omp parallel for
        // Loop through points
        RCLCPP_DEBUG_STREAM(g_node->get_logger(),"  Loop through points: " << img_pts.size());        
        for (size_t i = 0; i < img_pts.size(); ++i)
        {
          int u = int(img_pts[i].x);
          int v = int(img_pts[i].y);
          
          // Check if point is inside fov of camera
          if ((u >= 0) && (u < depthImage.image.cols) &&  
              (v >= 0) && (v < depthImage.image.rows) &&
              obj_pts[i].z > 0)
          {
            // add to depth image
            if(depthImage.image.at<float>(v, u) <= 0.001 || 
              depthImage.image.at<float>(v, u) > obj_pts[i].z)
            {
              // color values
              uint8_t b,g,r;
              std::memcpy(&b, &output_msg.data[i * output_msg.point_step + rgbField.offset + 0],sizeof(uint8_t));
              std::memcpy(&g, &output_msg.data[i * output_msg.point_step + rgbField.offset + 1],sizeof(uint8_t));
              std::memcpy(&r, &output_msg.data[i * output_msg.point_step + rgbField.offset + 2],sizeof(uint8_t));              
              
              // draw box around each pixel based on pixel_size param
              int shift = config_.pixel_size / 2;
              int lowerIndex1 = v - shift;
              if(lowerIndex1 < 0) lowerIndex1 = 0;
              int upperIndex1 = v + shift;
              if(upperIndex1 > config_.height - 1) upperIndex1 = config_.height - 1; 
              int lowerIndex2 = u - shift; 
              if(lowerIndex2 < 0) lowerIndex2 = 0;                           
              int upperIndex2 = u + shift;
              if(upperIndex2 > config_.width - 1) upperIndex2 = config_.width - 1;
              for(int j = lowerIndex1; j <= upperIndex1; j++)
              {
                for(int k = lowerIndex2; k <= upperIndex2; k++)
                {
                  // depth image
                  depthImage.image.at<float>(j, k) = obj_pts[i].z;
                
                  // color image
                  cv::Vec3b bgr_pixel;
                  bgr_pixel[0] = b;
                  bgr_pixel[1] = g;
                  bgr_pixel[2] = r;
                  colorImage.image.at<cv::Vec3b>(j, k) = bgr_pixel;
                  // user image
                  if((userField.datatype == sensor_msgs::PointField::UINT8   && userField.count == 4) ||
                    (userField.datatype == sensor_msgs::PointField::UINT32  && (userField.name == "rgb" || userField.name == "rgba")) ||
                    (userField.datatype == sensor_msgs::PointField::FLOAT32 && (userField.name == "rgb" || userField.name == "rgba"))
                    )
                  {
                    uint8_t user1, user2, user3;
                    std::memcpy(&user1, &output_msg.data[i * output_msg.point_step + userField.offset + 0],sizeof(uint8_t));
                    std::memcpy(&user2, &output_msg.data[i * output_msg.point_step + userField.offset + 1],sizeof(uint8_t));
                    std::memcpy(&user3, &output_msg.data[i * output_msg.point_step + userField.offset + 2],sizeof(uint8_t));
                    cv::Vec3b pixel;
                    pixel[0] = user1;
                    pixel[1] = user2;
                    pixel[2] = user3;
                    userImage.image.at<cv::Vec3b>(j, k) = pixel;
                  }
                  else if((userField.datatype == sensor_msgs::PointField::UINT16  && userField.count == 4) ||
                          (userField.datatype == sensor_msgs::PointField::FLOAT64 && (userField.name == "rgb" || userField.name == "rgba"))
                        )
                  {
                    uint16_t user1, user2, user3;
                    std::memcpy(&user1, &output_msg.data[i * output_msg.point_step + userField.offset + 0],sizeof(uint16_t));
                    std::memcpy(&user2, &output_msg.data[i * output_msg.point_step + userField.offset + 1],sizeof(uint16_t));
                    std::memcpy(&user3, &output_msg.data[i * output_msg.point_step + userField.offset + 2],sizeof(uint16_t));
                    cv::Vec3w bgr_pixel;
                    bgr_pixel[0] = user1;
                    bgr_pixel[1] = user2;
                    bgr_pixel[2] = user3;
                    userImage.image.at<cv::Vec3w>(j, k) = bgr_pixel;
                  }
                  else if((userField.datatype == sensor_msgs::PointField::UINT8))
                  {
                    
                    uint8_t value;
                    std::memcpy(&value, &output_msg.data[i * output_msg.point_step + userField.offset],sizeof(uint8_t));
                    userImage.image.at<uint8_t>(j, k) = value;      
                  }
                  else if((userField.datatype == sensor_msgs::PointField::INT8))
                  {
                    int8_t value;
                    std::memcpy(&value, &output_msg.data[i * output_msg.point_step + userField.offset],sizeof(int8_t));
                    userImage.image.at<int8_t>(j, k) = value;         
                  }        
                  else if((userField.datatype == sensor_msgs::PointField::UINT16))
                  {
                    uint16_t value;
                    std::memcpy(&value, &output_msg.data[i * output_msg.point_step + userField.offset],sizeof(uint16_t));
                    userImage.image.at<uint16_t>(j, k) = value;          
                  }
                  else if((userField.datatype == sensor_msgs::PointField::INT16))
                  {
                    int16_t value;
                    std::memcpy(&value, &output_msg.data[i * output_msg.point_step + userField.offset],sizeof(int16_t));
                    userImage.image.at<int16_t>(j, k) = value;
                  }
                  else if((userField.datatype == sensor_msgs::PointField::FLOAT32))
                  {
                    float value;
                    std::memcpy(&value, &output_msg.data[i * output_msg.point_step + userField.offset],sizeof(float));
                    userImage.image.at<float>(j, k) = value;         
                  }        
                  else if((userField.datatype == sensor_msgs::PointField::FLOAT64))
                  {
                    double value;
                    std::memcpy(&value, &output_msg.data[i * output_msg.point_step + userField.offset],sizeof(double));
                    userImage.image.at<double>(j, k) = value;
                  }  
                }   
              } 
            }
          }
        }

      }

      static bool compareFieldsOffset(sensor_msgs::PointField& field1, sensor_msgs::PointField& field2)
      {
        return (field1.offset < field2.offset);
      }

  };

}

PLUGINLIB_EXPORT_CLASS(pointcloud2_filters_erdc::ConvertToImage, filters::FilterBase<sensor_msgs::msg::PointCloud2>)
