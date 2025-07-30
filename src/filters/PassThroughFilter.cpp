
#include <rclcpp/rclcpp.hpp>
#include <filters/filter_base.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>


namespace pointcloud2_filters
{
  class PassThroughFilter : public filters::FilterBase<sensor_msgs::msg::PointCloud2>
  {
    protected:

      std::unique_ptr<tf2_ros::Buffer> m_buffer;
      std::shared_ptr<tf2_ros::TransformListener> m_listener{nullptr};
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pubIntermediate;
      
      std::string m_frameId;
      std::string m_field;
      double m_waitForTfDelay;
      bool m_keepOrganized;
      bool m_invert;
      double m_min;
      double m_max;

    public:
      PassThroughFilter() { }

      bool configure()
      {
        RCLCPP_INFO(this->logging_interface_->get_logger(),"Pointcloud2PassThroughFilter configuring");

        // Setup tf2
        if(this->get_node() != nullptr)
        {
          m_buffer = std::make_unique<tf2_ros::Buffer>(this->get_node()->get_clock());
          m_listener = std::make_shared<tf2_ros::TransformListener>(*m_buffer);
        }

        this->getParam("frame_id", m_frameId, "base_link");
        this->getParam("wait_for_tf_delay", m_waitForTfDelay, 0.1);
        this->getParam("keep_organized", m_keepOrganized, false);
        this->getParam("invert", m_invert, false);        
        
        if (!this->getParam("field", m_field))
        {
          RCLCPP_ERROR(this->logging_interface_->get_logger(),"  PassThroughFilter did not find parameter 'field'.");
          return false;
        }
        
        if (!this->getParam("min", m_min))
        {
          RCLCPP_ERROR(this->logging_interface_->get_logger(),"  PassThroughFilter did not find parameter 'min'.");
          return false;
        }

        if (!this->getParam("max", m_max))
        {
          RCLCPP_ERROR(this->logging_interface_->get_logger(),"  PassThroughFilter did not find parameter 'max'.");
          return false;
        }

        RCLCPP_INFO(this->logging_interface_->get_logger(),"  Frame_id is: %s", m_frameId.c_str());
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  wait for tf delay: %f", m_waitForTfDelay);
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  Field is: %s", m_field.c_str());
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  Range: min %f, max %f", m_min, m_max);
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  Keep Organized: %s", (m_keepOrganized ? "true" : "false"));
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  Invert: %s", (m_invert ? "true" : "false"));

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

        // Check to see if we can do transforms
        if(m_buffer == nullptr)
        {
          output_msg = input_msg;
          RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "pointcloud2_filters_erdc/PassThroughFilterArray does not have access to tf data!");
        } else { 
          //transform
          if(m_frameId != "" && m_frameId != input_msg.header.frame_id)
          {
            geometry_msgs::msg::TransformStamped transform;
            try
            {
              if(m_buffer->canTransform(m_frameId, input_msg.header.frame_id, input_msg.header.stamp, rclcpp::Duration::from_seconds(m_waitForTfDelay)))
              {
                transform = m_buffer->lookupTransform(m_frameId, input_msg.header.frame_id, input_msg.header.stamp);
                
                tf2::doTransform(input_msg, output_msg, transform);
              }
              else
              {
                RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "  PassThroughFilter is waiting for transform from " << input_msg.header.frame_id << " to " << m_frameId << " to become available.");
                output_msg = input_msg;
                return false;
              }
            }
            catch (tf2::TransformException &ex) 
            {
              RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "  PassThroughFilter Thread: " << ex.what());
              output_msg = input_msg;            
              return false;
            }
          }
          else
          {
            output_msg = input_msg;
          }
        }

        // Convert to pcl
        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(output_msg, *cloud);

        // filter
        pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
        pcl::PassThrough<pcl::PCLPointCloud2> PassThroughFilter;
        PassThroughFilter.setInputCloud(cloud);
        PassThroughFilter.setKeepOrganized(m_keepOrganized);
        PassThroughFilter.setFilterFieldName(m_field);
        PassThroughFilter.setFilterLimits(m_min, m_max);
        PassThroughFilter.setNegative(m_invert);
        PassThroughFilter.filter(*cloud_filtered);
    
        // Convert to msg
        pcl_conversions::fromPCL(*cloud_filtered, output_msg);

        // publish here for debug purposes
        if(m_pubIntermediate!= nullptr && m_pubIntermediate->get_subscription_count() > 0) 
          m_pubIntermediate->publish(output_msg);

        return true;
      }

  };

}

PLUGINLIB_EXPORT_CLASS(pointcloud2_filters::PassThroughFilter, filters::FilterBase<sensor_msgs::msg::PointCloud2>)
