
#include <rclcpp/rclcpp.hpp>
#include <filters/filter_base.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>


namespace pointcloud2_filters
{
  class Transform : public filters::FilterBase<sensor_msgs::msg::PointCloud2>
  {
    protected:

      std::unique_ptr<tf2_ros::Buffer> m_buffer;
      std::shared_ptr<tf2_ros::TransformListener> m_listener{nullptr};
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pubIntermediate;

      std::string m_frameId;
      double m_waitForTfDelay;

    public:
      Transform() { }

      bool configure()
      {
        RCLCPP_INFO(this->logging_interface_->get_logger(), "Pointcloud2Transform configuring");

        // Setup tf2
        if(this->get_node() != nullptr)
        {
          m_buffer = std::make_unique<tf2_ros::Buffer>(this->get_node()->get_clock());
          m_listener = std::make_shared<tf2_ros::TransformListener>(*m_buffer);
        }

        // Get parameters
        RCLCPP_INFO(this->logging_interface_->get_logger(), "Pointcloud2Transform started");

        if (!this->getParam("frame_id", m_frameId)) 
        {
          RCLCPP_ERROR(this->logging_interface_->get_logger(),"Transform did not find parameter 'frame_id'.");
          return false;
        }

        this->getParam("wait_for_tf_delay", m_waitForTfDelay, 0.1);

        RCLCPP_INFO(this->logging_interface_->get_logger(),"  Frame_id is: %s", m_frameId.c_str());
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  wait for tf delay: %f", m_waitForTfDelay);

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

        // Check to see if tf_buffer exists
        if(m_buffer == nullptr)
        {
          RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "filters_pointcloud2_erdc/Transform does not have access to tf data!");
          
          return false;
        } 

        //transform
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
            RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "  Transform filter is waiting for transform from " << input_msg.header.frame_id << " to " << m_frameId << " to become available.");
            output_msg = input_msg;
            return false;
          }
        }
        catch (tf2::TransformException &ex) 
        {
          RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "  Publish Thread: " << ex.what());
          output_msg = input_msg;
          return false;
        }

        // publish here for debug purposes
        if(m_pubIntermediate!= nullptr && m_pubIntermediate->get_subscription_count() > 0) 
          m_pubIntermediate->publish(output_msg);

        return true;
      }

  };

}

PLUGINLIB_EXPORT_CLASS(pointcloud2_filters::Transform, filters::FilterBase<sensor_msgs::msg::PointCloud2>)
