
#include <rclcpp/rclcpp.hpp>
#include <filters/filter_base.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>


namespace pointcloud2_filters
{
  class RemoveBlackPoints : public filters::FilterBase<sensor_msgs::msg::PointCloud2>
  {
    protected:

      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pubIntermediate;

      std::string m_field;

      // dynamic reconfigure
    
    public:
      RemoveBlackPoints() { }

      bool configure()
      {
        RCLCPP_INFO(this->logging_interface_->get_logger(),"Pointcloud2RemoveBlackPoints configuring");

        this->getParam("field",m_field, "rgba");

        RCLCPP_INFO(this->logging_interface_->get_logger(),"  field: %s", m_field.c_str());
        
        // Create debug publisher
        if(this->get_node() != nullptr)
        {
            m_pubIntermediate = this->get_node()->create_publisher<sensor_msgs::msg::PointCloud2>(this->getName(), 10);
        }

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

        // Remove Black Points
        output_msg = input_msg;
        try
        {
            output_msg.is_dense = false;

            sensor_msgs::PointCloud2Iterator<float> it_x (output_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> it_y (output_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> it_z (output_msg, "z");

            sensor_msgs::PointCloud2Iterator<uint8_t> it_r(output_msg, "r");
            sensor_msgs::PointCloud2Iterator<uint8_t> it_g(output_msg, "g");
            sensor_msgs::PointCloud2Iterator<uint8_t> it_b(output_msg, "b");

            // #pragma omp parallel for
            for (int j = 0; j < (output_msg.height * output_msg.width); ++j, ++it_x, ++it_y, ++it_z, ++it_r, ++it_g, ++it_b)
            {
              uint8_t r = *it_r;
              uint8_t g = *it_g;
              uint8_t b = *it_b;
              if (r == (uint8_t)0 && g == (uint8_t)0 && b == (uint8_t)0)
              {
                *it_x = std::numeric_limits<float>::quiet_NaN();
                *it_y = std::numeric_limits<float>::quiet_NaN();
                *it_z = std::numeric_limits<float>::quiet_NaN();
              }
            }
        }
        catch(const std::exception& e)
        {
          RCLCPP_WARN_STREAM_THROTTLE(this->logging_interface_->get_logger(), *this->get_node()->get_clock(), 1000, "Failed to find pointfield! :" << e.what());
          return false;
        }

        // publish here for debug purposes
        if(m_pubIntermediate!= nullptr && m_pubIntermediate->get_subscription_count() > 0) 
          m_pubIntermediate->publish(output_msg);

        return true;
      }

  };

}

PLUGINLIB_EXPORT_CLASS(pointcloud2_filters::RemoveBlackPoints, filters::FilterBase<sensor_msgs::msg::PointCloud2>)
