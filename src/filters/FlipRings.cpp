
#include <rclcpp/rclcpp.hpp>
#include <filters/filter_base.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>


namespace pointcloud2_filters_erdc
{
  class FlipRings : public filters::FilterBase<sensor_msgs::msg::PointCloud2>
  {
    protected:

      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pubIntermediate;

      int m_numRings;
      std::string m_field;

    public:
      FlipRings() { }

      bool configure()
      {
        RCLCPP_INFO(this->logging_interface_->get_logger(),"Pointcloud2FlipRings started");

        this->getParam("field", m_field, "ring");
        this->getParam("num_rings", m_numRings, 64);

        RCLCPP_INFO(this->logging_interface_->get_logger(),"  field: %s", m_field.c_str());
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  num_rings: %d", m_numRings);

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

        // Flip the rings in the point cloud
        output_msg = input_msg;
        try
        {
          sensor_msgs::PointCloud2Iterator<uint16_t> it(output_msg, m_field);
          for (; it != it.end(); ++it)
          {
            if ( *it  > m_numRings )
            {
                RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "Skipping ring flipping operation: current ring value is larger than num_rings: " << m_field << " : " << *it << " > " << m_numRings );
                break;
            }

            *it = m_numRings - *it - 1;
          }
        }
        catch(const std::exception& e)
        {
          RCLCPP_WARN_STREAM_THROTTLE(this->logging_interface_->get_logger(), *this->get_node()->get_clock(), 1000, "Failed to find pointfield! :" << e.what());
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

PLUGINLIB_EXPORT_CLASS(pointcloud2_filters_erdc::FlipRings, filters::FilterBase<sensor_msgs::msg::PointCloud2>)
