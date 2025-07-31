
#include <rclcpp/rclcpp.hpp>
#include <filters/filter_base.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>


namespace pointcloud2_filters
{
  class FieldValueFilter_Integer : public filters::FilterBase<sensor_msgs::msg::PointCloud2>
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
      int m_min;
      int m_max;

    public:
      FieldValueFilter_Integer() { }

      bool configure()
      {
        RCLCPP_INFO(this->logging_interface_->get_logger(),"Pointcloud2FieldValueFilter_Integer configuring");

        // Setup tf2
        if(this->get_node() != nullptr)
        {
          m_buffer = std::make_unique<tf2_ros::Buffer>(this->get_node()->get_clock());
          m_listener = std::make_shared<tf2_ros::TransformListener>(*m_buffer);
        }

        this->getParam("frame_id", m_frameId, "base_link");
        this->getParam("wait_for_tf_delay", m_waitForTfDelay, 0.1);
        this->getParam("keep_organized", m_keepOrganized);
        this->getParam("invert", m_invert, false);    

        if (!this->getParam("field", m_field))
        {
          RCLCPP_ERROR(this->logging_interface_->get_logger(),"  FieldValueFilter_Integer did not find parameter 'field'.");
          return false;
        }
        
        if (!this->getParam("min", m_min))
        {
          RCLCPP_ERROR(this->logging_interface_->get_logger(),"  FieldValueFilter_Integer did not find parameter 'min'.");
          return false;
        }

        if (!this->getParam("max", m_max))
        {
          RCLCPP_ERROR(this->logging_interface_->get_logger(),"  FieldValueFilter_Integer did not find parameter 'max'.");
          return false;
        }
        
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  Frame_id is: %s", m_frameId.c_str());
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  wait for tf delay: %f", m_waitForTfDelay);
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  Field is: %s", m_field.c_str());
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  Range: min %d, max %d", m_min, m_max);
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  Keep Organized: %s", (m_keepOrganized ? "true" : "false"));
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  Invert: %s", (m_invert ? "true" : "false"));

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

        // Check to see if we can do transforms
        if(m_buffer == nullptr)
        {
          output_msg = input_msg;
          RCLCPP_WARN_STREAM(this->logging_interface_->get_logger(), "filters_pointcloud2_erdc/PassThroughFilterArray does not have access to tf data!");
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

        // Remove Points
        try
        {
          output_msg.is_dense = false;

          // Sort input cloud fields by field offset
          std::vector<sensor_msgs::msg::PointField> sortedFields(output_msg.fields);
          std::sort(sortedFields.begin(), sortedFields.end(), compareFieldsOffset);

          // Copy data into cloud
          for (int i=0; i < sortedFields.size(); i++)
          {
            //Setup input field values
            std::string name = sortedFields[i].name;
            int datatype = sortedFields[i].datatype;
            int offset = sortedFields[i].offset;

            //Extract Indices
            if(name == m_field)
            {
              // std::vector<pcl::index_t> indices;
              pcl::IndicesPtr indices(new pcl::Indices());
              for(uint32_t j = 0; j < output_msg.height; j++)
              {
                for(uint32_t k = 0; k < output_msg.width; k++)
                {
                  uint32_t input_begin = j * output_msg.row_step + k * output_msg.point_step + offset;

                  switch (datatype)
                  {
                  case sensor_msgs::msg::PointField::INT8 :
                    int8_t value1;
                    std::memcpy(&value1, &output_msg.data[input_begin], sizeof(int8_t));                
                    if(m_min <= value1 && value1 <= m_max) indices->push_back(j * output_msg.width + k);
                    break;
                  case sensor_msgs::msg::PointField::INT16 :
                    int16_t value3;
                    std::memcpy(&value3, &output_msg.data[input_begin], sizeof(int16_t));                   
                    if(m_min <= value3 && value3 <= m_max) indices->push_back(j * output_msg.width + k);
                    break;
                  case sensor_msgs::msg::PointField::INT32 :
                    int32_t value5;
                    std::memcpy(&value5, &output_msg.data[input_begin], sizeof(int32_t));     
                    if(m_min <= value5 && value5 <= m_max) indices->push_back(j * output_msg.width + k);         
                    break;
                  case sensor_msgs::msg::PointField::UINT8 :
                    uint8_t value2;                
                    std::memcpy(&value2, &output_msg.data[input_begin], sizeof(uint8_t));     
                    if(m_min <= value2 && value2 <= m_max) indices->push_back(j * output_msg.width + k);
                    break;
                  case sensor_msgs::msg::PointField::UINT16 :
                    uint16_t value4;
                    std::memcpy(&value4, &output_msg.data[input_begin], sizeof(uint16_t));     
                    if(m_min <= value4 && value4 <= m_max) indices->push_back(j * output_msg.width + k);           
                    break;
                  case sensor_msgs::msg::PointField::UINT32 :
                    uint32_t value6;
                    std::memcpy(&value6, &output_msg.data[input_begin], sizeof(uint32_t));     
                    if(m_min <= value6 && value6 <= m_max) indices->push_back(j * output_msg.width + k);
                    break;
                  case sensor_msgs::msg::PointField::FLOAT32 :
                    _Float32 value7;
                    std::memcpy(&value7, &output_msg.data[input_begin], sizeof(_Float32));     
                    if(m_min <= value7 && value7 <= m_max) indices->push_back(j * output_msg.width + k);
                    break;
                  case sensor_msgs::msg::PointField::FLOAT64 :
                    _Float64 value8;
                    std::memcpy(&value8, &output_msg.data[input_begin], sizeof(_Float64));     
                    if(m_min <= value8 && value8 <= m_max) indices->push_back(j * output_msg.width + k);
                    break;                
                  }
                }
              }

              // ROS_WARN_STREAM("  index size:" << indices->size());

              // Convert to pcl
              pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
              pcl_conversions::toPCL(output_msg, *cloud);

              // Filter
              pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
              pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
              extract.setInputCloud(cloud);
              extract.setKeepOrganized(m_keepOrganized);
              extract.setIndices(indices);
              extract.setNegative(m_invert);
              extract.filter(*cloud_filtered);

              // Convert to msg
              pcl_conversions::fromPCL(*cloud_filtered, output_msg);

              // ROS_WARN_STREAM("output_msg size:" << output_msg.height << "x" << output_msg.width);

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

      static bool compareFieldsOffset(sensor_msgs::msg::PointField& field1, sensor_msgs::msg::PointField& field2)
      {
        return (field1.offset < field2.offset);
      }

  };

}

PLUGINLIB_EXPORT_CLASS(pointcloud2_filters::FieldValueFilter_Integer, filters::FilterBase<sensor_msgs::msg::PointCloud2>)
