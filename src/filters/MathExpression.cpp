
#include <rclcpp/rclcpp.hpp>


#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_eigen/tf2_eigen.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <pointcloud2_filters_erdc/MathExpressionConfig.h>

#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filters/filter_base.hpp>
#include "pluginlib/class_list_macros.hpp"

#include "EigenLab/EigenLab.h"
#include <Eigen/Eigen>


namespace pointcloud2_filters
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
  
  class MathExpression : public filters::FilterBase<sensor_msgs::msg::PointCloud2>
  {
    protected:

      boost::recursive_mutex own_mutex_;

      ros::Publisher intermediate_pub_;
      
      // tf2
      tf2_ros::Buffer tfBuffer_;
      tf2_ros::TransformListener tfListener_;

      // dynamic reconfigure
      MathExpressionConfig config_;
      std::shared_ptr<dynamic_reconfigure::Server<MathExpressionConfig>> dyn_server_;

      void reconfigureCB(MathExpressionConfig& config, uint32_t level)
      {
        config_ = config;
      };
    
    public:
      MathExpression()
      : tfBuffer_()
      , tfListener_(tfBuffer_)
      {
        MathExpressionConfig config_ = MathExpressionConfig::__getDefault__();
      }

      bool configure()
      {
        RCLCPP_INFO(this->logging_interface_->get_logger(),"Pointcloud2MathExpression configuring");

        ros::NodeHandle pnh("~" + getName());
        dyn_server_.reset(new dynamic_reconfigure::Server<MathExpressionConfig>(own_mutex_, pnh));
        dynamic_reconfigure::Server<MathExpressionConfig>::CallbackType f;
        f = [this](auto& config, auto level){ reconfigureCB(config, level); };
        dyn_server_->setCallback(f);

        getParam("frame_id", config_.frame_id);
        getParam("wait_for_tf_delay", config_.wait_for_tf_delay);
        
        if (!getParam(std::string("expression"), config_.expression)) 
        {
          RCLCPP_ERROR(this->logging_interface_->get_logger(),"  MathExpression did not find parameter 'expression'.");
          return false;
        }

        if (!getParam("output_field", config_.output_field)) 
        {
          RCLCPP_ERROR(this->logging_interface_->get_logger(),"  MathExpression did not find parameter 'output_field'.");
          return false;
        }
       
        dyn_server_->updateConfig(config_);

        intermediate_pub_ = pnh.template advertise<sensor_msgs::msg::PointCloud2>("output", 10);
        
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  Frame_id is: %s", config_.frame_id.c_str());
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  wait for tf delay: %f", config_.wait_for_tf_delay);
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  Expression is: %s", config_.expression.c_str());
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  Output Field: %s", config_.output_field.c_str());

        return true;
      }

      bool update(
        const sensor_msgs::msg::PointCloud2& input_msg,
        sensor_msgs::msg::PointCloud2& output_msg)
      {
        if (input_msg.data.size() == 0)
        {
          RCLCPP_WARN_STREAM_THROTTLE(this->logging_interface_->get_logger(), *this->logging_interface_->get_clock(), 1000, "No data in pointcloud!");
          output_msg = input_msg;
          return false;
        }

        ros::WallTime beginTime = ros::WallTime::now();

        //transform
        if(config_.frame_id != "" && config_.frame_id != input_msg.header.frame_id)
        {
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
              ROS_WARN_STREAM("  Publisher is waiting for transform from " << input_msg.header.frame_id << " to " << config_.frame_id << " to become available.");
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
        }
        else
        {
          output_msg = input_msg;
        }

        // Create new field
        sensor_msgs::PointField newField;
        newField.count = 1;
        newField.name = config_.output_field;
        newField.datatype = sensor_msgs::PointField::FLOAT32;
        appendPointCloud2Fields(input_msg, output_msg, newField);

        // process expression
        EigenLab::Parser<Eigen::MatrixXf> parser;
        PointCloud2Eigen(output_msg, parser);
        EigenLab::Value<Eigen::MatrixXf> result = parser.eval(config_.expression);
        Eigen2PointCloud(newField.name, result.matrix(), output_msg);
    
        // publish here for debug purposes
        if(intermediate_pub_.getNumSubscribers() > 0) intermediate_pub_.publish(output_msg);

        // Check timer
        ros::WallTime endTime = ros::WallTime::now();
        ros::WallDuration timer = beginTime - endTime;
        RCLCPP_DEBUG_STREAM(g_node->get_logger(),getName() << " timer: " << timer);

        return true;
      }

      static bool compareFieldsOffset(sensor_msgs::PointField& field1, sensor_msgs::PointField& field2)
      {
        return (field1.offset < field2.offset);
      }

      void PointCloud2Eigen(const sensor_msgs::msg::PointCloud2 &cloud, EigenLab::Parser<Eigen::MatrixXf> &parser)
      {
        // Sort input cloud fields by field offset
        std::vector<sensor_msgs::PointField> sortedFields(cloud.fields);
        std::sort(sortedFields.begin(), sortedFields.end(), compareFieldsOffset);

        // Copy data into cloud
        for (int i=0; i < sortedFields.size(); i++)
        {
          //Setup input field values
          std::string name = sortedFields[i].name;
          int datatype = sortedFields[i].datatype;
          int offset = sortedFields[i].offset;

          //Copy data
          Eigen::MatrixXf eMatrix(cloud.height, cloud.width);
          for(uint32_t j = 0; j < cloud.height; j++)
          {
            for(uint32_t k = 0; k < cloud.width; k++)
            {
              uint32_t input_begin = j * cloud.row_step + k * cloud.point_step + offset;
              //copy
              switch (datatype)
              {
              case sensor_msgs::PointField::INT8 :
                int8_t value1;
                std::memcpy(&value1, &cloud.data[input_begin], sizeof(int8_t));                
                eMatrix(j, k) = (float)value1;
                break;
              case sensor_msgs::PointField::INT16 :
                int16_t value3;
                std::memcpy(&value3, &cloud.data[input_begin], sizeof(int16_t));                   
                eMatrix(j, k) = (float)value3;
                break;
              case sensor_msgs::PointField::INT32 :
                int32_t value5;
                std::memcpy(&value5, &cloud.data[input_begin], sizeof(int32_t));     
                eMatrix(j, k) = (float)value5;                
                break;
              case sensor_msgs::PointField::UINT8 :
                uint8_t value2;                
                std::memcpy(&value2, &cloud.data[input_begin], sizeof(uint8_t));     
                eMatrix(j, k) = (float)value2;
                break;
              case sensor_msgs::PointField::UINT16 :
                uint16_t value4;
                std::memcpy(&value4, &cloud.data[input_begin], sizeof(uint16_t));     
                eMatrix(j, k) = (float)value4;               
                break;
              case sensor_msgs::PointField::UINT32 :
                uint32_t value6;
                std::memcpy(&value6, &cloud.data[input_begin], sizeof(uint32_t));     
                eMatrix(j, k) = (float)value6;
                break;
              case sensor_msgs::PointField::FLOAT32 :
                _Float32 value7;
                std::memcpy(&value7, &cloud.data[input_begin], sizeof(_Float32));     
                eMatrix(j, k) = (float)value7;
                break;
              case sensor_msgs::PointField::FLOAT64 :
                _Float64 value8;
                std::memcpy(&value8, &cloud.data[input_begin], sizeof(_Float64));     
                eMatrix(j, k) = (float)value8;
                break;                
              }
            }
          }
          parser.var(name).setLocal(eMatrix);
          EigenLab::Value<Eigen::MatrixXf> temp = parser.var(name);
        }
      }

      void Eigen2PointCloud(const std::string &field, const Eigen::MatrixXf &eigen, sensor_msgs::msg::PointCloud2 &cloud)
      {
        // Sort input cloud fields by field offset
        std::vector<sensor_msgs::PointField> sortedFields(cloud.fields);
        std::sort(sortedFields.begin(), sortedFields.end(), compareFieldsOffset);

        // Copy data into cloud
        for (int i=0; i < sortedFields.size(); i++)
        {
          //Setup input field values
          int start_in = sortedFields[i].offset;

          if (sortedFields[i].name == field) 
          {
            //Copy data
            for(uint32_t j = 0; j < cloud.height; j++)
            {
              for(uint32_t k = 0; k < cloud.width; k++)
              {
                uint32_t output_begin = j * cloud.row_step + k * cloud.point_step + start_in;
                //copy
                float value = eigen(j, k);
                std::memcpy(&cloud.data[output_begin], &value, sizeof(float));
               }
            }
          }
        }
      }

      static void appendPointCloud2Fields(const sensor_msgs::msg::PointCloud2 &cloud_in, sensor_msgs::msg::PointCloud2 &cloud_out, sensor_msgs::PointField &newField)
      {
        int in_count = cloud_in.fields.size();
        int out_count = in_count + 1;

        //Setup new cloud
        //ROS_DEBUG("input: height = %d, width = %d, step = %d", cloud_in.height, cloud_in.width, cloud_in.point_step);
        cloud_out.fields.clear();
        cloud_out.header = cloud_in.header;
        cloud_out.height = cloud_in.height;
        cloud_out.width  = cloud_in.width;
        cloud_out.fields.reserve(out_count);
        cloud_out.is_bigendian = cloud_in.is_bigendian;
        cloud_out.is_dense = cloud_in.is_dense;
        int offset = 0;

        //Copy existing fields
        for (int i = 0; i < in_count; ++i) 
        {
          std::string field_name = cloud_in.fields[i].name;
          int count = cloud_in.fields[i].count;
          int datatype = cloud_in.fields[i].datatype;
          offset = cloud_in.fields[i].offset; // use input cloud offsets to maintain existing padding
          offset = appendPointField(cloud_out, field_name, count, datatype, offset);
        }
        offset = cloud_in.point_step; // use input cloud offsets to maintain existing padding

        //Add new field
        offset = appendPointField(cloud_out, newField.name, newField.count, newField.datatype, offset);

        // Resize the point cloud accordingly
        cloud_out.point_step = offset;
        cloud_out.row_step = cloud_out.width * cloud_out.point_step;
        cloud_out.data.resize(cloud_out.height * cloud_out.row_step);
        //ROS_DEBUG("output: height = %d, width = %d, step = %d", cloud_out.height, cloud_out.width, cloud_out.point_step);

        //Copy existing data and initialize new data
        for(uint32_t i=0; i < cloud_in.height; i++)
        {
          for(uint32_t j=0; j < cloud_in.width; j++)
          {
            uint32_t input_step = cloud_in.point_step;
            uint32_t output_step = cloud_out.point_step;
            uint32_t count = std::min(cloud_in.point_step, cloud_out.point_step);
            uint32_t input_begin = i * cloud_in.row_step + j * cloud_in.point_step;
            uint32_t output_begin = i * cloud_out.row_step + j * cloud_out.point_step;
            //copy old fields
            for(uint32_t k=0; k < count; k++)
            {
              cloud_out.data[output_begin+k] = cloud_in.data[input_begin+k];
            }
            //Initialize new fields
            for(uint32_t k=count; k < output_step; k++)
            {
              cloud_out.data[output_begin+k] = 0;
            }
          }
        }
      }

      /** Private function that adds a PointField to the end of "fields" member of a PointCloud2.
       * @param cloud the PointCloud2 to add a field to
       * @param name the name of the field
       * @param count the number of elements in the PointField
       * @param datatype the datatype of the elements
       * @param offset the offset of that element
       * @return the offset of the next PointField that will be added to the PointCLoud2
       */
      static int appendPointField(sensor_msgs::msg::PointCloud2 &cloud, const std::string &name, const int &count, const int &datatype, const int &offset)
      {
        sensor_msgs::PointField point_field;
        point_field.name = name;
        point_field.count = count;
        point_field.datatype = datatype;
        point_field.offset = offset;

        bool compare = std::any_of(cloud.fields.begin(), cloud.fields.end(), hasField(point_field));
        if (compare) 
        {
          ROS_ERROR_STREAM("Field " << point_field.name << " already exists!");
          return offset;
        }
        else 
        {
          cloud.fields.push_back(point_field);

          // Update the offset
          return offset + point_field.count * sizeOfPointField(datatype);
        }
    };
  };

}

PLUGINLIB_EXPORT_CLASS(pointcloud2_filters::MathExpression, filters::FilterBase<sensor_msgs::msg::PointCloud2>)
