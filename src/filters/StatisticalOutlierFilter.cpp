
#include <rclcpp/rclcpp.hpp>
#include <filters/filter_base.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>


namespace pointcloud2_filters
{
  class StatisticalOutlierFilter : public filters::FilterBase<sensor_msgs::msg::PointCloud2>
  {
    protected:

      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pubIntermediate;

      int m_numNeighbors;
      double m_stdDevMulThresh;
      
    public:
      StatisticalOutlierFilter() { }

      bool configure()
      {
        RCLCPP_INFO(this->logging_interface_->get_logger(),"Pointcloud2StatisticalOutlierFilter configuring");

        getParam("num_neighbors", m_numNeighbors, 10);
        getParam("std_dev_mul_thresh", m_stdDevMulThresh, 1.0);

        RCLCPP_INFO(this->logging_interface_->get_logger(),"  num_neighbors: %d", m_numNeighbors);
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  std_dev_mul_thresh: %f", m_stdDevMulThresh);

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

        // Convert to pcl
        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(input_msg, *cloud);

        // filter
        pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
        pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(m_numNeighbors);
        sor.setStddevMulThresh(m_stdDevMulThresh);
        sor.filter(*cloud_filtered);

        // Convert to msg
        pcl_conversions::fromPCL(*cloud_filtered, output_msg);

        // publish here for debug purposes
        if(m_pubIntermediate!= nullptr && m_pubIntermediate->get_subscription_count() > 0) 
          m_pubIntermediate->publish(output_msg);

        return true;
      }

  };

}

PLUGINLIB_EXPORT_CLASS(pointcloud2_filters::StatisticalOutlierFilter, filters::FilterBase<sensor_msgs::msg::PointCloud2>)
