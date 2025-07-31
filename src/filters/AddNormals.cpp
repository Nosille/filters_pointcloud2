
#include <rclcpp/rclcpp.hpp>
#include <filters/filter_base.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>


#if defined __has_include
#  if __has_include (<pcl/gpu/features/features.hpp>)
#    define _PCL_GPU_FEATURES_
#    include <pcl/gpu/features/features.hpp>
#  endif
#endif

#ifdef _OPENMP
#include <pcl/features/normal_3d_omp.h>
#endif


namespace pointcloud2_filters
{
  class AddNormals : public filters::FilterBase<sensor_msgs::msg::PointCloud2>
  {
    protected:

      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pubIntermediate;

      int m_ompThreads;
      int m_numNeighbors;
      double m_searchRadius;
      
    public:
      AddNormals() { }

      bool configure()
      {
        RCLCPP_INFO(this->logging_interface_->get_logger(),"Pointcloud2AddNormals configuring");

        getParam("omp_threads", m_ompThreads, 1);
        getParam("num_neighbors", m_numNeighbors, 10);
        getParam("search_radius", m_searchRadius, 1.0);

        RCLCPP_INFO(this->logging_interface_->get_logger(),"  omp_threads: %d", m_ompThreads);       
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  num_neighbors: %d", m_numNeighbors);
        RCLCPP_INFO(this->logging_interface_->get_logger(),"  search_radius: %f", m_searchRadius);

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

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

        pcl::fromPCLPointCloud2(*cloud, *cloud_xyz);

        #ifdef _PCL_GPU_FEATURES_
          #warning "Using  _PCL_GPU_FEATURES_ for normal estimation"

          // ROS_WARN_STREAM("PCL gpu features found");

          pcl::gpu::NormalEstimation::PointCloud cloud_gpu;
          pcl::gpu::NormalEstimation::Normals normals_gpu;
          
          // copy cloud to gpu memory
          cloud_gpu.upload(cloud_xyz->points);

          // calculate normals
          pcl::gpu::NormalEstimation ne;
          ne.setInputCloud(cloud_gpu);
          ne.setRadiusSearch(m_searchRadius, m_numNeighbors);
          ne.setViewPoint(0.0, 0.0, 0.0);
          ne.compute(normals_gpu);

          // download normals to cpu
          std::vector<pcl::PointXYZ> download;
          normals_gpu.download(download);

          for (int i = 0; i < download.size(); i++)
          {
              pcl::PointXYZ entry = download[i];
              normals->push_back(pcl::Normal(entry.data[0], entry.data[1], entry.data[2], entry.data[3]));
          }

        #elif _OPENMP
          #warning "Using _OPENMP for normal estimation"

          // ROS_WARN_STREAM("OPENMP Found, Using " << omp_threads_ << " threads.");

          pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne(m_ompThreads);

          ne.setInputCloud(cloud_xyz);

          // Create an empty kdtree representation, and pass it to the normal estimation object.
          // Its content will be filled inside the object, based on the given input dataset (as no other search surface is
          // given).
          pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
          ne.setSearchMethod(tree);
          // Use all neighbors in a sphere
          if (m_numNeighbors > 0)
          {
            ne.setKSearch(m_numNeighbors);
          } 
          else
          {
            ne.setRadiusSearch(m_searchRadius);
          }
          ne.setViewPoint(0.0, 0.0, 0.0);
            
          // Compute the features
          ne.compute(*normals);
        #else
          #warning "Using neither _PCL_GPU_FEATURES_ or _OPENMP for normal estimation"

          // ROS_WARN_STREAM("OPENMP Not Found, Using single thread");

          pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            
          ne.setInputCloud(cloud_xyz);

          // Create an empty kdtree representation, and pass it to the normal estimation object.
          // Its content will be filled inside the object, based on the given input dataset (as no other search surface is
          // given).
          pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
          ne.setSearchMethod(tree);

          // Use all neighbors in a sphere
          if (m_numNeighbors > 0)
          {
            ne.setKSearch(m_numNeighbors);
          } 
          else
          {
            ne.setRadiusSearch(m_searchRadius);
          }
          ne.setViewPoint(0.0, 0.0, 0.0);          
            
          // Compute the features
          ne.compute(*normals);
        #endif

        // A bunch of conversion nonsense
        pcl::PCLPointCloud2::Ptr point_normals(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*normals, *point_normals);
        
        pcl::PCLPointCloud2 cloud_normals;
        point_normals->width = cloud->width;
        point_normals->height = cloud->height;
        point_normals->row_step = point_normals->width * point_normals->point_step;
        pcl::concatenateFields(*cloud, *point_normals, cloud_normals);

        // Convert to msg
        pcl_conversions::fromPCL(cloud_normals, output_msg);
        output_msg.header = input_msg.header;

        // publish here for debug purposes
        if(m_pubIntermediate!= nullptr && m_pubIntermediate->get_subscription_count() > 0) 
          m_pubIntermediate->publish(output_msg);

        return true;
      }

  };

}

PLUGINLIB_EXPORT_CLASS(pointcloud2_filters::AddNormals, filters::FilterBase<sensor_msgs::msg::PointCloud2>)
