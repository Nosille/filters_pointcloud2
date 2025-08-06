# filters_pointcloud2_erdc

To run example code

```bash
roslaunch filters_pointcloud2_erdc example.launch.xml type:=BoxFilter
```

Replace BoxFilter with any of the yaml filenames in params folder.


Creating new filters requires code edits in three places.

1) Create new class file - place new .cpp file in src/filters folder.  You can use one of the existing files as a template.  It is important that the new filter inherits from filters::FilterBase<sensor_msgs::msg::PointCloud2> and overrides two functions 1) configure() 2) update(const sensor_msgs::msg::PointCloud2& input_msg, sensor_msgs::msg::PointCloud2& output_msg).

2) Edit `CMakeList.txt` - Add the new file to list of files under `add_library`.

3) Edit `filters_pointcloud2_plugins.xml` - Create library entry that maps class name to the plugin name.

