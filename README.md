
To run example code

```
roslaunch pointcloud2_filters_erdc example.launch type:=BoxFilter
```

Replace BoxFilter with any of the yaml filenames in examples folder.


Creating new filters requires code edits in three places.

1) Create new class file - place new .cpp file in src/filters folder.

2) Create dynamic parameters file (optional) - place new .cfg file in cfg folder.

2) Edit `CMakeList.txt` - Add the new file to list of files under `add_library`.  Also (optional), add dynamic parameter file to `generate_dynamic_reconfigure_options`.

3) Edit `pointcloud2_filters_plugins.xml` - Create library entry that maps class name to the plugin name.

