cmake_minimum_required(VERSION 3.8)
project(filters_pointcloud2_erdc)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(filters CONFIG REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)

# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenMP)

# include_directories(include ${Boost_INCLUDE_DIRS})

# ## System dependencies are found with CMake's conventions
# find_package(PkgConfig REQUIRED)

# pkg_check_modules(TBB "tbb")
# if (TBB_FOUND)
#   add_definitions(
#     ${TBB_CFLAGS}
#     ${TBB_CFLAGS_OTHER}
#   )
# else()
#   message([FATAL_ERROR] "tbb module not found")
# endif ()


# build
add_library(${PROJECT_NAME} SHARED
  src/filters/AddNormals.cpp
  src/filters/BoxFilter.cpp
  src/filters/DeskewImu.cpp
  src/filters/DeskewLinear.cpp
  src/filters/DeskewTF.cpp
  src/filters/FieldValueFilter_Integer.cpp   
  src/filters/FlipRings.cpp
  # src/filters/MathExpression.cpp
  src/filters/PassThroughFilter.cpp
  src/filters/PassThroughFilterArray.cpp
  src/filters/RadiusFilter.cpp
  src/filters/RemoveBlackPoints.cpp
  src/filters/StatisticalOutlierFilter.cpp
  src/filters/Transform.cpp  
  src/filters/VoxelGridFilter.cpp
)
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)  
ament_target_dependencies(${PROJECT_NAME} 
  pluginlib 
  filters 
  pcl_ros 
  pcl_conversions
  tf2_ros
  tf2_sensor_msgs  
  tf2_geometry_msgs
  cv_bridge)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenMP_LIBS})


##############################################################################
# Install
##############################################################################

pluginlib_export_plugin_description_file(filters filters_pointcloud2_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}  
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

  ### Install Launch, Config, Params, etc. ###
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_package()