cmake_minimum_required(VERSION 3.8)
project(pointcloud2_filters_erdc)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(filters)

find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenMP)

include_directories(include ${Boost_INCLUDE_DIRS})

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
  add_library(${PROJECT_NAME}
    src/filters/AddNormals.cpp
    src/filters/BoxFilter.cpp
    #src/filters/ConvertToImage.cpp
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
    rclcpp_lifecycle 
    pluginlib 
    filters 
    pcl_ros 
    pcl_conversions 
    cv_bridge)
  target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})


##############################################################################
# Install
##############################################################################

  install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}  
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION bin/${PROJECT_NAME}
  )

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

ament_package()