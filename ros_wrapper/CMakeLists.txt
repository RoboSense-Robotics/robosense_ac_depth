cmake_minimum_required(VERSION 3.8)
project(robosense_ac_depth)

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Detect ROS version
if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  # Try to detect based on available packages
  find_package(catkin QUIET)
  if(catkin_FOUND)
    set(ROS_VERSION 1)
  else()
    find_package(ament_cmake QUIET)
    if(ament_cmake_FOUND)
      set(ROS_VERSION 2)
    endif()
  endif()
endif()

message(STATUS "Building for ROS${ROS_VERSION}")

# ROS1 (catkin) configuration
if(ROS_VERSION EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    cv_bridge
    image_transport
    message_filters
    std_msgs  
    robosense_msgs 
  )
elseif(ROS_VERSION EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(robosense_msgs REQUIRED)
endif()

# Find OpenCV
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time)
find_package(PCL REQUIRED)

find_library(DEPTH_INFERENCE_LIBRARY
  NAMES depth_inference
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib_depth_inference/lib
  REQUIRED
)
get_filename_component(DEPTH_INFERENCE_LIBRARY_DIR ${DEPTH_INFERENCE_LIBRARY} DIRECTORY)

# Include directories
if(ROS_VERSION EQUAL 1)
  include_directories(${catkin_INCLUDE_DIRS})
  # Catkin package configuration
  catkin_package(    
    CATKIN_DEPENDS roscpp message_filters cv_bridge image_transport std_msgs sensor_msgs
    INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/lib_depth_inference/include
  )
endif()
add_executable(ros_node src/ros_node.cpp)

if(ROS_VERSION EQUAL 1)
  target_link_libraries(ros_node
    ${catkin_LIBRARIES}
  )
elseif(ROS_VERSION EQUAL 2)
  ament_target_dependencies(ros_node
    rclcpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
    message_filters
    pcl_conversions
    robosense_msgs
  )
endif()
target_compile_definitions(ros_node PRIVATE ROS_VERSION=${ROS_VERSION})
target_include_directories(ros_node PUBLIC
  # ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR}/lib_depth_inference/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
) 
target_link_libraries(ros_node
    # ${OpenCV_LIBRARIES}
    ${DEPTH_INFERENCE_LIBRARY}
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}
    Boost::date_time
  )
set_target_properties(ros_node PROPERTIES
  BUILD_RPATH "${DEPTH_INFERENCE_LIBRARY_DIR}"
  INSTALL_RPATH "$ORIGIN"
)

# Install library
if(ROS_VERSION EQUAL 1)
  install(TARGETS ros_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  if(EXISTS "${DEPTH_INFERENCE_LIBRARY_DIR}")
    install(DIRECTORY ${DEPTH_INFERENCE_LIBRARY_DIR}/
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
      FILES_MATCHING PATTERN "libdepth_inference.so*"
    )
  endif()
  # Install header files
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
  )
  
  # Install launch files
  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
  )
elseif(ROS_VERSION EQUAL 2)
  install(TARGETS ros_node
    DESTINATION lib/${PROJECT_NAME}
  )
  if(EXISTS "${DEPTH_INFERENCE_LIBRARY_DIR}")
    install(DIRECTORY ${DEPTH_INFERENCE_LIBRARY_DIR}/
      DESTINATION lib/${PROJECT_NAME}
      FILES_MATCHING PATTERN "libdepth_inference.so*"
    )
  endif()
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    FILES_MATCHING PATTERN "*.launch" PATTERN "*.launch.py"
  )
  ament_package()
endif()
