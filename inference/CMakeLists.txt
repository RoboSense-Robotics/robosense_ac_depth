# libdepth_inference/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
project(lib_depth_inference LANGUAGES CXX VERSION 1.0.0)
option(ENABLE_TENSORRT "Enable TensorRT inference backend" ON)
set(CMAKE_BUILD_TYPE Release)
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )
if(CMAKE_INSTALL_PREFIX STREQUAL "/usr/local")  
  set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../ros_wrapper/lib_depth_inference")
  message(STATUS "Using default install path: ${CMAKE_INSTALL_PREFIX}")
else()
  message(STATUS "Setting install path: ${CMAKE_INSTALL_PREFIX}")  
endif()

add_definitions(
  -DLIBDEPTH_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
  -DLIBDEPTH_VERSION_MINOR=${PROJECT_VERSION_MINOR}
  -DLIBDEPTH_VERSION_PATCH=${PROJECT_VERSION_PATCH}
  -DLIBDEPTH_VERSION="${PROJECT_VERSION}"
)

# add_definitions(-DENABLE_DEBUG_OUTPUT)

include_directories(${GLOG_INCLUDE_DIRS})

if(ENABLE_TENSORRT)
  add_definitions(-DENABLE_TENSORRT)
  find_package(CUDA REQUIRED)
  set(TENSORRT_LIBS nvinfer nvonnxparser nvinfer_plugin)
else()
  set(TENSORRT_LIBS "")
endif()


find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)


file(GLOB_RECURSE SRCS src/*/*.cpp)

file(GLOB_RECURSE YAML_SRCS 3rdparty/yaml-cpp/src/*)

add_library(depth_inference SHARED ${SRCS} ${YAML_SRCS} src/depth_inference_impl.cpp)


# set version properties for the shared library
set_target_properties(depth_inference PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
)

if(ENABLE_TENSORRT)
  target_link_libraries(depth_inference PUBLIC ${CUDA_LIBRARIES})
endif()

target_link_libraries(depth_inference PUBLIC 
  ${OpenCV_LIBS} 
  ${PCL_LIBRARIES}
  ${TENSORRT_LIBS}
)
target_include_directories(depth_inference PUBLIC 
  include
  3rdparty/yaml-cpp/include  
  ${OpenCV_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}

)

if(ENABLE_TENSORRT)
  target_include_directories(depth_inference PUBLIC ${CUDA_INCLUDE_DIRS})
endif()

# install library
install(TARGETS depth_inference
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# install api depth_inference_api.h
install(FILES include/depth_inference_api.h
    DESTINATION include
)

# install config
install(DIRECTORY config/
    DESTINATION config
)