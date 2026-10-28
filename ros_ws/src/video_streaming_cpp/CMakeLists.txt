cmake_minimum_required(VERSION 3.8)
project(video_streaming_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Lets VS Code IntelliSense find the real include paths (removes false red squiggles).
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_publisher_ffmpeg src/camera_publisher_ffmpeg.cpp)
ament_target_dependencies(camera_publisher_ffmpeg rclcpp image_transport cv_bridge sensor_msgs)
target_link_libraries(camera_publisher_ffmpeg ${OpenCV_LIBS})

add_executable(viewer_ffmpeg src/viewer_ffmpeg.cpp)
ament_target_dependencies(viewer_ffmpeg rclcpp image_transport cv_bridge sensor_msgs)
target_link_libraries(viewer_ffmpeg ${OpenCV_LIBS})

install(TARGETS
  camera_publisher_ffmpeg
  viewer_ffmpeg
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
