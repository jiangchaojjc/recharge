cmake_minimum_required(VERSION 3.0.2)
project(charging_station)

# 设置默认构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-g3 -O0 -Wall -Wextra)
  add_definitions(-DDEBUG_MODE=1)
  message(STATUS "Debug build configuration activated")
endif()


# 启用 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置OpenCV路径
set(OpenCV_DIR "/usr/local/share/OpenCV")

# 查找OpenCV包
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
  robot_sensors
)

add_service_files(
  FILES
  GoToChargingStation.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## 声明C++库
add_library(dec_circle
  include/charging_station/dec_circle.h
  src/dec_circle.cpp
)

add_dependencies(dec_circle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(dec_circle
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(charging_station_service_server src/charging_station_service_server.cpp)
add_dependencies(charging_station_service_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(charging_station_service_server 
  dec_circle  
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(charging_station_service_client src/charging_station_service_client.cpp)
add_dependencies(charging_station_service_client  
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(charging_station_service_client 
  dec_circle 
  ${catkin_LIBRARIES})

# add_executable(charging_station_publisher src/charging_station_publisher.cpp)
# target_link_libraries(charging_station_publisher ${catkin_LIBRARIES})

# add_executable(dec_rect_test src/dec_rect_test.cpp)
# target_link_libraries(dec_rect_test
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

# add_executable(dec_rect_py2c src/dec_rect_py2c.cpp)
# target_link_libraries(dec_rect_py2c
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

# add_executable(image_publisher src/image_publisher.cpp)
# target_link_libraries(image_publisher 
#   ${catkin_LIBRARIES} 
#   ${OpenCV_LIBRARIES}
# )