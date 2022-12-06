cmake_minimum_required(VERSION 3.0.2)
project(dip_proj_pkg)

find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)

set(CMAKE_CXX_STANDARD 17)

add_message_files(
  FILES
  pid_driver.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
CATKIN_DEPENDS
roscpp
rospy
std_msgs
message_runtime
)


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    include
)

add_executable(${PROJECT_NAME}_node 
    src/main.cpp
    src/color_split.cpp
    src/line_detect.cpp
    src/dip_process.cpp
    src/pill_detect.cpp
    src/pid_calc.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
