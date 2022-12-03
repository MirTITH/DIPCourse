cmake_minimum_required(VERSION 3.0.0)
project(dip4_none_ros VERSION 0.1.0)

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ./
)

add_executable(${PROJECT_NAME} 
    main.cpp
    color_split.cpp
    line_detect.cpp
)

target_link_libraries(${PROJECT_NAME} 
    ${OpenCV_LIBS}
)
