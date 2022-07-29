cmake_minimum_required(VERSION 3.0.2)
project(serialPort)
find_package(OpenCV 4 REQUIRED)
find_package(yaml-cpp)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)
set(OpenCV_DIR  ~/opencv/build)
set(CMAKE_CXX_STANDARD 11)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(detect STATIC
        lib/ED.cpp
        lib/ED.h
        lib/EDCircles.cpp
        lib/EDCircles.h
        lib/EDColor.cpp
        lib/EDColor.h
        lib/EDLib.h
        lib/EDLines.cpp
        lib/EDLines.h
        lib/EDPF.cpp
        lib/EDPF.h
        lib/NFA.cpp
        lib/NFA.h)

list(APPEND thirdparty_libraries
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_executable(${PROJECT_NAME} src/serialPort.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES} yaml-cpp detect ${thirdparty_libraries}
 )

 add_executable(read src/read_key.cpp)
add_dependencies(read ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(read
   ${catkin_LIBRARIES} yaml-cpp detect  ${thirdparty_libraries}
 )
