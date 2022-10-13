cmake_minimum_required(VERSION 3.0.2)
project(cvros0cpp)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
)

find_package(OpenCV REQUIRED)
catkin_package() # <--- this is required!!!!!

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(view_C         src/view_C.cpp)
target_link_libraries(view_C ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(view_OO        src/view_OO.cpp)
target_link_libraries(view_OO ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(cntWhitePixels src/cntWhitePixels.cpp)
target_link_libraries(cntWhitePixels ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(draw1Circle    src/draw1Circle.cpp)
target_link_libraries(draw1Circle ${catkin_LIBRARIES} ${OpenCV_LIBS})
