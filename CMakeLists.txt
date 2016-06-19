cmake_minimum_required(VERSION 2.8.3)
project(ardrone_station)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(Qt4 REQUIRED)
set( QT_USE_QTGUI TRUE )
set( QT_USE_QTOPENGL TRUE )
set( QT_USE_QTXML TRUE )
include(${QT_USE_FILE})
#ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

find_package(OpenCV)
#find_package(Phonon REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(qt_srcs
      src/main.cpp
      src/mainwindow.cpp
      src/painterWiget.cpp
      
   )
set(qt_hdrs
      src/mainwindow.h
      src/receiver.h
      src/painterWiget.h
      src/camera.h
      src/camera2.h
      src/camera_calibration.h
   )
set(qt_resrcs src/Icons.qrc)

qt4_automoc(${qt_srcs})
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs} ${catkin_LIBRARIES})
QT4_WRAP_UI(uis_h src/mainwindow.ui)
QT4_ADD_RESOURCES(qt_resrcs_RCC ${qt_resrcs})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
#uncomment if you have defined messages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

add_executable(receiver src/receiver.cpp src/camera.cpp src/camera2.cpp src/camera_calibration.cpp
    ${uis_h} ${qt_srcs} ${qt_moc_srcs} ${qt_resrcs_RCC})
target_link_libraries(receiver ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(receiver receiver_generate_messages_cpp)



