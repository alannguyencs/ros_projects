# Install script for directory: /home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/cpr_gazebo/cpr_obstacle_gazebo

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/build/cpr_gazebo/cpr_obstacle_gazebo/catkin_generated/installspace/cpr_obstacle_gazebo.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cpr_obstacle_gazebo/cmake" TYPE FILE FILES
    "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/build/cpr_gazebo/cpr_obstacle_gazebo/catkin_generated/installspace/cpr_obstacle_gazeboConfig.cmake"
    "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/build/cpr_gazebo/cpr_obstacle_gazebo/catkin_generated/installspace/cpr_obstacle_gazeboConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cpr_obstacle_gazebo" TYPE FILE FILES "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/cpr_gazebo/cpr_obstacle_gazebo/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cpr_obstacle_gazebo" TYPE DIRECTORY FILES
    "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/cpr_gazebo/cpr_obstacle_gazebo/launch"
    "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/cpr_gazebo/cpr_obstacle_gazebo/models"
    "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/cpr_gazebo/cpr_obstacle_gazebo/rviz"
    "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/cpr_gazebo/cpr_obstacle_gazebo/urdf"
    "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/cpr_gazebo/cpr_obstacle_gazebo/worlds"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cpr_obstacle_gazebo" TYPE FILE FILES "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/cpr_gazebo/cpr_obstacle_gazebo/model.config")
endif()

