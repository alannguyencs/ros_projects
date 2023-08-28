# Install script for directory: /home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/cpr_gazebo/cpr_agriculture_gazebo

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/build/cpr_gazebo/cpr_agriculture_gazebo/catkin_generated/installspace/cpr_agriculture_gazebo.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cpr_agriculture_gazebo/cmake" TYPE FILE FILES
    "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/build/cpr_gazebo/cpr_agriculture_gazebo/catkin_generated/installspace/cpr_agriculture_gazeboConfig.cmake"
    "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/build/cpr_gazebo/cpr_agriculture_gazebo/catkin_generated/installspace/cpr_agriculture_gazeboConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cpr_agriculture_gazebo" TYPE FILE FILES "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/cpr_gazebo/cpr_agriculture_gazebo/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cpr_agriculture_gazebo" TYPE DIRECTORY FILES
    "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/cpr_gazebo/cpr_agriculture_gazebo/images"
    "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/cpr_gazebo/cpr_agriculture_gazebo/launch"
    "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/cpr_gazebo/cpr_agriculture_gazebo/meshes"
    "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/cpr_gazebo/cpr_agriculture_gazebo/rviz"
    "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/cpr_gazebo/cpr_agriculture_gazebo/scripts"
    "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/cpr_gazebo/cpr_agriculture_gazebo/urdf"
    "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/cpr_gazebo/cpr_agriculture_gazebo/worlds"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cpr_agriculture_gazebo" TYPE FILE FILES "/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/cpr_gazebo/cpr_agriculture_gazebo/model.config")
endif()

