# Install script for directory: /home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/autonomous_vehicle_nosecurity

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/kartik/Documents/gazebo_practice_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/kartik/Documents/gazebo_practice_ws/build/autonomous_vehicle/autonomous_vehicle_nosecurity/catkin_generated/installspace/autonomous_vehicle_nosecurity.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/autonomous_vehicle_nosecurity/cmake" TYPE FILE FILES
    "/home/kartik/Documents/gazebo_practice_ws/build/autonomous_vehicle/autonomous_vehicle_nosecurity/catkin_generated/installspace/autonomous_vehicle_nosecurityConfig.cmake"
    "/home/kartik/Documents/gazebo_practice_ws/build/autonomous_vehicle/autonomous_vehicle_nosecurity/catkin_generated/installspace/autonomous_vehicle_nosecurityConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/autonomous_vehicle_nosecurity" TYPE FILE FILES "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/autonomous_vehicle_nosecurity/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/autonomous_vehicle_nosecurity" TYPE DIRECTORY FILES "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/autonomous_vehicle_nosecurity/launch")
endif()
