execute_process(COMMAND "/home/kartik/Documents/gazebo_practice_ws/build/autonomous_vehicle/catvehicle_basics/catvehicle/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/kartik/Documents/gazebo_practice_ws/build/autonomous_vehicle/catvehicle_basics/catvehicle/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
