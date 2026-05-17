execute_process(COMMAND "/home/jetbot/main_ws/build/sensors/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/jetbot/main_ws/build/sensors/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
