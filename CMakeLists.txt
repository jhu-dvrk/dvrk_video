cmake_minimum_required (VERSION 3.16)
project (dvrk_video VERSION 2.4.0)

find_package (
  catkin QUIET
  )

if (catkin_FOUND)
  catkin_package ()

else (catkin_FOUND)

  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)

    install (DIRECTORY
      ros2/launch
      DESTINATION share/${PROJECT_NAME})

    file (MAKE_DIRECTORY calibrations)
    install (DIRECTORY
      calibrations
      DESTINATION share/${PROJECT_NAME})

    ament_package ()
  endif (ament_cmake_FOUND)

endif (catkin_FOUND)
