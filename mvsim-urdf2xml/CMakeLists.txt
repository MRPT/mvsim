# mvsim-urdf2xml: standalone Python tool (URDF -> MVSim vehicle XML)
install(
  PROGRAMS mvsim-urdf2xml.py
  DESTINATION ${CMAKE_INSTALL_BINDIR}
  RENAME mvsim-urdf2xml
)
if (DETECTED_ROS2)
  # Also runnable with "ros2 run mvsim mvsim-urdf2xml":
  install(
    PROGRAMS mvsim-urdf2xml.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME mvsim-urdf2xml
  )
endif()
