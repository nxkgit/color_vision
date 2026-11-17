cmake_minimum_required(VERSION 3.0.2)
project(color_vision)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

# Scripts run from src/ (chmod +x src/*.py); no catkin_install_python, whose devel
# relay scripts break the local `import color_geometry`.
