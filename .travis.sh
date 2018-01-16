#!/bin/bash

set -x
set -e

apt-get update
apt-get install -y sudo software-properties-common wget # minimum deps for the commands below

echo "Testing branch $TRAVIS_BRANCH of $REPOSITORY_NAME"
sudo sh -c 'echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo add-apt-repository $MRPT_PPA -y
sudo apt-get update -qq
sudo apt-get install -qq -y python-rosdep
sudo rosdep init
rosdep update; while [ $? != 0 ]; do sleep 1; rosdep update; done

# Use rosdep to install all dependencies (including ROS itself)
rosdep install --from-paths ./ -i -y --rosdistro $CI_ROS_DISTRO

# ROS env vars:
source /opt/ros/$CI_ROS_DISTRO/setup.bash

# Setup catkin Workspace:
mkdir -p ~/catkin_ws/src
ln -s $CI_SOURCE_PATH ~/catkin_ws/src # Link the repo we are testing to the new workspace
cd ~/catkin_ws/

# Build [and Install] packages, build tests, and run tests
catkin_make_isolated --install --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin_make_isolated --install --cmake-args -DCMAKE_BUILD_TYPE=Release --catkin-make-args tests
catkin_make_isolated --install --cmake-args -DCMAKE_BUILD_TYPE=Release --catkin-make-args run_tests
# Check results
catkin_test_results ./build_isolated
