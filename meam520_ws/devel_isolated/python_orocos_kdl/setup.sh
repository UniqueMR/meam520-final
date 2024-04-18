#!/usr/bin/env sh
# generated from catkin.builder Python module

# remember type of shell if not already set
if [ -z "$CATKIN_SHELL" ]; then
  CATKIN_SHELL=sh
fi
. "/home/student/meam520_ws/devel_isolated/panda_simulator_examples/setup.$CATKIN_SHELL"

# detect if running on Darwin platform
_UNAME=`uname -s`
IS_DARWIN=0
if [ "$_UNAME" = "Darwin" ]; then
  IS_DARWIN=1
fi

# Prepend to the environment
export CMAKE_PREFIX_PATH="/home/student/meam520_ws/devel_isolated/python_orocos_kdl:$CMAKE_PREFIX_PATH"
if [ $IS_DARWIN -eq 0 ]; then
  export LD_LIBRARY_PATH="/home/student/meam520_ws/devel_isolated/python_orocos_kdl/lib:/home/student/meam520_ws/devel_isolated/python_orocos_kdl/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"
else
  export DYLD_LIBRARY_PATH="/home/student/meam520_ws/devel_isolated/python_orocos_kdl/lib:/home/student/meam520_ws/devel_isolated/python_orocos_kdl/lib/aarch64-linux-gnu:$DYLD_LIBRARY_PATH"
fi
export PATH="/home/student/meam520_ws/devel_isolated/python_orocos_kdl/bin:$PATH"
export PKG_CONFIG_PATH="/home/student/meam520_ws/devel_isolated/python_orocos_kdl/lib/pkgconfig:/home/student/meam520_ws/devel_isolated/python_orocos_kdl/lib/aarch64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH"
export PYTHONPATH="/home/student/meam520_ws/devel_isolated/python_orocos_kdl/lib/python3/dist-packages:$PYTHONPATH"
