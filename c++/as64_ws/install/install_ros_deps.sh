#!/bin/bash

##########################################################
#              Install ROS Dependencies                  #
##########################################################
cd $INSTALL_SCRIPTS_DIR
cd ..
MESSAGE="Installing the ROS dependencies of as64_ws packages (see the package.xml files)"; blue_echo && \
rosdep install --from-paths ./ --ignore-src --rosdistro indigo -y
if [ $? -ne 0 ]; then
  MESSAGE="Failed to install as64_ws Packages."; red_echo
  exit 1
fi
