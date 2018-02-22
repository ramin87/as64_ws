#!/bin/bash

cd $INSTALL_SCRIPTS_DIR/deps

MESSAGE="Installing Eigen Library..."; blue_echo && \

MESSAGE="Downloading the latest Eigen (v3.3.4)"; blue_echo && \

if [ -d eigen3 ]; then
  rm -rf eigen3
fi

wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz > /dev/null &&\
tar xvf 3.3.4.tar.gz > /dev/null && \
rm -rf 3.3.4.tar.gz && \
mv eigen-eigen-5a0156e40feb eigen3
cd eigen3
mkdir build
cd build
cmake ..
sudo make install

if [ $? -eq 0 ]; then
  MESSAGE="Eigen Successfully installed."; green_echo
  AS64_ERROR=0
else
  MESSAGE="Failed to install Eigen"; red_echo
  AS64_ERROR=1
fi
