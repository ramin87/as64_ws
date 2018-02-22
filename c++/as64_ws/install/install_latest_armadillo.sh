#!/bin/bash

cd $INSTALL_SCRIPTS_DIR/deps

MESSAGE="Installing Armadillo"; blue_echo && \

MESSAGE="Installing Armadillo Dependencies: cmake, OpenBLAS and LAPACK, wget, xz-utils..."; blue_echo && \
sudo apt-get update > /dev/null && \
sudo apt-get install -y cmake libopenblas-dev liblapack-dev wget xz-utils > /dev/null && \

MESSAGE="Downloading and building the latest Armadillo (v8.300)"; blue_echo && \

if [ -d armadillo-8.300.1 ]; then
  rm -rf armadillo-8.300.
fi

wget --no-check-certificate http://sourceforge.net/projects/arma/files/armadillo-8.300.1.tar.xz > /dev/null
tar xvf armadillo-8.300.1.tar.xz > /dev/null
rm -rf armadillo-8.300.1.tar.xz
cd armadillo-8.300.1
cmake .
make
sudo make install

if [ $? -eq 0 ]; then
  MESSAGE="Armadillo Successfully installed."; green_echo
  AS64_ERROR=0
else
  MESSAGE="Failed to install Armadillo."; red_echo
  AS64_ERROR=1
fi
