#!/bin/bash

cd $INSTALL_SCRIPTS_DIR
AS64_ERROR=0
source color_echo.sh

if [ $# -eq 0 ]; then
  declare -a arr=("latest_armadillo" "latest_eigen3" "frilibrary")
elif [ "$1" = "-a" ]; then
  declare -a arr=("latest_armadillo" "latest_eigen3" "frilibrary" "ati_sensor" "barretthand" "mujoco" "ros_deps")
else
  MESSAGE="The provided option does not exit."; red_echo
  exit 1
fi

MESSAGE="Installing as64_ws"; blue_echo

MESSAGE="Installing main Dependencies: cmake, wget, xz-utils..."; blue_echo
sudo apt-get update > /dev/null
sudo apt-get install -y build-essential cmake wget xz-utils unzip > /dev/null

mkdir -p deps

## now loop through the above array
for i in "${arr[@]}"
do
  cd $INSTALL_SCRIPTS_DIR
  source install_$i.sh
  if [ $AS64_ERROR -ne 0 ]; then
    MESSAGE="Failed to install as64_ws Packages."; red_echo
    exit 1
  fi
done

cd $INSTALL_SCRIPTS_DIR
rm -rf deps/
cd ..

MESSAGE="as64_ws Packages Successfully installed."; green_echo
