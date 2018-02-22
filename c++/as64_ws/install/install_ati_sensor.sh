#!/bin/bash

MESSAGE="Installing ATI Sensor ROS Package..."; blue_echo && \
cd $AS64_WS_DIR/ati
if [ -d "ati_sensor" ]
then
  echo "Ati Sensor package already exists. Fetching changes."
  cd ati_sensor
  git pull origin master
  cd ..
else
  echo "Adding Ati Sensor Package"
  git clone https://github.com/kuka-isir/ati_sensor
fi
cd ..

if [ $? -eq 0 ]; then
  MESSAGE="ATI Sensor Successfully installed."; green_echo
  AS64_ERROR=0
else
  MESSAGE="Failed to install ATI Sensor"; red_echo
  AS64_ERROR=1
fi
