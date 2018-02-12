#!/bin/bash

catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash

roslaunch dmp_kuka_test DMP_Kuka_test.launch 
