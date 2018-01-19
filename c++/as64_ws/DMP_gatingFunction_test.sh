#!/bin/bash

catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roscore &

rosrun dmp_test DMP_gatingFunction_test lin 1.0 0.01
rosrun dmp_test DMP_gatingFunction_test constant 1.0 0.2
rosrun dmp_test DMP_gatingFunction_test exp 1.0 0.01
rosrun dmp_test DMP_gatingFunction_test sigmoid 1.0 0.5
rosrun dmp_test DMP_gatingFunction_test spring-damper 1.0 0.01
