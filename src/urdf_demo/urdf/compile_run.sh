#!/bin/bash 

rosrun xacro xacro crab_model.xacro > crab_model.urdf;
roslaunch urdf_tutorial display.launch model:=`rospack find urdf_demo`/urdf/crab_model.urdf gui:=true;
# rosrun gazebo_ros spawn_model -file `rospack find urdf_demo`/urdf/crab_model.urdf -urdf -x 0 -y 0 -z 1 -model crab 
