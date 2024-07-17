#!/bin/bash 

rosrun xacro xacro crab_model.xacro > crab_model.urdf;
roslaunch urdf_tutorial display.launch model:=`rospack find urdf_demo`/urdf/crab_model.urdf gui:=true;

