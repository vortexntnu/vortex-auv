#!/bin/bash

echo "yaw";
timeout 2s rostopic pub /thrust/desired_forces geometry_msgs/Wrench "force:
  x: 0.0  
  y: 0.0  
  z: 0.0  
torque:   
  x: 0.0  
  y: 0.0     
  z: 0.01";

echo "##First##"
timeout 2s rostopic pub /thrust/desired_forces geometry_msgs/Wrench "force:
  x: 0.0  
  y: 0.0  
  z: 0.0  
torque:   
  x: 0.0  
  y: 0.0 
  z: 0.0";

  echo "done" ;
  echo "##First##"  