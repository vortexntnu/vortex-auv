#!/bin/bash

drone="beluga"
type="simulator"

while getopts ":d:t" opt; do
  case $opt in
    d)
      drone=$OPTARG
      ;;
    t)
      type=$OPTARG
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
  esac
done

source /vortex_ws/devel/setup.bash
roslaunch auv_setup $drone.launch type:=$type