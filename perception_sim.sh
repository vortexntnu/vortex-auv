#!/bin/bash

rostopic pub /fsm/object_positions_in vortex_msgs/ObjectPosition "objectID: 'gate'
objectPose:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  pose:
    position: {x: 2.0, y: 2.0, z: -2.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} 
isDetected: true
estimateConverged: true
estimateFucked: false" &


sleep 10

rostopic pub /fsm/object_positions_in vortex_msgs/ObjectPosition "objectID: 'buoy'
objectPose:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  pose:
    position: {x: 5.0, y: 2.0, z: -2.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} 
isDetected: true
estimateConverged: true
estimateFucked: false" &

sleep 10

rostopic pub /fsm/object_positions_in vortex_msgs/ObjectPosition "objectID: 'torpedo'
objectPose:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  pose:
    position: {x: 7.0, y: 1.0, z: -2.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} 
isDetected: true
estimateConverged: true
estimateFucked: false" &


sleep 10

rostopic pub /fsm/object_positions_in vortex_msgs/ObjectPosition "objectID: 'octagon'
objectPose:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  pose:
    position: {x: 10.0, y: 0.0, z: -2.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} 
isDetected: true
estimateConverged: true
estimateFucked: false" &