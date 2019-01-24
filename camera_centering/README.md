#Camera centering
Package to center Manta on an object in the camera feed.

##Overview

Subscribing to topic with CameraObjectInfo.msg from computer vision node

Initializing 2 PID controllers, one in x- and one in y-direction on the video frame.

Controlling heave and yaw by publishing on /propulsion_command with PropulsionCommand.msg according to objects offset from center of camera feed.
