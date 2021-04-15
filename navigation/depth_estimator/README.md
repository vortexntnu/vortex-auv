# Depth Estimator

Node that converts measured water pressure to depth estimates.  

## Input and Output

Subscibers
* __/dvl/pressure__ pressure in pascal

Publishers
* __/depth/estimated__ Estimated depth in meters

## Parameters

* __/atmosphere/pressure__ Pressure of air at sea level
* __/water/density__ density of water
* __/gravity/acceleration__ gravity acceleration
