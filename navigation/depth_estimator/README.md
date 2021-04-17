# Depth Estimator

Node that converts measured water pressure to depth estimates in ENU frame.  

## Input and Output

Subscibers
* __/dvl/pressure__ pressure in [kPa]

Publishers
* __/depth/estimated__ Estimated depth in ENU [m]

## Parameters

* __/atmosphere/pressure__ Pressure of air at sea level [kPa]
* __/water/density__ density of water [kg/m3]
* __/gravity/acceleration__ gravity acceleration [m/s2]
