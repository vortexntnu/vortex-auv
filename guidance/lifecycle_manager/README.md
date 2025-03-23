This package contains the lifecycle manager and lifecycle client nodes.
The purpose of these nodes is to manage the transitions for a list of lifecycle nodes.



## LifecycleNodeManager:

#### Specifications

Input: Should be a list of lifecyclenodes to be controlled (perhaps grouped together in which paris should work together)

Be able to control each lifecycle node by itself
Be able to control pre-determined groups of lifecycle nodes i.e DP + DP_reference, LQR + LOS(T)_guidance

Should be able to set the state of all lifecycle nodes to for example shutdown or active, through the correct set of transitions.
