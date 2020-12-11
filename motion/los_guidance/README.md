# The Line-Of-Sight guidance module

## IN-PROGRESS
Currently the guidance system is also importing a controller, which is suboptimal for modularity. The guidance block should only contain the necessary components for the LOS system, and publish results to a topic. This same topic can be subscribed to by a controller block.

For this reason, this guidance block should be moved to navigation, and the new controller module will be placed in motion. (The controller module is mostly already defined in "motion/autopilot" but should be renamed and clarified)

Once that is done, proper READMEs can be produced for each module.