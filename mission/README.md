## Mission
Any packages that determine how the AUV behaves during a mission is put here. This will mainly be
a state machine and/or behavior tree for decision making, but also collision detection and other mission-related
plans.

The current system contains:

* finite_state_machine
    * SMACH-based fsm that determines how tasks are to be executed
    * Closely linked to the guidance interface

* anomaly_detection
    * A currently unused battery simulator to keep track of actual battery status