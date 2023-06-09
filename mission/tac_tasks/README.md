# Pipeline

## Futher improvements
- Currently the the z value of the goal position is set to the same as the position of the drone. This should be improved. Maybe it shouldnt be updated at each iteration?
- The integration with the task manager is not tested. The idea is that the task manager also needs to be able to shut down while loops within the different states as well as the main node. Therefore it is present in different parts of the code.
- Integration with DP-controller is not fully confirmed. The threshold for when dpReached is True needs to be tuned.
- Integration with beluga.launch such that the node is running and ready for action.

## Notes
- The object 'pipeline' is constantly requested to update the bool isDetected.