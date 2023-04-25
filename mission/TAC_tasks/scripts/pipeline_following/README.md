## Futher improvements
- Currently the the z value of the goal position is set to the same as the position of the drone. This should be improved.
- The integration with the task manager is not tested. The idea is that the task manager also needs to be able to shut down while loops within the different states as well as the main node. Therefore it is present in different parts of the code.
- Integration with DP-controller is not fully confirmed. The threshold for when dpReached is True needs to be tuned. Also note that the integration part of the DP-controller should be deactivated.

## Note to self
- The object 'pipeline' is constantly requasted to update the bool isDetected. 