## Guidance interface
The mission planner/state machine communicates with the guidance nodes exclusively through this node.
This is done through the 'move' action server. Here, a goal is sent alongside the guidance block to be used.
The guidance interface then passes the required data to the addressed guidance node.