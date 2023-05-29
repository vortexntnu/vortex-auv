#!/usr/bin/python3

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer

from circle import Circle
from reach_depth import ReachDepth


def main():
    rospy.init_node("nortek_fsm")

    rospy.sleep(rospy.get_param("/fsm/time_to_launch"))

    #rospy.wait_for_service(
    #    "send_positions"
    #)

    showcase_fsm = StateMachine(outcomes=["preempted", "succeeded", "aborted"])

    with showcase_fsm:

        StateMachine.add("REACH_DEPTH",
                         ReachDepth(),
                         transitions={"succeeded": "CIRCLE"})

        StateMachine.add("CIRCLE_CW",
                         Circle(),
                         transitions={"succeeded": "CIRCLE_CCW"})
        StateMachine.add("CIRCLE_CCW",
                         Circle(ccw=True),
                         transitions={"succeeded": "CIRCLE_CW_POINT"})

        StateMachine.add("CIRCLE_CW_POINT",
                         Circle(is_path_dependent=False),
                         transitions={"succeeded": "ANTICIRCLE_POINT"})
        StateMachine.add("CIRCLE_CCW_POINT",
                         Circle(ccw=True, is_path_dependent=False))

    intro_server = IntrospectionServer(str(rospy.get_name()), showcase_fsm,
                                       "/SM_ROOT")
    intro_server.start()

    try:
        showcase_fsm.execute()
        intro_server.stop()

    except Exception as e:
        rospy.loginfo("Prequalification test failed: %s" % e)


if __name__ == "__main__":
    main()
