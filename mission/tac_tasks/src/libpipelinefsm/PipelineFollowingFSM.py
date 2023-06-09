#!/usr/bin/python3

import rospy
from smach import StateMachine

from libpipelinefsm.PipelineExecute import PipelineExecute
from libpipelinefsm.PipelineConverge import PipelineConverge


class PipelineFollowingFSM():

    def __init__(self):
        rospy.init_node("tac_pipeline_fsm")

    def main(self):
        pipeline_following_sm = StateMachine(outcomes=["done"])
        with pipeline_following_sm:
            StateMachine.add("PIPELINE_CONVERGE",
                             PipelineConverge(),
                             transitions={
                                 "preempted": "done",
                                 "succeeded": "PIPELINE_EXECUTE"
                             })
            StateMachine.add("PIPELINE_EXECUTE",
                             PipelineExecute(),
                             transitions={
                                 "preempted": "PIPELINE_CONVERGE",
                                 "succeeded": "done"
                             })
        try:
            #Execute SMACH plan
            pipeline_following_sm.execute()
            rospy.loginfo("Exiting Pipeline Following")

        except Exception as e:
            rospy.loginfo("State machine failed: %s" % e)
