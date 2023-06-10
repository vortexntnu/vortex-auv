#!/usr/bin/python3

import rospy
from smach import StateMachine

from libpipelinefsm.PipelineExecute import PipelineExecute
from libpipelinefsm.PipelineConverge import PipelineConverge
from libpipelinefsm.PipelineReturn import PipelineReturn


class PipelineFollowingFSM():

    def __init__(self):
        rospy.init_node("tac_pipeline_fsm")

        self.follow_depth = 0.0
        self.return_depth = -1
        self.margin = 0.3

    def main(self):
        pipeline_following_sm = StateMachine(outcomes=["done"])
        with pipeline_following_sm:
            StateMachine.add("PIPELINE_CONVERGE",
                             PipelineConverge(self.follow_depth),
                             transitions={
                                 "preempted": "done",
                                 "succeeded": "PIPELINE_EXECUTE"
                             })
            StateMachine.add("PIPELINE_EXECUTE",
                             PipelineExecute(self.follow_depth, self.margin),
                             transitions={
                                 "preempted": "PIPELINE_CONVERGE",
                                 "succeeded": "PIPELINE_RETURN"
                             })
            StateMachine.add("PIPELINE_RETURN",
                             PipelineReturn(self.return_depth, self.margin),
                             transitions={"succeeded": "PIPELINE_CONVERGE"})

        try:
            #Execute SMACH plan
            pipeline_following_sm.execute()
            rospy.loginfo("Exiting Pipeline Following")

        except Exception as e:
            rospy.loginfo("State machine failed: %s" % e)
