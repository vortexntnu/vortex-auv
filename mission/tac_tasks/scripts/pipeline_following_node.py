#! /usr/bin/env python3

import rospy

from libpipelinefsm.PipelineFollowingFSM import PipelineFollowingFSM

if __name__ == "__main__":
    pipeline_fsm = PipelineFollowingFSM()
    if not rospy.is_shutdown():
        pipeline_fsm.main()
