#!/usr/bin/python3

import rospy
from dynamic_reconfigure.server import Server
from vortex_msgs.srv import SetVelocityRequest, SetVelocity
from velocity_controller.cfg import vel_controllerConfig
from geometry_msgs.msg import Twist


def callback(config, level):
    velocity_server = rospy.get_param(
        "/controllers/velocity_controller/desired_velocity_topic")

    rospy.wait_for_service(velocity_server)
    set_velocity = rospy.ServiceProxy(velocity_server, SetVelocity)

    twist = Twist()

    twist.linear.x = config.x_vel
    twist.linear.y = config.y_vel
    twist.linear.z = config.z_vel

    twist.angular.x = config.roll_vel
    twist.angular.y = config.pitch_vel
    twist.angular.z = config.yaw_vel

    msg = SetVelocityRequest()
    msg.desired_velocity = twist
    msg.active = config.active

    set_velocity(msg)

    return config


if __name__ == "__main__":
    rospy.init_node("velocity_config_server")
    srv = Server(vel_controllerConfig, callback)
    rospy.spin()
