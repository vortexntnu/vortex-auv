#!/usr/bin/env python

import numpy as np

import rospy

import tf2_ros
from vortex_msgs.msg import Object
from geometry_msgs.msg import Point, Wrench
import tf.transformations as tft

from auv_model import AUVModel


class LocalPIDNode:
    """
    A PID controller for regulating objects in local camera frame. Used when Perception needs to center camera on something. The incomming topic
    """

    def __init__(self):
        rospy.init_node("local_pid_node")

        self.parent_frame = "odom"
        self.child_frame = "base_link"

        control_bandwidths = [
            0.6,
            1,
            0.6,
            0.6,
            0.6,
            0.6,
        ]  # Tune the PID control bandwidths for [surge, sway, heave, roll, pitch, yaw]
        relative_damping_ratios = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        """Initialize Fossen's equations"""
        m = rospy.get_param("/controllers/vtf/model_parameters/mass")
        r_g = rospy.get_param("/controllers/vtf/model_parameters/center_of_mass")
        r_b = rospy.get_param("/controllers/vtf/model_parameters/center_of_buoyancy")
        inertia = np.array(rospy.get_param("/controllers/vtf/model_parameters/inertia"))
        volume = rospy.get_param("/controllers/vtf/model_parameters/volume")
        M_A = np.diag(rospy.get_param("/controllers/vtf/model_parameters/M_A"))
        D = -np.diag(rospy.get_param("/controllers/vtf/model_parameters/D"))
        rho = rospy.get_param("/controllers/vtf/model_parameters/water_density")
        g = 9.81

        self.auv_model = AUVModel(m, r_g, r_b, inertia, volume, M_A, D, rho=rho, g=g)

        self.Kp, self.Kd, self.Ki = self.pole_placement(
            control_bandwidths, relative_damping_ratios
        )

        self.perception_sub = rospy.Subscriber(
            "/pfps/error_img", Point, self.local_error_cb
        )
        self.tau_pub = rospy.Publisher("/pfps/local_control", Wrench, queue_size=1)
        # TF between odom and body
        self.__tfBuffer = tf2_ros.Buffer()

        # The init will only continue if a transform between parent frame and child frame can be found
        while (
            self.__tfBuffer.can_transform(
                self.parent_frame, self.child_frame, rospy.Time()
            )
            == 0
        ):
            try:
                rospy.loginfo(
                    "No transform between "
                    + str(self.parent_frame)
                    + " and "
                    + str(self.child_frame)
                )
                rospy.sleep(2)
            except:  # , tf2_ros.ExtrapolationException  (tf2_ros.LookupException, tf2_ros.ConnectivityException)
                rospy.sleep(2)
                continue

        rospy.loginfo(
            "Transform between "
            + str(self.parent_frame)
            + " and "
            + str(self.child_frame)
            + "found."
        )

        # Set initial yaw as reference
        tf_lookup_init = self.__tfBuffer.lookup_transform(
            self.parent_frame, self.self.child_frame, rospy.Time(), rospy.Duration(5)
        )
        quat = np.array(
            [
                tf_lookup_init.transform.rotation.x,
                tf_lookup_init.transform.rotation.y,
                tf_lookup_init.transform.rotation.z,
                tf_lookup_init.transform.rotation.w,
            ]
        )

        self.psi_ref, _, _ = tft.euler_from_quaternion(quat, axes="szyx")
        self.z_ref = rospy.get_param("/fsm/operating_depth")

    def pole_placement(self, control_bandwidths, relative_damping_ratios):
        """
        Implementation of algorithm 15.2 in (Fossen, 2021)
        """
        natural_frequencies = np.empty((6,))

        for damp, i in enumerate(relative_damping_ratios):
            cursed_factor = 1 / (
                np.sqrt(1 - 2 * damp**2 + np.sqrt(4 * damp**4 - 4 * damp**2 + 2))
            )
            natural_frequencies[i] = cursed_factor * control_bandwidths

        Omega_n = np.diag(natural_frequencies)
        Z = np.diag(relative_damping_ratios)
        M = self.auv_model.M

        Kp = np.matmul(M, Omega_n**2)
        Kd = 2 * np.matmul(M, np.matmul(Z, Omega_n))
        Ki = (1 / 10) * np.matmul(Kp, Omega_n)

        return Kp, Kd, Ki

    def publish_wrench(self, tau):
        w = Wrench()

        w.force.x = tau[0]
        w.force.y = tau[1]
        w.force.z = tau[2]

        w.torque.x = tau[3]
        w.torque.y = tau[4]
        w.torque.z = tau[5]

        self.tau_pub.publish(w)

    def local_error_cb(self, msg):
        tf_lookup_baselink = self.__tfBuffer.lookup_transform(
            self.parent_frame, self.self.child_frame, rospy.Time(), rospy.Duration(5)
        )

        pos = np.array(
            [
                tf_lookup_baselink.transform.translation.x,
                tf_lookup_baselink.transform.translation.y,
                tf_lookup_baselink.transform.translation.z,
            ]
        )

        quat = np.array(
            [
                tf_lookup_baselink.transform.rotation.x,
                tf_lookup_baselink.transform.rotation.y,
                tf_lookup_baselink.transform.rotation.z,
                tf_lookup_baselink.transform.rotation.w,
            ]
        )

        psi, _, _ = tft.euler_from_quaternion(quat, axes="szyx")

        error_x = msg.x
        error_y = msg.y
        error_z = self.z_ref - pos[2]

        error_roll = 0
        error_pitch = 0
        error_yaw = self.psi_ref - psi

        error = np.array(
            [error_x, error_y, error_z, error_roll, error_pitch, error_yaw]
        )

        tau_body = -np.matmul(self.Kp, error)
        self.publish_wrench(tau_body)


if __name__ == "__main__":
    while not rospy.is_shutdown():
        try:
            local_pid = LocalPIDNode()

            rospy.spin()
        except rospy.ROSInterruptException:
            pass
