#!/usr/bin/env python3
"""Exercise the actual ROS node with deterministic synthetic sensor messages."""

import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Imu
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def main():
    parameters = {
        'estimator_backend': os.environ.get('ESKF_TEST_BACKEND', 'eskf'),
        'frame_prefix': 'test',
        'publish_debug': True,
        'publish_tf': False,
        'publish_pose': True,
        'publish_twist': True,
        'publish_biases': False,
        'publish_nis': True,
        'publish_rate_ms': 10,
        'topics.imu': 'imu',
        'topics.dvl_twist': 'dvl',
        'topics.pressure_sensor': 'pressure',
        'topics.odom': 'odom',
        'topics.pose': 'pose',
        'topics.twist': 'twist',
        'sensors.imu.use_tf_transform': False,
        'sensors.dvl.use_tf_transform': False,
        'sensors.pressure.use_tf_transform': False,
        'sensors.dvl.use_msg_noise': True,
        'sensors.pressure.use_msg_noise': True,
        'sensors.pressure.measurement_noise': 40000.0,
        'gravity.acceleration': 9.81,
        'water.density': 1000.0,
        'atmosphere.pressure': 101325.0,
        'pressure_is_gauge': True,
        'diag_Q_std': [0.01] * 12,
        'diag_p_init': [0.1] * 15,
        'max_imu_dt': 0.2,
        'max_estimate_age': 0.3,
        'max_aiding_skew': 0.05,
    }
    for sensor in ('imu', 'dvl'):
        parameters[f'sensors.{sensor}.transform.r'] = [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
    for sensor in ('imu', 'dvl', 'pressure'):
        parameters[f'sensors.{sensor}.transform.t'] = [0.0, 0.0, 0.0]
    rclpy.init()
    node = rclpy.create_node('eskf_contract_test')
    odometry, validity, twists = [], [], []
    node.create_subscription(
        Odometry, 'eskf/odom', odometry.append, qos_profile_sensor_data
    )
    node.create_subscription(
        Bool, 'eskf/valid', lambda msg: validity.append(msg.data), 1
    )
    node.create_subscription(
        TwistWithCovarianceStamped, 'eskf/twist', twists.append, qos_profile_sensor_data
    )
    imu_pub = node.create_publisher(Imu, 'imu', qos_profile_sensor_data)
    dvl_pub = node.create_publisher(
        TwistWithCovarianceStamped, 'dvl', qos_profile_sensor_data
    )
    pressure_pub = node.create_publisher(
        FluidPressure, 'pressure', qos_profile_sensor_data
    )

    def spin(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.005)

    def samples(duration, aiding=False, depth=0.0):
        end = time.monotonic() + duration
        count = 0
        while time.monotonic() < end:
            stamp = node.get_clock().now().to_msg()
            msg = Imu()
            msg.header.stamp = stamp
            msg.linear_acceleration.z = -9.81
            imu_pub.publish(msg)
            spin(0.01)
            if aiding and count % 5 == 0:
                dvl = TwistWithCovarianceStamped()
                dvl.header.stamp = stamp
                for index in (0, 7, 14):
                    dvl.twist.covariance[index] = 0.01
                dvl_pub.publish(dvl)
                pressure = FluidPressure()
                pressure.header.stamp = stamp
                pressure.fluid_pressure = depth * 9810
                pressure.variance = 40000.0
                pressure_pub.publish(pressure)
            count += 1

    with tempfile.TemporaryDirectory() as directory:
        config = Path(directory) / 'params.yaml'
        config.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
        with (Path(directory) / 'node.log').open('w+') as log:
            process = subprocess.Popen(
                [
                    'ros2',
                    'run',
                    'eskf',
                    'eskf_node',
                    '--ros-args',
                    '--params-file',
                    str(config),
                ],
                stdout=log,
                stderr=log,
                start_new_session=True,
            )
            try:
                deadline = time.monotonic() + 15
                while (
                    imu_pub.get_subscription_count() == 0
                    and time.monotonic() < deadline
                ):
                    spin(0.1)
                assert process.poll() is None, 'Node process running'
                assert imu_pub.get_subscription_count() > 0, 'Node startup'
                spin(0.1)
                assert not odometry, 'No initialized-state odometry before measurements'
                samples(0.3)
                assert not odometry, 'Await both DVL and depth before valid navigation'
                samples(0.6, aiding=True)
                spin(0.03)
                assert odometry, 'Odometry after IMU and aiding'
                assert twists, 'Twist after IMU and aiding'
                assert validity[-1], 'Valid state after IMU and aiding'
                latest = odometry[-1]
                assert latest.header.frame_id == 'test/odom'
                assert latest.child_frame_id == 'test/base_link'
                assert twists[-1].header.frame_id == 'test/base_link'
                assert abs(latest.pose.pose.position.z) < 1e-5
                assert latest.twist.covariance[21] > 0, 'Angular uncertainty populated'
                samples(0.2, aiding=True, depth=10000.0)
                assert abs(odometry[-1].pose.pose.position.z) < 1e-3, (
                    'Pressure outlier rejected'
                )
                spin(0.45)
                assert not validity[-1], 'Stale estimate marked invalid'
                count = len(odometry)
                spin(0.1)
                assert len(odometry) == count, 'Stale estimate not restamped/published'
                samples(0.2, aiding=True)
                assert not validity[-1], (
                    'Gap fault remains invalid until explicit reset'
                )
                client = node.create_client(Trigger, 'eskf/reset')
                assert client.wait_for_service(timeout_sec=2)
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=2)
                assert future.result().success
                samples(0.6, aiding=True)
                assert validity[-1], 'Navigation recovers after explicit reset'
                print(
                    'ROS startup, frames, covariance, outlier, staleness, gap and reset contracts passed'
                )
            except BaseException:
                log.flush()
                log.seek(0)
                print(log.read())
                raise
            finally:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    main()
