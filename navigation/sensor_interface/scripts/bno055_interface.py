#!/usr/bin/env python

import csv
import rospkg
import rospy

from sensor_msgs.msg import Imu, MagneticField, Temperature
from geometry_msgs.msg import Vector3, Vector3Stamped, Quaternion
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_interface.srv import SaveImuCalibration, SaveImuCalibrationResponse
from sensor_interface.srv import LoadImuCalibration, LoadImuCalibrationResponse

from Adafruit_BNO055 import BNO055


class Bno055InterfaceNode(object):
    def __init__(self):
        rospy.init_node('imu_node')

        self.pub_euler = rospy.Publisher(
            'sensors/imu/euler',
            Vector3Stamped,
            queue_size=1)
        self.pub_diagnostics = rospy.Publisher(
            'sensors/imu/diagnostics',
            DiagnosticStatus,
            queue_size=1)
        self.pub_imu = rospy.Publisher(
            'sensors/imu/data',
            Imu,
            queue_size=1)

        self.srv_save_calibration = rospy.Service(
            'sensors/imu/save_calibration',
            SaveImuCalibration,
            self.save_calibration)
        self.srv_load_calibration = rospy.Service(
            'sensors/imu/load_calibration',
            LoadImuCalibration,
            self.load_calibration)

        try:
            mode_name = rospy.get_param('sensors/bno055/mode')
        except KeyError:
            rospy.logerr('Could not read mode parameter, defaulting to IMU mode.')
            mode_name = 'IMU'

        if mode_name == 'IMU':
            mode = BNO055.OPERATION_MODE_IMUPLUS
        elif mode_name == 'NDOF':
            mode = BNO055.OPERATION_MODE_NDOF
        else:
            rospy.logerr('Invalid mode parameter, defaulting to IMU mode.')
            mode_name = 'IMU'
            mode = BNO055.OPERATION_MODE_IMUPLUS

        try:
            self.bno = BNO055.BNO055(rst='P9_12')
        except RuntimeError:
            rospy.logfatal('Unsupported hardware, intiating clean shutdown.')
            rospy.signal_shutdown('')
            return

        if not self.bno.begin(mode):
            rospy.logfatal('Failed to initialise BNO055! Is the sensor connected?')
            raise rospy.ROSInitException('Failed to initialise BNO055! Is the sensor connected?')

        self.status, self.self_test, self.error = self.bno.get_system_status()
        rospy.logdebug("System status: %s", self.status)
        rospy.logdebug("Self test result (0x0F is normal): 0x%02X", self.self_test)
        if self.status == 0x01:
            rospy.logwarn("System status: 0x%02X\n (see datasheet section 4.3.59).", self.status)

        self.sw_v, \
            self.bootloader_v, \
            self.accelerometer_id, \
            self.magnetometer_id, \
            self.gyro_id = self.bno.get_revision()

        rospy.logdebug(("Software version: %s\n" "Bootloader version: %s\n"
                        "Accelerometer ID: 0x%02X\n" "Magnetometer ID: 0x%02X\n"
                        "Gyroscope ID: 0x%02X\n"), self.sw_v,
                       self.bootloader_v, self.accelerometer_id, self.accelerometer_id, self.gyro_id)

        rospy.loginfo('Initialized in %s mode.', mode_name)

        self.bno.set_axis_remap(
            BNO055.AXIS_REMAP_Y,
            BNO055.AXIS_REMAP_X,
            BNO055.AXIS_REMAP_Z,
            BNO055.AXIS_REMAP_POSITIVE,
            BNO055.AXIS_REMAP_POSITIVE,
            BNO055.AXIS_REMAP_POSITIVE
        )
        rospy.logdebug('IMU axis config is {0}'.format(self.bno.get_axis_remap()))

        self.talker()

    def get_diagnostic(self):
        diag_msg = DiagnosticStatus()

        sys_status, gyro_status, accel_status, mag_status = self.bno.get_calibration_status()
        sys = KeyValue("System status", str(sys_status))
        gyro = KeyValue("Gyro calibration status", str(gyro_status))
        accel = KeyValue("Accelerometer calibration status", str(accel_status))
        mag = KeyValue("Magnetometer calibration status", str(mag_status))

        diag_msg.values = [sys, gyro, accel, mag]
        return diag_msg

    def talker(self):
        imu_msg = Imu()
        # imu_euler_msg = Vector3Stamped()
        # imu_temp_msg = Temperature()
        # imu_mag_msg = MagneticField()
        # imu_diag_msg = DiagnosticStatus()
        imu_diag_msg_prev = DiagnosticStatus()

        while not rospy.is_shutdown():
            x, y, z, w = self.bno.read_quaternion()
            # heading, roll, pitch = self.bno.read_euler()
            # mag_x, mag_y, mag_z = self.bno.read_magnetometer()
            # temperature = self.bno.read_temp()
            gyro_x, gyro_y, gyro_z = self.bno.read_gyroscope()

            imu_msg.header.stamp = rospy.get_rostime()
            imu_msg.orientation = Quaternion(x, y, z, w)
            imu_msg.angular_velocity = Vector3(gyro_x, gyro_y, gyro_z)
            imu_msg.orientation_covariance[0] = -1
            imu_msg.angular_velocity_covariance[0] = -1
            imu_msg.linear_acceleration_covariance[0] = -1

            # imu_euler_msg.header.stamp = rospy.get_rostime()
            # imu_euler_msg.vector = Vector3(heading, roll, pitch)

            # imu_temp_msg.header.stamp = rospy.get_rostime()
            # imu_temp_msg.temperature = temperature

            # imu_mag_msg.header.stamp = rospy.get_rostime()
            # imu_mag_msg.magnetic_field = Vector3(mag_x, mag_y, mag_z)

            self.pub_imu.publish(imu_msg)
            # self.pub_euler.publish(imu_euler_msg)
            # self.pub_imu_temp.publish(imu_temp_msg)
            # self.pub_mag.publish(imu_mag_msg)

            imu_diag_msg = self.get_diagnostic()
            if imu_diag_msg.values != imu_diag_msg_prev.values:
                imu_diag_msg_prev = imu_diag_msg
                self.pub_diagnostics.publish(imu_diag_msg)

            rospy.Rate(10).sleep()

    def save_calibration(self, req):
        values = self.bno.get_calibration()
        path = rospkg.RosPack().get_path('sensor_interface') + "/calibration.csv"
        with open(path, 'w') as outfile:
            writer = csv.writer(outfile)
            writer.writerow(values)

        rospy.loginfo("Successfully saved calibration data to %s", path)
        return SaveImuCalibrationResponse()

    def load_calibration(self, req):
        path = rospkg.RosPack().get_path('sensor_interface') + "/calibration.csv"
        try:
            with open(path) as infile:
                reader = csv.reader(infile)
                values = next(reader)
            values = [int(value) for value in values]
            self.bno.set_calibration(values)
            rospy.loginfo("Successfully loaded calibration data from %s", path)
        except IOError:
            rospy.logwarn("Unable to open %s", path)
        except ValueError as e:
            rospy.logwarn("Unexpected format: %s", str(e))
        return LoadImuCalibrationResponse()


if __name__ == '__main__':
    try:
        imu_node = Bno055InterfaceNode()
        rospy.spin()
    except IOError:
        rospy.logerr('IOError caught, shutting down.')
    except rospy.ROSInterruptException:
        pass
