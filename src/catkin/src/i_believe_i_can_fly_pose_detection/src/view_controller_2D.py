import rospy

from mbientlab.metawear import MetaWear, libmetawear, parse_value, cbindings, SensorFusionData, SensorFusionMode
from mbientlab.metawear import FnVoid_VoidP_UByte_Long_UByteP_UByte, FnVoid_VoidP_DataP, byref, LogDownloadHandler, FnVoid_VoidP_UInt_UInt
from mbientlab.metawear.cbindings import *

from time import sleep
from threading import Event
import os
import re
import math
import numpy as np

# from imu_state import State

mac_address = 'CB:4C:61:C2:62:39'

# TODO: Change it so you can also use the simulator without imu.
#       Trying to connect should not make the i_believe_i_can_fly_pose_detection node crash.


class State:
    def __init__(self, device):
        self.device = device
        self.euler = []
        self.callback = FnVoid_VoidP_DataP(self.imu_data_handler)

    def imu_data_handler(self, ctx, data):
        rospy.loginfo('%s -> %s' % (self.device.address, parse_value(data)))

        # convert euler to String
        euler_string = parse_value(data).__str__()

        # find all float values in the string
        euler = re.findall(r"[+-]? *(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?", euler_string)

        # get each euler value (here in degree)
        euler_heading = float(euler[0])
        euler_pitch = float(euler[1])
        euler_roll = float(euler[2])
        euler_yaw = float(euler[3])

        # clear current euler data
        # del self.quaternion[:]
        del self.euler[:]

        # store them as radian
        self.euler.append(math.radians(euler_heading))
        self.euler.append(math.radians(euler_roll))
        self.euler.append(math.radians(euler_pitch))
        self.euler.append(math.radians(euler_yaw))

        # quaternion = copy.deepcopy(data)


class ViewController2D:

    def __init__(self):
        global mac_address
        self.imu = MetaWear(mac_address)

        self.calibrated = False

        # arrays to store imu euler data during calibration
        self.calibration_euler_heading = []
        self.calibration_euler_roll = []
        self.calibration_euler_pitch = []
        self.calibration_euler_yaw = []

        # calibrated euler angles (in radian)
        self.calibrated_euler_roll = 0.0
        self.calibrated_euler_pitch = 0.0
        self.calibrated_euler_yaw = 0.0

        self.states = []
        self.status_connected = self.connect_imu()
        if self.status_connected:
            self.imu.on_disconnect = lambda status: self.on_disconnect()
            self.configure_imu()

    def connect_imu(self):
        if not self.imu.is_connected:
            try:
                self.imu.connect()
            except IMUConnectionError:
                rospy.logdebug('Cannot connect to IMU!')
                return False

        # in case connection to imu worked, add it to states (on_disconnect self.states will be cleared)
        self.states.append(State(self.imu))

        return True

    def configure_imu(self):
        # configure imu, subscribe euler signal
        for s in self.states:
            rospy.loginfo('Configuring imu')
            libmetawear.mbl_mw_settings_set_connection_parameters(s.device.board, 7.5, 7.5, 0, 6000)
            sleep(1.5)

            libmetawear.mbl_mw_sensor_fusion_set_mode(s.device.board, SensorFusionMode.NDOF)
            libmetawear.mbl_mw_sensor_fusion_set_acc_range(s.device.board, SensorFusionAccRange._8G)
            libmetawear.mbl_mw_sensor_fusion_set_gyro_range(s.device.board, SensorFusionGyroRange._2000DPS)
            libmetawear.mbl_mw_sensor_fusion_write_config(s.device.board)

            signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(s.device.board, SensorFusionData.EULER_ANGLE)
            libmetawear.mbl_mw_datasignal_subscribe(signal, None, s.callback)

            libmetawear.mbl_mw_sensor_fusion_enable_data(s.device.board, SensorFusionData.EULER_ANGLE)
            libmetawear.mbl_mw_sensor_fusion_start(s.device.board)

    def get_euler_data(self):
        # connect to imu
        if not self.status_connected:
            self.status_connected = self.connect_imu()

            # if connection has failed stop
            if not self.status_connected:
                return

            self.configure_imu()

        # if no euler data has been found so far wait
        while not self.states[0].euler:
            sleep(0.001)

        return self.states[0].euler

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
            Convert a euler (radian as input) into quaternion
        """
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qw, qx, qy, qz]

    def calculate_head_orientation(self, not_normalized_euler_radian):

        not_normalized_euler_roll = not_normalized_euler_radian[1]
        not_normalized_euler_pitch = not_normalized_euler_radian[2]
        not_normalized_euler_yaw = not_normalized_euler_radian[3]

        # normalize it by subtracting current euler values from calibrated euler values
        # somehow yaw is mirror-inverted --> subtract calibrated euler value from current euler value
        normalized_euler_roll = self.calibrated_euler_roll - not_normalized_euler_roll
        normalized_euler_pitch = self.calibrated_euler_pitch - not_normalized_euler_pitch
        normalized_euler_yaw = not_normalized_euler_yaw - self.calibrated_euler_yaw

        normalized_quaternion = self.euler_to_quaternion(normalized_euler_roll, normalized_euler_pitch,
                                                         normalized_euler_yaw)

        quaternion = [normalized_quaternion[0], normalized_quaternion[1], normalized_quaternion[2],
                      normalized_quaternion[3]]

        return quaternion

    def detect_head_orientation(self):

        not_normalized_euler = self.get_euler_data()

        # take last euler data as calibrated euler data
        if not self.calibrated:

            self.calibrated_euler_roll = self.calibration_euler_roll[len(self.calibration_euler_roll)-1]
            self.calibrated_euler_pitch = self.calibration_euler_pitch[len(self.calibration_euler_pitch)-1]
            self.calibrated_euler_yaw = self.calibration_euler_yaw[len(self.calibration_euler_yaw)-1]

            self.calibrated = True

        # get head orientation
        quaternion = self.calculate_head_orientation(not_normalized_euler)

        return quaternion

    def unsubscribe_imu_signal(self):
        # stop reading euler, unsubscribe euler signal
        for s in self.states:
            libmetawear.mbl_mw_sensor_fusion_stop(s.device.board)

            signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(s.device.board, SensorFusionData.EULER_ANGLE)
            libmetawear.mbl_mw_datasignal_unsubscribe(signal)
            libmetawear.mbl_mw_debug_disconnect(s.device.board)

    def disconnect_imu(self):
        self.unsubscribe_imu_signal()

        self.imu.disconnect()

    def on_disconnect(self):
        self.status_connected = False

        # clearing the list of IMUs
        del self.states[:]
        rospy.logwarn('Disconnected to IMU!')

    def start_calibration(self):

        if not self.calibrated:

            # get imu data during 2D-calibration to get the value we are looking forward
            euler = self.get_euler_data()

            self.calibration_euler_heading.append(euler[0])
            self.calibration_euler_roll.append(euler[1])
            self.calibration_euler_pitch.append(euler[2])
            self.calibration_euler_yaw.append(euler[3])

        else:
            self.reset_calibration()
            self.start_calibration()

    def reset_calibration(self):
        # clear calibration data
        del self.calibration_euler_heading[:]
        del self.calibration_euler_roll[:]
        del self.calibration_euler_pitch[:]
        del self.calibration_euler_yaw[:]

        self.calibrated = False


class Error(Exception):
    """Base class for other exceptions"""
    pass


class IMUConnectionError(Error):
    """Error when connecting to IMU."""
    pass
