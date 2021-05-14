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
        self.quaternion = []
        self.callback = FnVoid_VoidP_DataP(self.imu_data_handler)

    def imu_data_handler(self, ctx, data):
        rospy.loginfo('%s -> %s' % (self.device.address, parse_value(data)))

        # clear current quaternion data
        del self.quaternion[:]

        # convert quaternion to String
        quaternion_string = parse_value(data).__str__()

        # find all float values in the string
        quaternion = re.findall(r"[+-]? *(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?", quaternion_string)

        # get each quaternion value
        quaternion_w = float(quaternion[0])
        quaternion_x = float(quaternion[1])
        quaternion_y = float(quaternion[2])
        quaternion_z = float(quaternion[3])

        self.quaternion.append(quaternion_w)
        self.quaternion.append(quaternion_x)
        self.quaternion.append(quaternion_y)
        self.quaternion.append(quaternion_z)

        # quaternion = copy.deepcopy(data)


class ViewController2D:

    def __init__(self):
        global mac_address
        self.imu = MetaWear(mac_address)

        self.calibrated = False

        # arrays to store imu quationion data during calibration
        self.calibration_quaternion_w = []
        self.calibration_quaternion_x = []
        self.calibration_quaternion_y = []
        self.calibration_quaternion_z = []

        # calibrated imu quaternion data (mean value)
        self.calibrated_quaternion_w = 0.0
        self.calibrated_quaternion_x = 0.0
        self.calibrated_quaternion_y = 0.0
        self.calibrated_quaternion_z = 0.0

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
        # configure imu, subscribe quaternion signal
        for s in self.states:
            rospy.loginfo('Configuring imu')
            # min connection interval, max connection interval, latency, timeout
            libmetawear.mbl_mw_settings_set_connection_parameters(s.device.board, 7.5, 7.5, 0, 6000)
            sleep(1.5)

            libmetawear.mbl_mw_sensor_fusion_set_mode(s.device.board, SensorFusionMode.NDOF)
            libmetawear.mbl_mw_sensor_fusion_set_acc_range(s.device.board, SensorFusionAccRange._8G)
            libmetawear.mbl_mw_sensor_fusion_set_gyro_range(s.device.board, SensorFusionGyroRange._2000DPS)
            libmetawear.mbl_mw_sensor_fusion_write_config(s.device.board)

            signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(s.device.board, SensorFusionData.QUATERNION)
            libmetawear.mbl_mw_datasignal_subscribe(signal, None, s.callback)

            libmetawear.mbl_mw_sensor_fusion_enable_data(s.device.board, SensorFusionData.QUATERNION)
            libmetawear.mbl_mw_sensor_fusion_start(s.device.board)

    def get_quaternion_data(self):
        # connect to imu
        if not self.status_connected:
            self.status_connected = self.connect_imu()

            # if connection has failed stop
            if not self.status_connected:
                return

            self.configure_imu()

        # if no quaternion data has been found so far wait
        while not self.states[0].quaternion:
            sleep(0.001)

        return self.states[0].quaternion

    def quaternion_to_euler(self, w, x, y, z):
        """
            Convert a quaternion into euler angles (roll, pitch, yaw) (radian)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return [roll, pitch, yaw]  # in radians

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
            Convert a euler (radian as input) into quaternion
        """
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qw, qx, qy, qz]

    def calculate_head_orientation(self, quaternion):

        not_normalized_euler_radian = self.quaternion_to_euler(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

        # normalize it by subtracting current euler values from calibrated euler values
        # somehow roll axis is mirror-inverted --> subtract calibrated euler value from current euler value
        normalized_euler_roll = not_normalized_euler_radian[0] - self.calibrated_euler_roll
        normalized_euler_pitch = self.calibrated_euler_pitch - not_normalized_euler_radian[1]
        normalized_euler_yaw = self.calibrated_euler_yaw - not_normalized_euler_radian[2]

        normalized_quaternion = self.euler_to_quaternion(normalized_euler_roll, normalized_euler_pitch,
                                                         normalized_euler_yaw)

        quaternion = [normalized_quaternion[0], normalized_quaternion[1], normalized_quaternion[2],
                      normalized_quaternion[3]]

        return quaternion

    def detect_head_orientation(self):

        not_normalized_quaternion = self.get_quaternion_data()

        # take last quaternion data as calibrated quaternion data
        if not self.calibrated:

            self.calibrated_quaternion_w = self.calibration_quaternion_w[len(self.calibration_quaternion_w)-1]
            self.calibrated_quaternion_x = self.calibration_quaternion_x[len(self.calibration_quaternion_x)-1]
            self.calibrated_quaternion_y = self.calibration_quaternion_y[len(self.calibration_quaternion_y)-1]
            self.calibrated_quaternion_z = self.calibration_quaternion_z[len(self.calibration_quaternion_z)-1]

            # convert it in euler vector and use euler vector as calibrated values (here in radian)
            euler_radian = self.quaternion_to_euler(
                self.calibrated_quaternion_w, self.calibrated_quaternion_x, self.calibrated_quaternion_y,
                self.calibrated_quaternion_z)

            # save it as calibrated euler values
            self.calibrated_euler_roll = euler_radian[0]
            self.calibrated_euler_pitch = euler_radian[1]
            self.calibrated_euler_yaw = euler_radian[2]

            self.calibrated = True

        # 'normalize' quaternion
        quaternion = self.calculate_head_orientation(not_normalized_quaternion)

        return quaternion

    def unsubscribe_imu_signal(self):
        # stop reading quaternion, unsubscribe quaternion signal
        for s in self.states:
            libmetawear.mbl_mw_sensor_fusion_stop(s.device.board)

            signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(s.device.board, SensorFusionData.QUATERNION)
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

            # get imu data during 2D-calibration to calculate mean value (the value we are looking forward)
            quaternion = self.get_quaternion_data()

            self.calibration_quaternion_w.append(quaternion[0])
            self.calibration_quaternion_x.append(quaternion[1])
            self.calibration_quaternion_y.append(quaternion[2])
            self.calibration_quaternion_z.append(quaternion[3])
        else:
            self.reset_calibration()
            self.start_calibration()

    def reset_calibration(self):
        # clear calibration data
        del self.calibration_quaternion_w[:]
        del self.calibration_quaternion_x[:]
        del self.calibration_quaternion_y[:]
        del self.calibration_quaternion_z[:]
        self.calibrated = False


class Error(Exception):
    """Base class for other exceptions"""
    pass


class IMUConnectionError(Error):
    """Error when connecting to IMU."""
    pass
