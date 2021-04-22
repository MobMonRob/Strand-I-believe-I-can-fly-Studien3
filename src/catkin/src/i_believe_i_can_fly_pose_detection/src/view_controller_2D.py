import rospy

from mbientlab.metawear import MetaWear, libmetawear, parse_value, cbindings, SensorFusionData, SensorFusionMode
from mbientlab.metawear import FnVoid_VoidP_UByte_Long_UByteP_UByte, FnVoid_VoidP_DataP, byref, LogDownloadHandler, FnVoid_VoidP_UInt_UInt
from mbientlab.metawear.cbindings import *

from time import sleep
from threading import Event
import os
import re

# from imu_state import State

mac_address = 'CB:4C:61:C2:62:39'

# TODO: Change it so you can also use the simulator without imu. Connection error should not make it crash
# TODO: Maybe we need to log quaternion data during calibration and take the mean value to check
#       at which quaternion data we are looking forward
# TODO: Delete writing files


class State:
    def __init__(self, device):
        self.device = device
        self.quaternion = []
        self.callback = FnVoid_VoidP_DataP(self.imu_data_handler)

    def imu_data_handler(self, ctx, data):
        rospy.loginfo('%s -> %s' % (self.device.address, parse_value(data)))

        # clear current quaternion data
        del self.quaternion[:]

        save_path = '/home/informatik/Desktop/Strand-I-believe-I-can-fly-Studien3'
        file_name = "Quaternion.txt"

        completeName = os.path.join(save_path, file_name)
        file1 = open(completeName, "a")

        if not self.quaternion:
            file1.write("Quaternion has been cleared.\n")

        file1.write("%s -> %s\n" % (self.device.address, parse_value(data)))

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

        file1.write(quaternion_string + "\n")
        file1.write("%f, %f, %f, %f\n" % (self.quaternion[0], self.quaternion[1], self.quaternion[2], self.quaternion[3]))
        file1.write("W: %f, X: %f, Y: %f, Z: %f\n\n\n" % (quaternion_w, quaternion_x, quaternion_y, quaternion_z))
        file1.close()

        # quaternion = copy.deepcopy(data)


class ViewController2D:

    def __init__(self):
        global mac_address
        self.imu = MetaWear(mac_address)
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

    def detect_head_orientation(self):

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

        quaternion = self.states[0].quaternion
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


class Error(Exception):
    """Base class for other exceptions"""
    pass


class IMUConnectionError(Error):
    """Error when connecting to IMU."""
    pass
