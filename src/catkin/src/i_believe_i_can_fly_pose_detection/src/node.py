#!/usr/bin/env python

import rospy
import sys
from calibrator import Calibrator2D
from fuzzy_controller_2D import FuzzyController2D
from view_controller_2D import ViewController2D
from point import Point
from skeleton import Skeleton
from pose import Pose
from i_believe_i_can_fly_common.msg import Reset as ResetMsg
from i_believe_i_can_fly_person_detection.msg import Skeleton as SkeletonMsg
from i_believe_i_can_fly_pose_detection.msg import Calibration as CalibrationMsg
from i_believe_i_can_fly_pose_detection.msg import Instruction as InstructionMsg
from i_believe_i_can_fly_pose_detection.msg import Instructions as InstructionsMsg
from i_believe_i_can_fly_pose_detection.msg import FOVInstructions as FOVInstructionsMsg

publisher_calibration = None
publisher_instructions = None
publisher_fov_instructions = None
calibrator_2D = None
fuzzy_controller_2D = None
view_controller_2D = None
frame = 0


def init_node():
    """
    Initializes ROS node 'i_believe_i_can_fly_pose_detection'.
    """
    global publisher_calibration, publisher_instructions, publisher_fov_instructions, calibrator_2D, fuzzy_controller_2D
    global view_controller_2D

    rospy.init_node('i_believe_i_can_fly_pose_detection',
                    log_level = (
                        rospy.DEBUG if rospy.get_param('/i_believe_i_can_fly_pose_detection/debug') else rospy.ERROR))
    publisher_instructions = rospy.Publisher('flight_instructions', InstructionsMsg, queue_size = 10)
    publisher_calibration = rospy.Publisher('calibration_status', CalibrationMsg, queue_size = 10)
    publisher_fov_instructions = rospy.Publisher('fov_instructions', FOVInstructionsMsg, queue_size = 10)
    rospy.Subscriber('i_believe_i_can_fly', ResetMsg, reset)

    if rospy.get_param('/i_believe_i_can_fly_pose_detection/mode') == '2D':
        calibrator_2D = Calibrator2D(publisher_calibration)
        fuzzy_controller_2D = FuzzyController2D()
        view_controller_2D = ViewController2D()
        rospy.Subscriber('person_detection', SkeletonMsg, detect_pose_2D)
    else:
        rospy.logerr('Invalid mode detected! Allowed values are: \'2D\'')
        sys.exit()
    rospy.on_shutdown(view_controller_2D.disconnect_imu)
    rospy.spin()


def return_number(number):
    """
    Returns number as string. If number is None, string 'None' is returned instead.
    :param number: number to be converted as string
    :return: number converted to string and round to 2 positions after decimal point or 'None' in case of given None
    """
    if number is None:
        return 'None'
    else:
        return str(round(number, 2))


def calibration_2D_finished(success, avg_arm_length, shoulder_distance):
    """
    Callback for the finished calibration process. Forwards the calibrated values to the actual Fuzzy Controller.
    :param success: True if calibration was successful, otherwise False
    :param avg_arm_length: calibrated average arm length
    :param shoulder_distance: calibrated shoulder distance
    """
    global fuzzy_controller_2D

    if success and fuzzy_controller_2D.calibrate(avg_arm_length, shoulder_distance):
        rospy.loginfo('Calibration finished successfully! Average arm length: %i, shoulder distance: %i',
                      avg_arm_length, shoulder_distance)
    else:
        rospy.logwarn('Calibration could not be finished!')


def detect_pose_2D(skeleton_msg):
    """
    Controls the actual program flow from the calibration to the actual flight instruction publication.
    :param skeleton_msg: Received skeleton message from the ROS node 'person_detection'.
    """
    global frame, calibrator_2D

    frame += 1
    keypoints = dict()
    for keypoint in skeleton_msg.keypoints:
        keypoints[keypoint.index] = Point(keypoint.x, keypoint.y, keypoint.accuracy, keypoint.index, keypoint.name)
    skeleton = Skeleton(frame, keypoints, 0.3)

    # check calibration status
    if not calibrator_2D.is_calibrated() and not calibrator_2D.is_calibrating():
        calibrator_2D.start_calibration(skeleton, calibration_2D_finished)
        view_controller_2D.reset_calibration()
    elif calibrator_2D.is_calibrating():
        calibrator_2D.skeleton_changed(skeleton)
        view_controller_2D.start_calibration()
    elif calibrator_2D.is_calibrated():
        if skeleton.check_for_important_keypoints():
            skeleton.transform_points()
            instructions = fuzzy_controller_2D.detect_pose(skeleton)
            publish_instructions(instructions)
            rospy.logdebug('%s, (frame %i with HD %s/HG %s/SD %s/HSD %s)', str(instructions), skeleton.frame,
                           return_number(skeleton.get_hand_distance()), return_number(
                    skeleton.get_hand_gradient()), return_number(skeleton.get_shoulder_distance()), return_number(
                    skeleton.get_hand_to_shoulder_distance()))
        else:
            publish_instructions({Pose.HOLD: 100.0})
            rospy.logwarn('Skeleton at frame %i is missing some important points!', frame)

        # if calibrated get head orientation, not needed when not calibrated
        fov_instructions = view_controller_2D.detect_head_orientation()

        # publish fov instructions
        publish_fov_instructions(fov_instructions)
        rospy.logdebug('Quaternion data: %s', str(fov_instructions))


def publish_instructions(instructions):
    """
    Publishes given flight instructions.
    :param instructions: flight instructions to publish
    """
    global publisher_instructions

    if instructions is None:
        return

    converted_instructions = list()
    for key in instructions:
        converted_instructions.append(InstructionMsg(instruction = key, intensity = instructions[key]))
    publisher_instructions.publish(InstructionsMsg(instructions = converted_instructions))


def publish_fov_instructions(fov_instructions):
    """
    Publishes given fov (field of view) instructions.
    :param fov_instructions: fov instructions to publish
    """
    global publisher_fov_instructions

    if fov_instructions is None:
        return

    publisher_fov_instructions.publish(FOVInstructionsMsg(w = fov_instructions[0], x = fov_instructions[1],
                                                          y = fov_instructions[2], z = fov_instructions[3]))


def reset(reset_msg):
    """
    Resets state of this ROS node.
    """
    global calibrator_2D

    rospy.logdebug('Received reset command. Resetting ...')
    calibrator_2D.reset_calibration()


if __name__ == '__main__':
    init_node()
