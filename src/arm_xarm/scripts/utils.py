import tempfile
import os
import subprocess

import actionlib
import control_msgs
import numpy as np
import openvr
import rospy
import skrobot
import tf.transformations as ttf
import xarm_gripper.msg
from skrobot.interfaces.ros.base import ROSRobotInterfaceBase
from skrobot.models.urdf import RobotModelFromURDF

WIDTH_MAX = 800
SPEED_MAX = 3000


class XArmROSRobotInterface(ROSRobotInterfaceBase):

    def __init__(self, *args, **kwargs):
        super(XArmROSRobotInterface, self).__init__(*args, **kwargs)

        self.gripper_move = actionlib.SimpleActionClient(
            '/xarm/gripper_move',
            xarm_gripper.msg.MoveAction)
        self.gripper_move.wait_for_server()

    @property
    def rarm_controller(self):
        return dict(
            controller_type='rarm_controller',
            controller_action='xarm/xarm7_traj_controller/follow_joint_trajectory',  # NOQA
            controller_state='xarm/xarm7_traj_controller/state',
            action_type=control_msgs.msg.FollowJointTrajectoryAction,
            joint_names=[j.name for j in self.robot.rarm.joint_list],
        )

    def default_controller(self):
        return [self.rarm_controller]

    def grasp(self, target_pulse=0, **kwargs):
        self.move_gripper(target_pulse=target_pulse, **kwargs)

    def ungrasp(self, **kwargs):
        self.move_gripper(target_pulse=WIDTH_MAX, **kwargs)

    def move_gripper(self, target_pulse, pulse_speed=SPEED_MAX, wait=True):
        goal = xarm_gripper.msg.MoveGoal(target_pulse=target_pulse, pulse_speed=pulse_speed)
        if wait:
            # self.gripper_move.send_goal_and_wait(goal)
            self.gripper_move.send_goal(goal)
            rospy.sleep(0.2)
        else:
            self.gripper_move.send_goal(goal)


class XArm(RobotModelFromURDF):
    def __init__(self, *args, **kwargs):
        urdf = rospy.get_param("/robot_description")
        tmp_file = tempfile.mktemp()
        with open(tmp_file, "w") as f:
            f.write(urdf)
        super().__init__(urdf_file=tmp_file)
        os.remove(tmp_file)
        self.reset_pose()

    @property
    def rarm(self):
        link_names = ["link{}".format(i) for i in range(1, 8)]
        links = [getattr(self, n) for n in link_names]
        joints = [link.joint for link in links]
        model = skrobot.model.RobotModel(link_list=links, joint_list=joints)
        model.end_coords = skrobot.coordinates.CascadedCoords(
            parent=self.link_tcp,
            name="rarm_end_coords",
        )
        return model

    # def pre_reset_pose(self):
    #     av = np.radians([
    #         180,
    #         50,
    #         -180,
    #         50,
    #         0,
    #         100,
    #         0,
    #     ])
    #     self.rarm.angle_vector(av)
    #     return self.angle_vector()
    #
    # def reset_pose(self):
    #     av = np.radians([
    #         180,
    #         50,
    #         -180,
    #         50,
    #         0,
    #         100,
    #         0,
    #     ])
    #     self.rarm.angle_vector(av)
    #     return self.angle_vector()

    def pre_reset_pose(self):
        av = [
            1.47, 0.42, -2.0, 1.75, 0.45, 1.9, -0.4
            # 1.85, 0.46, -1.74, 1.56, 0.42, 1.62, -1.37,
        ]
        self.rarm.angle_vector(av)
        return self.angle_vector()

    def reset_pose(self):
        av = [
            1.47, 0.42, -2.0, 1.75, 0.45, 1.9, -0.0
            #1.47, 0.42, -2.0, 1.75, 0.45, 1.9, -0.4
            # 1.85, 0.46, -1.74, 1.56, 0.42, 1.62, -1.37,
        ]
        av = np.array(av) + np.random.uniform(-0.025, 0.025)
        self.rarm.angle_vector(av)
        return self.angle_vector()



def matrix_from_openvr_pose(pose):
    if not isinstance(pose, openvr.HmdMatrix34_t):
        raise TypeError(f"Type of pose must be {openvr.HmdMatrix34_t}")

    matrix = np.array(
        [
            [pose[0][0], pose[0][1], pose[0][2], pose[0][3]],
            [pose[1][0], pose[1][1], pose[1][2], pose[1][3]],
            [pose[2][0], pose[2][1], pose[2][2], pose[2][3]],
            [0, 0, 0, 1],
        ],
        dtype=np.float64,
    )
    # # Map x->-x, y->z, z->y
    matrix = (
            ttf.rotation_matrix(np.deg2rad(180), [0, 1, 0])
            @ ttf.rotation_matrix(np.deg2rad(-90), [1, 0, 0])
            @ matrix
    )

    return matrix


def scale_transformation(transformation_matrix, scale):
    translation = ttf.translation_from_matrix(transformation_matrix)
    ai, aj, ak = ttf.euler_from_matrix(transformation_matrix)
    transformation_matrix = ttf.euler_matrix(
        ai * scale, aj * scale, ak * scale
    )
    transformation_matrix[:3, 3] = translation * scale
    return transformation_matrix


def dict_from_controller_state(pControllerState):
    # https://github.com/ValveSoftware/openvr/wiki/IVRSystem::GetControllerState
    d = {}
    d["unPacketNum"] = pControllerState.unPacketNum
    # on trigger .y is always 0.0 says the docs
    d["trigger"] = pControllerState.rAxis[1].x
    # 0.0 on trigger is fully released
    # -1.0 to 1.0 on joystick and trackpads
    d["trackpad_x"] = pControllerState.rAxis[0].x
    d["trackpad_y"] = pControllerState.rAxis[0].y
    # These are published and always 0.0
    # for i in range(2, 5):
    #     d['unknowns_' + str(i) + '_x'] = pControllerState.rAxis[i].x
    #     d['unknowns_' + str(i) + '_y'] = pControllerState.rAxis[i].y
    d["ulButtonPressed"] = pControllerState.ulButtonPressed
    d["ulButtonTouched"] = pControllerState.ulButtonTouched
    # To make easier to understand what is going on
    # Second bit marks menu button
    d["menu_button"] = bool(pControllerState.ulButtonPressed >> 1 & 1)
    # 32 bit marks trackpad
    d["trackpad_pressed"] = bool(pControllerState.ulButtonPressed >> 32 & 1)
    d["trackpad_touched"] = bool(pControllerState.ulButtonTouched >> 32 & 1)
    # third bit marks grip button
    d["grip_button"] = bool(pControllerState.ulButtonPressed >> 2 & 1)
    # System button can't be read, if you press it
    # the controllers stop reporting

    trackpad_pressed_button = "none"
    if d["trackpad_pressed"]:
        x = d["trackpad_x"]
        y = d["trackpad_y"]
        assert -1 <= x <= 1
        assert -1 <= y <= 1
        if -0.5 <= x <= 0.5 and -1 <= y <= -0.5:
            trackpad_pressed_button = "bottom"
        elif -0.5 <= x <= 0.5 and 0.5 <= y <= 1:
            trackpad_pressed_button = "top"
        elif -1 <= x <= -0.5 and -0.5 <= y <= 0.5:
            trackpad_pressed_button = "left"
        elif 0.5 <= x <= 1 and -0.5 <= y <= 0.5:
            trackpad_pressed_button = "right"
        elif -0.5 <= x <= 0.5 and -0.5 <= y <= 0.5:
            trackpad_pressed_button = "center"
    d["trackpad_pressed_button"] = trackpad_pressed_button

    return d


def pointcloud_from_depth(
    depth: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> np.ndarray:
    assert depth.dtype.kind == "f", "depth must be float and have meter values"

    rows, cols = depth.shape
    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    valid = ~np.isnan(depth)
    z = np.where(valid, depth, np.nan)
    x = np.where(valid, z * (c - cx) / fx, np.nan)
    y = np.where(valid, z * (r - cy) / fy, np.nan)
    pc = np.dstack((x, y, z))

    return pc


def recover_xarm_from_error():
    # subprocess.call(["rosservice", "call", "/xarm/clear_err"],
    #                 stdout=open(os.devnull, "w"), stderr=subprocess.STDOUT)
    # subprocess.call(["rosservice", "call", "/xarm/set_mode", "1"],
    #                 stdout=open(os.devnull, "w"), stderr=subprocess.STDOUT)
    subprocess.call(["rosservice", "call", "/xarm/moveit_clear_err"],
                    stdout=open(os.devnull, "w"), stderr=subprocess.STDOUT)


def is_xarm_in_error():
    res = subprocess.Popen(["rostopic", "echo", "-n", "1", "/xarm/xarm_states"],
                           stdout=subprocess.PIPE).communicate()[0]
    res = res.decode("utf-8")
    return 'err: 0' not in res
