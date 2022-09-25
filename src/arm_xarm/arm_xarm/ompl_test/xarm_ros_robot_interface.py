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
import path


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
        # here = path.Path(__file__).abspath().parent
        # urdf_file = here + '/urdf/xarm7.urdf'
        # super().__init__(urdf_file=urdf_file)
        # self.reset_pose()
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

    def pre_reset_pose(self):
        av = np.radians([
            180,
            0,
            -180,
            100,
            0,
            100,
            0,
        ])
        self.rarm.angle_vector(av)
        return self.angle_vector()

    def reset_pose(self):
        av = np.radians([
            180,
            0,
            -180,
            100,
            0,
            100,
            0,
        ])
        self.rarm.angle_vector(av)
        return self.angle_vector()
