#!/usr/bin/env python

import copy
import pickle
import sys
import time
from typing import Any, List

import message_filters
import numpy as np
import path
import pybullet_planning as pp
import rospy
import skrobot
import tf
import tf.transformations as ttf
from actionlib_msgs.msg import GoalStatus
from rlbench.backend.observation import Observation
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from termcolor import cprint
from xarm_msgs.msg import RobotMsg
from yarr.agents.agent import ActResult
from yarr.envs.env import Env
from yarr.utils.observation_type import ObservationElement
from yarr.utils.transition import Transition

from scripts import utils as sutils
from ompl_test import utils
from ompl_test.utils import MessageSubscriber
from ompl_test.xarm_ros_robot_interface import XArmROSRobotInterface, XArm
from ompl_test.xarm_sim_robot_interface import XArmSimRobotInterface

sys.path.insert(0, '/home/stepjam/miniconda/envs/py37/lib/python3.7/site-packages')
import cv_bridge
import imgviz

TS = 12


class RealXArmEnv2(Env):

    def __init__(self, camera_resolution: tuple, episode_length: int):
        super(RealXArmEnv2, self).__init__()
        self._episode_length = episode_length
        self._camera_resolution = camera_resolution
        self._gripper_action_prev = 0
        self._wait_for_reset = True
        self._step_i = 0
        self._in_error = False
        self._time_in_state = True

    def launch(self):

        self._gripper_action_prev = 1  # open
        self._robot = sutils.XArm()
        self._ri = sutils.XArmROSRobotInterface(self._robot, namespace='')
        self._ri.update_robot_state()
        self._robot.angle_vector(self._ri.angle_vector())
        self._robot2 = copy.deepcopy(self._robot)

        self._tf_listener = tf.listener.TransformListener(
            cache_time=rospy.Duration(30)
        )

        self._obs = None
        self._sub_points = MessageSubscriber(
            [
                # ("/camera/aligned_depth_to_color/camera_info", CameraInfo),
                ("/camera/color/camera_info", CameraInfo),
                ("/camera/color/image_rect_color", Image),
                ("/camera/aligned_depth_to_color/image_raw", Image),
                ("/joint_states", JointState),
            ],
            callback=self._obs_callback,
        )
        self._sub_points.subscribe()

        self._sub_error = MessageSubscriber(
            [
                ("/xarm/xarm_states", RobotMsg)
            ],
            callback=self._xarm_state_callback,
        )
        self._sub_error.subscribe()

        self.reset()

    def _xarm_state_callback(
        self, state_msg: RobotMsg
    ):
        self._in_error = state_msg.err != 0

    def _obs_callback(self, caminfo_msg, rgb_msg, depth_msg, joint_msg):
        self._tf_listener.waitForTransform(
            target_frame="link_base",
            source_frame="camera_color_optical_frame",
            time=rospy.Time(0),
            timeout=rospy.Duration(0),
        )
        position, quaternion = self._tf_listener.lookupTransform(
            target_frame="link_base",
            source_frame="camera_color_optical_frame",
            time=rospy.Time(0),
        )
        camera_to_base = position, quaternion
        T_camera_in_link0 = ttf.quaternion_matrix(quaternion)
        T_camera_in_link0[:3, 3] = position

        bridge = cv_bridge.CvBridge()
        rgb = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
        depth = bridge.imgmsg_to_cv2(depth_msg)
        assert rgb.dtype == np.uint8 and rgb.ndim == 3
        assert depth.dtype == np.uint16 and depth.ndim == 2
        depth = depth.astype(np.float32) / 1000
        depth[depth == 0] = np.nan
        
        K = np.array(caminfo_msg.K).reshape(3, 3)
        joint_positions = np.array(joint_msg.position, dtype=np.float32)
        joint_velocites = np.array(joint_msg.velocity, dtype=np.float32)

        self._robot2.angle_vector(joint_positions)
        T_ee_in_link0 = self._robot2.rarm.end_coords.worldcoords().T()

        self._obs = dict(
            stamp=caminfo_msg.header.stamp,
            K=K,
            rgb=rgb,
            depth=depth,
            T_ee_in_link0=T_ee_in_link0,
            T_camera_in_link0=T_camera_in_link0,
            joint_positions=joint_positions,
            joint_velocites=joint_velocites,
            camera_to_base=camera_to_base,
            gripper_open=self._gripper_action_prev == 1
        )

    def get_obs(self, stamp) -> Observation:
        while not self._obs or self._obs["stamp"] < stamp:
            rospy.sleep(0.01)
        return utils.real_obs_to_rlbench_obs(self._obs)

    def reset(self) -> dict:
        self._reset()
        while self._in_error:
            print('Reset failed. Trying again.')
            self._reset()
            rospy.sleep(0.1)

        # Wait util ready to start next episode (i.e. reset scene)
        if self._wait_for_reset:
            obs = self.get_obs(rospy.Time.now())
            print('\n\n\nPress Key to reset episode.\n\n\n')
            imgviz.io.cv_imshow(obs.front_rgb)
            imgviz.io.cv_waitkey(-1)
        else:
            print('Not waiting for reset.')

        # if self._wait_for_reset:
        #     self._hide_robot_to_get_map()

        self._wait_for_reset = False
        obs = self.get_obs(rospy.Time.now())
        self._step_i = 0
        return self.extract_obs(obs)

    def _reset(self):
        if self._in_error:
            utils.recover_xarm_from_error()
            rospy.sleep(0.2)

        self._ri.ungrasp()
        rospy.sleep(1)
        self._robot.reset_pose()
        self._ri.angle_vector(self._robot.angle_vector(), time_scale=TS)
        self._ri.wait_interpolation()

        if self._in_error:
            utils.recover_xarm_from_error()
            self._ri.ungrasp()
            rospy.sleep(1)

        self._gripper_action_prev = 1  # open

    def step(self, act_result: ActResult) -> Transition:
        if self._in_error:
            utils.recover_xarm_from_error()

        if act_result.action[0] < 0.1:
            obs = self.get_obs(rospy.Time.now())
            print('Auto fail because x < 0.1')
            self._wait_for_reset = self._step_i != 0
            return Transition(self.extract_obs(obs), -1, True)

        r = R.from_quat(act_result.action[3:7])
        gripper_action = act_result.action[-1]

        target_coords = skrobot.coordinates.Coordinates(
            pos=act_result.action[:3], rot=r.as_matrix()
        )

        av_prev = self._robot.angle_vector()
        av = self._robot.rarm.inverse_kinematics(target_coords)

        # self._viewer.add(skrobot.model.Axis.from_coords(target_coords))

        ik_fail = av is False
        if ik_fail:
            cprint("==> IK is failed", color="red")

        # collision check
        _, pairs = self._robot.self_collision_check()
        scene_object_names = [
            "table",
        ]
        in_collision = False
        for name_a, name_b in pairs:
            if (name_a in scene_object_names) ^ (name_b in scene_object_names):
                in_collision = True
        if in_collision:
            cprint("==> Robot may collide with scene", color="red")
            self._robot.angle_vector(av_prev)

        # self._viewer.redraw()

        if not ik_fail and not in_collision:
            self._ri.angle_vector(self._robot.angle_vector(), time_scale=TS)
            self._ri.wait_interpolation()
            rospy.sleep(0.5)

            # if self._gripper_action_prev != gripper_action:
            #     # actually moving gripper
            #     if gripper_action == 1:
            #         self._ri.ungrasp()
            #     else:
            #         self._ri.grasp()
            #     rospy.sleep(1)
            self._gripper_action_prev = gripper_action

        obs = self.get_obs(rospy.Time.now())
        ik_fail |= self._in_error

        key = None
        self._wait_for_reset = True
        if not ik_fail:
            # user_input
            #     nothing: reward=0, terminal=0
            #   something: reward=1, terminal=1
            imgviz.io.cv_imshow(obs.front_rgb)
            print('Press Key to give reward..')
            key = imgviz.io.cv_waitkey(3000)
            if key == -1:
                reward = 0
                terminal = False
            else:
                reward = 100
                terminal = True
        else:
            # ignore user_input
            reward = 0
            terminal = True
            self._wait_for_reset = self._step_i != 0

        print(f"user_input: {key}, reward: {reward}, terminal: {terminal}")
        self._step_i += 1

        if self._in_error:
            utils.recover_xarm_from_error()

        return Transition(self.extract_obs(obs), reward, terminal)

    @property
    def observation_elements(self) -> List[ObservationElement]:
        self.low_dim_state_len = 8 + 1  # time in state
        return [
            ObservationElement('front_rgb', (3,) + self._camera_resolution, np.uint8),
            ObservationElement('front_point_cloud', (3,) + self._camera_resolution, np.float32),
            ObservationElement('low_dim_state', (self.low_dim_state_len,), np.float32),
            ObservationElement('front_camera_extrinsics', (4, 4), np.float32),
            ObservationElement('front_camera_intrinsics', (3, 3), np.float32)
        ]

    def extract_obs(self, obs: Observation, t=None, prev_action=None):
        # Turn gripper quaternion to be positive w
        if obs.gripper_pose is not None and obs.gripper_pose[-1] < 0:
            obs.gripper_pose[3:] *= -1.0

        xmin, xmax = -400, 640
        ymin, ymax = 0, 400
        pc = np.transpose(obs.front_point_cloud[ymin:ymax, xmin:xmax], [2, 0, 1])
        valid = ~np.isnan(pc)
        pc = np.where(valid, pc, np.full_like(pc, -2)).astype(np.float32)

        obs_dict = {
            'low_dim_state': np.concatenate([obs.gripper_pose, [float(obs.gripper_open)]]).astype(np.float32),
            'front_rgb': np.transpose(obs.front_rgb[ymin:ymax, xmin:xmax], [2, 0, 1]),
            'front_point_cloud': pc,
        }

        for k, v in obs.misc.items():
            if 'extrinsics' in k or 'intrinsics' in k:
                obs_dict[k] = v.astype(np.float32)

        if self._time_in_state:
            time = (1. - ((self._step_i if t is None else t) / float(
                self._episode_length - 1))) * 2. - 1.
            obs_dict['low_dim_state'] = np.concatenate(
                [obs_dict['low_dim_state'], [time]]).astype(np.float32)

        return obs_dict

    @property
    def action_shape(self) -> tuple:
        return (8,)

    @property
    def env(self) -> Any:
        raise NotImplementedError()


class TestUnsafeAgent:
    def __init__(self):
        self._robot = XArm()
        self._ri = XArmROSRobotInterface(self._robot)
        self._ri.update_robot_state()
        self._robot.angle_vector(self._ri.potentio_vector())
        self._i = 0
        self._ee_coords = self._robot.rarm.end_coords.copy_worldcoords()

    def act(self, obs):
        del obs  # not used

        if self._i == 0:
            self._ee_coords.translate([0.0, 0, -0.2], wrt="world")
        else:
            self._ee_coords.translate([0, 0, -0.02], wrt="world")
        #if self._i == 0:
        #    self._ee_coords.translate([0.0, 0.4, -0.2], wrt="world")
        #elif self._i == 1:
        #    self._ee_coords.translate([0, -0.8, 0], wrt="world")
        #else:
        #    pass

        T_ee_in_link0 = self._ee_coords.T()
        gripper_action = 0
        self._i += 1
        # target_coords = skrobot.coordinates.Coordinates(
        #     pos=T_ee_in_link0[:3, 3], rot=T_ee_in_link0[:3, :3]
        # )
        act = np.concatenate([T_ee_in_link0[:3, 3],
                              np.array(R.from_matrix(T_ee_in_link0[:3, :3]).as_quat()),
                              [gripper_action]])
        print(act)
        return ActResult(act)


class TestAgent:

    saved_dir = path.Path(
        # "/home/stepjam/.ros/arm_xarm/filing_cabinet/2022-01-20T11:39:44.956325"
        # "/home/stepjam/.ros/arm_xarm/demo_saved/2022-02-17T09:11:45.982058"
        "/home/stepjam/.ros/arm_xarm/demo_saved/2022-02-17T09:16:51.075019"
    )

    def __init__(self):
        self._i = 0

        self._pkl_files = []
        pkl_files = sorted(self.saved_dir.listdir())
        for i in range(0, len(pkl_files), len(pkl_files) // 10):
            self._pkl_files.append(pkl_files[i])

    def act(self, obs):
        del obs  # not used

        pkl_file = self._pkl_files[self._i]
        with open(pkl_file, "rb") as f:
            data = pickle.load(f)

        self._i += 1

        T_ee_in_link0 = data["T_ee_in_link0"]
        # joint_positions_gripper = data["joint_positions"][-2:]
        gripper_open = data["gripper_open"]
        if gripper_open:
            gripper_action = 1  # open
        else:
            gripper_action = 0  # close

        # target_coords = skrobot.coordinates.Coordinates(
        #     pos=T_ee_in_link0[:3, 3], rot=T_ee_in_link0[:3, :3]
        # )
        act = np.concatenate([T_ee_in_link0[:3, 3], np.array(R.from_matrix(T_ee_in_link0[:3, :3]).as_quat()), [gripper_action]])
        return ActResult(act)

def main():
    rospy.init_node("env_xarm")

    env = RealXArmEnv2((400, 400), 20)
    env.launch()

    agent = TestAgent()
    # agent = TestUnsafeAgent()

    cprint(f"==> Running the environment with {agent}")
    cprint("==> Press any button to annotate reward=1, terminal=1")

    obs = env.reset()
    while True:
        try:
            action = agent.act(obs)
        except IndexError:
            break
        transition = env.step(action)
        if transition.terminal:
            break
    env.reset()

    cprint("==> Finishing to run the environment")


if __name__ == "__main__":
    main()
