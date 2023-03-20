#!/usr/bin/env python
import copy
import dataclasses
import enum
import pickle
import sys
import time
from typing import Any, Dict, Optional, Tuple

import dcargs
import numpy as np
import path
import skrobot
import torch
from termcolor import cprint

sys.path.insert(0, "../")
from bc_mae.deploy import get_model, normalize, resize_crop

sys.path.insert(0, "/home/stepjam/miniconda/envs/py37/lib/python3.7/site-packages")
import cv_bridge
import imgviz
import message_filters
import rospy
import tf
import tf.transformations as ttf
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, JointState

import utilities as utils

print("test")


class Space:
    """Copied from embodied"""

    def __init__(
        self,
        dtype: Any,
        shape: Tuple[int, ...] = (),
        low: Optional[float] = None,
        high: Optional[float] = None,
    ):
        # For integer types, high is the excluside upper bound.
        self.dtype = np.dtype(dtype)
        self.low = self._infer_low(dtype, shape, low, high)
        self.high = self._infer_high(dtype, shape, low, high)
        self.shape = self._infer_shape(dtype, shape, low, high)
        self.discrete = np.issubdtype(self.dtype, np.integer) or self.dtype == np.bool
        self._random = np.random.RandomState()

    def sample(self) -> np.ndarray:
        low, high = self.low, self.high
        if np.issubdtype(self.dtype, np.floating):
            low = np.maximum(np.ones(self.shape) * np.finfo(self.dtype).min, low)
            high = np.minimum(np.ones(self.shape) * np.finfo(self.dtype).max, high)
        return self._random.uniform(low, high, self.shape).astype(self.dtype)

    def _infer_low(
        self,
        dtype: Any,
        shape: Tuple[int, ...],
        low: Optional[float],
        high: Optional[float],
    ) -> np.ndarray:
        if low is None:
            if np.issubdtype(dtype, np.floating):
                low = -np.inf * np.ones(shape)
            elif np.issubdtype(dtype, np.integer):
                low = np.iinfo(dtype).min * np.ones(shape, dtype)
            elif np.issubdtype(dtype, np.bool):
                low = np.zeros(shape, np.bool)
            else:
                raise ValueError("Cannot infer low bound from shape and dtype.")
        return np.array(low)

    def _infer_high(
        self,
        dtype: Any,
        shape: Tuple[int, ...],
        low: Optional[float],
        high: Optional[float],
    ) -> np.ndarray:
        if high is None:
            if np.issubdtype(dtype, np.floating):
                high = np.inf * np.ones(shape)
            elif np.issubdtype(dtype, np.integer):
                high = np.iinfo(dtype).max * np.ones(shape, dtype)
            elif np.issubdtype(dtype, np.bool):
                high = np.ones(shape, np.bool)
            else:
                raise ValueError("Cannot infer high bound from shape and dtype.")
        return np.array(high)

    def _infer_shape(
        self,
        dtype: Any,
        shape: Tuple[int, ...],
        low: Optional[float],
        high: Optional[float],
    ) -> np.ndarray:
        if shape is None and low is not None:
            shape = low.shape
        if shape is None and high is not None:
            shape = high.shape
        if not hasattr(shape, "__len__"):
            shape = (shape,)
        assert all(dim and dim > 0 for dim in shape), shape
        return tuple(shape)


class ControlMode(enum.Enum):
    NONE = enum.auto()
    DELTA_XY = enum.auto()
    DELTA_XYZ = enum.auto()

    def control_shape(self) -> Tuple[int, ...]:
        if self == ControlMode.DELTA_XY:
            return (2,)
        if self == ControlMode.DELTA_XYZ:
            return (3,)
        else:
            raise NotImplementedError


@dataclasses.dataclass
class EnvConfig:
    control_mode: ControlMode = ControlMode.DELTA_XY
    max_delta_meters: float = 0.01


XYZ_MIN = np.array([0.25, -0.2, 0.018])
XYZ_MAX = np.array([0.45, 0.2, 0.350])
TABLE_Z = 0.0185


def control_to_target_coords(
    env_config: EnvConfig, control_action: np.ndarray, curr_pose: np.ndarray
) -> np.ndarray:
    """Convert control action to TCP homogeneous transform.

    Args:
        env_config (EnvConfig): The environment configuration.
        control_action (np.ndarray, shape=self.control_shape()): control_action
        (should be values between -1 and 1, following the dm_control convention)
        curr_pose (np.ndarray, shape=(4, 4)): the current robot pose

    Returns:
        np.ndarray, shape=(4, 4): The target pose.
    """
    target_pose = curr_pose.copy()
    control_action = np.clip(control_action, -1, 1) * env_config.max_delta_meters
    if env_config.control_mode == ControlMode.DELTA_XY:
        assert control_action.shape == (2,)
        target_pose[:2, 3] = target_pose[:2, 3] + control_action
        target_pose[2, 3] = TABLE_Z
    elif env_config.control_mode == ControlMode.DELTA_XYZ:
        target_pose = curr_pose.copy()
        assert control_action.shape == (3,)
        target_pose[:3, 3] = target_pose[:3, 3] + control_action
    else:
        raise NotImplementedError

    target_pose[:3, 3] = np.clip(target_pose[:3, 3], XYZ_MIN, XYZ_MAX)
    return target_pose


class BaseEnv:
    def __init__(self, cfg: EnvConfig):
        self.cfg = cfg
        self._gripper_action_prev = 1  # open
        self._robot = utils.XArm()
        self._ri = utils.XArmROSRobotInterface(self._robot, namespace="")
        self._ri.update_robot_state()
        self._robot.angle_vector(self._ri.angle_vector())

        self._robot2 = copy.deepcopy(self._robot)

        # self._viewer = skrobot.viewers.TrimeshSceneViewer()
        # self._viewer.add(self._robot)
        # self._viewer.show()

        self._tf_listener = tf.listener.TransformListener(cache_time=rospy.Duration(30))

        self._obs = None
        self._sub_caminfo = message_filters.Subscriber(
            "/camera/aligned_depth_to_color/camera_info", CameraInfo
        )
        self._sub_rgb = message_filters.Subscriber(
            "/camera/color/image_rect_color", Image
        )
        self._sub_depth = message_filters.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image
        )
        self._sub_joints = message_filters.Subscriber("/joint_states", JointState)
        sync = message_filters.ApproximateTimeSynchronizer(
            [self._sub_caminfo, self._sub_rgb, self._sub_depth, self._sub_joints],
            queue_size=50,
            slop=0.1,
        )
        sync.registerCallback(self._obs_callback)
        self.reset()

    def _fk(self, joint_positions: np.ndarray) -> np.ndarray:
        """Forward kinematics helper.

        Args:
            joint_positions (np.ndarray): current joints

        Returns:
            np.ndarray, shape=(4, 4): the pose
        """
        self._robot2.angle_vector(joint_positions)
        return self._robot2.rarm.end_coords.worldcoords().T()

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
        T_ee_in_link0 = self._fk(joint_positions)
        self._obs = dict(
            stamp=caminfo_msg.header.stamp,
            # K=K,
            rgb=rgb,
            # depth=depth,
            T_ee_in_link0=T_ee_in_link0,
            # T_camera_in_link0=T_camera_in_link0,
            # joint_positions=joint_positions,
            # joint_velocites=joint_velocites,
            # camera_to_base=camera_to_base,
            gripper_open=self._gripper_action_prev == 1,
        )

    @property
    def act_space(self) -> Dict[str, Space]:
        return {
            "robot_control": Space(np.int64, self.cfg.control_mode.control_shape()),
            "gripper_control": Space(bool),
        }

    @property
    def obs_space(self) -> Dict[str, Space]:
        return {
            "rgb": Space(np.uint8, (32, 32, 3)),  # TODO
            "T_ee_in_link0": Space(np.float32, (4, 4)),
            "gripper_open": Space(bool),
            "reward": Space(np.float32),
            "is_first": Space(bool),
        }

    def get_reward(self, curr_obs: Dict[str, Any]) -> float:
        raise NotImplementedError

    def get_obs(
        self,
        stamp: rospy.Time,
        robot_in_safe_state: bool,
        is_first: bool = False,
    ) -> Dict[str, Any]:
        # assert (not is_first) and (not is_last)
        while not self._obs or self._obs["stamp"] < stamp:
            rospy.sleep(0.01)
        assert self._obs is not None
        obs_copy = self._obs.copy()

        if robot_in_safe_state:
            obs_copy["reward"] = self.get_reward(obs_copy)
        else:
            obs_copy["reward"] = 0

        obs_copy["is_first"] = is_first
        return obs_copy

    def reset(self) -> Dict[str, Any]:
        ret = self._reset()
        while utils.is_xarm_in_error():
            print("Reset failed. Trying again.")
            ret = self._reset()
            rospy.sleep(0.1)
        return ret

    def _reset(self) -> Dict[str, Any]:
        utils.recover_xarm_from_error()
        rospy.sleep(0.2)
        self._gripper_action_prev = 1  # open

        self._ri.ungrasp()
        rospy.sleep(1)
        # self._robot.reset_pose()
        # av is in radians
        if self.cfg.control_mode is ControlMode.DELTA_XY:
            av = np.deg2rad([1.4, -2.4, -1.2, 23.5, 4.1, 26, -1.8])
        else:
            av = np.deg2rad([1.2, -23.4, -1.5, 32.6, 1.5, 56, -1.3])
        # av = np.array(av) + np.random.uniform(-0.025, 0.025)
        self._robot.rarm.angle_vector(av)

        self._ri.angle_vector(self._robot.angle_vector(), time=2)
        self._ri.wait_interpolation()

        obs = self.get_obs(rospy.Time.now(), robot_in_safe_state=True, is_first=True)
        print("Reset done!")
        return obs

    def step(self, action: Dict[str, Any]) -> Dict[str, Any]:
        if utils.is_xarm_in_error():
            utils.recover_xarm_from_error()

        av_prev = self._robot.angle_vector()
        print(av_prev)
        print(self._fk(av_prev))
        T_ee_in_link0 = control_to_target_coords(
            self.cfg, action["robot_control"], self._fk(av_prev)
        )

        target_coords = skrobot.coordinates.Coordinates(
            pos=T_ee_in_link0[:3, 3], rot=T_ee_in_link0[:3, :3]
        )
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

        robot_in_safe_state = not ik_fail and not in_collision
        if robot_in_safe_state:
            gripper_action = action["gripper_control"]
            self._ri.angle_vector(self._robot.angle_vector(), time=0.1)
            self._ri.wait_interpolation()
            rospy.sleep(0.5)
            err = utils.is_xarm_in_error()

            if not err and self._gripper_action_prev != gripper_action:
                # actually moving gripper
                if gripper_action == 1:
                    self._ri.ungrasp()
                else:
                    self._ri.grasp()
                rospy.sleep(1)
            self._gripper_action_prev = gripper_action

        # imgviz.io.cv_imshow(obs["rgb"])

        return self.get_obs(rospy.Time.now(), robot_in_safe_state, is_first=False)

    def render(self) -> np.ndarray:
        """TODO"""
        raise NotImplementedError

    def close(self) -> None:
        """Nothing to close."""


class KeyRewardEnv(BaseEnv):
    def get_reward(self, curr_obs: Dict[str, Any]) -> float:
        key = imgviz.io.cv_waitkey(1000)
        if key == -1:
            reward = 0
        else:
            reward = 1
        print(f"user_input: {key}, reward: {reward}")
        return reward


class TestUnsafeAgent:
    def __init__(self, env_config: EnvConfig):
        self._gripper_action = True
        self._cfg = env_config

    def act(self, obs: Dict[str, Any]) -> Dict[str, Any]:
        del obs  # not used

        assert self._cfg.control_mode in [ControlMode.DELTA_XY, ControlMode.DELTA_XYZ]
        control_action = np.array([1.0] * self._cfg.control_mode.control_shape()[0])
        # randomize direction of action
        control_action = (
            np.random.choice([-1, 1], control_action.shape) * control_action
        )
        self._gripper_action = not self._gripper_action
        return {
            "robot_control": control_action,
            "gripper_control": self._gripper_action,
        }


def main(env_config: EnvConfig):
    rospy.init_node("env_xarm")

    env = KeyRewardEnv(env_config)
    agent = TestUnsafeAgent(env_config)

    cprint(f"==> Running the environment with {agent}")
    cprint("==> Press any button to annotate reward=1, terminal=1")

    obs = env.reset()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            action = agent.act(obs)
        except IndexError:
            break
        t = time.time()
        obs = env.step(action)
        print("Step time :", time.time() - t, action)
        r.sleep()

    obs = env.reset()
    cprint("==> Finishing to run the environment")


if __name__ == "__main__":
    main(dcargs.parse(EnvConfig))
