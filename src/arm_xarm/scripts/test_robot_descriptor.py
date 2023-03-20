#!/usr/bin/env python
import copy
import dataclasses
import enum
import pickle
import sys
import time
from typing import Any, Dict, Optional, Tuple
from std_msgs.msg import Bool
import mediapy as mp
# import Path
from pathlib import Path
import numpy as np
import skrobot
import torch
import torchvision.transforms as transforms
from termcolor import cprint
from sapien.core import Pose
import datetime
import json

sys.path.insert(0, "../")

import cv_bridge
import message_filters
import rospy
import tf
import tf.transformations as ttf
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, JointState

from maniskill2_learn.utils.torch import load_checkpoint
from maniskill2_learn.networks.utils import get_kwargs_from_shape, replace_placeholder_with_args
from maniskill2_learn.methods.builder import build_agent
from maniskill2_learn.utils.data import is_not_null, is_null, num_to_str
import configs.brl.bc.conv_mlp_robot as cfg
from maniskill2_learn.utils.meta.config import Config
from gym.spaces import Discrete, Box
import utilities as utils
import click

from einops import rearrange
import h5py

DEMO_FN = "/data/joso_0310_faucet_converted_dpose/traj_3.h5"
CKPT = '/home/xingyu/xarm/src/arm_xarm/ckpt/impala_half_dropout.ckpt'

class DescriptorAgent:
    def __init__(self, 
                 ckpt=None,
                 dry=False, 
                 traj=None,
                 wait_interp=True,
                 device="cuda",
                 video_save_dir=None,
                 num_steps=50,
                 ee_frame_control=False,
                 proprio=False):

        self.num_steps = num_steps
        self.wait_interp = wait_interp
        self.ckpt = ckpt
        self.device = device
        self.dry = dry
        self.ee_frame_control = ee_frame_control
        self.proprio = proprio

        if traj is not None:
            self.traj = h5py.File(traj, "r")
            self.traj_id = traj.split("/")[-1].split(".")[-2]
            self.traj = iter(self.traj["dict_str_" + self.traj_id]["dict_str_actions"])
            self.agent = None

        elif self.ckpt is not None:
            self.traj = None
            cfg = Config.fromfile("/home/xingyu/xarm/src/OpenWorldManipulation/ManiSkill2-Learn/configs/brl/bc/conv_mlp_robot.py")
            replaceable_kwargs = cfg.shape_cfg
            
            high_bound = np.ones(8)
            low_bound = -np.ones(8)
            low_bound[7] = 0
            cfg.agent_cfg['env_params']['action_space'] = Box(low=low_bound, high=high_bound, dtype=np.float32)
            cfg.agent_cfg["actor_cfg"]["nn_cfg"]['visual_nn_cfg']["visual_type"] = 'hand_rgb'
            
            cfg = replace_placeholder_with_args(cfg, **replaceable_kwargs)

            print(cfg.agent_cfg)
            self.agent = build_agent(cfg.agent_cfg)
            load_checkpoint(self.agent, ckpt)
            self.agent.to(device)
            self.agent.eval()

            rospy.loginfo("policy loaded")
        else:
            raise ValueError("No ckpt or traj provided")

        if video_save_dir is not None:
            self.video_save_dir = video_save_dir
        else:
            time = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
            self.video_save_dir = "/data/demo"
            if self.ckpt is not None:
                self.video_save_dir = f"{self.video_save_dir}_{self.ckpt.split('/')[-1].split('.')[0]}"
            elif self.traj is not None:
                self.video_save_dir = f"{self.video_save_dir}_{self.traj_id}"
            
            self.video_save_dir = f"{self.video_save_dir}_{time}_wait_{wait_interp}_dry_{dry}_steps_{num_steps}"
        
        self.video_save_dir = Path(self.video_save_dir)
        self.video_save_dir.mkdir(parents=True, exist_ok=True)
        self.vid = []

        # dump the variables into a json file in self.video_save_dir
        self.dump_dict = {}
        self.dump_dict["ckpt"] = ckpt
        self.dump_dict["dry"] = dry
        self.dump_dict["traj"] = traj
        self.dump_dict["wait_interp"] = wait_interp
        self.dump_dict["device"] = device
        self.dump_dict["video_save_dir"] = video_save_dir
        self.dump_dict["num_steps"] = num_steps

        with open(self.video_save_dir / "config.json", "w") as f:
            json.dump(self.dump_dict, f, indent=4)

        # policy setup
        rospy.init_node("descriptor_agent", anonymous=True)
        rospy.on_shutdown(self.save_vid)

        # robot setup
        self._gripper_action_prev = True  # true if closed, false if open
        self._robot = utils.XArm()
        self._ri = utils.XArmROSRobotInterface(self._robot, namespace="")
        self._ri.update_robot_state()
        self._robot.angle_vector(self._ri.angle_vector())

        self._tf_listener = tf.listener.TransformListener(cache_time=rospy.Duration(30))

        self._tf_listener.waitForTransform(
            target_frame="link_base",
            source_frame="wrist_camera_color_optical_frame",
            time=rospy.Time(0),
            timeout=rospy.Duration(10),
        )

        self._obs = None

        self._sub_caminfo_wrist = message_filters.Subscriber(
            "/wrist_camera/aligned_depth_to_color/camera_info", CameraInfo
        )
        self._sub_rgb_wrist = message_filters.Subscriber(
            "/wrist_camera/color/image_rect_color", Image
        )
        self._sub_depth_wrist = message_filters.Subscriber(
            "/wrist_camera/aligned_depth_to_color/image_raw", Image
        )

        # base camera
        self._sub_caminfo_base = message_filters.Subscriber(
            "/base_camera/aligned_depth_to_color/camera_info", CameraInfo
        )
        self._sub_rgb_base = message_filters.Subscriber(
            "/base_camera/color/image_rect_color", Image
        )
        self._sub_depth_base = message_filters.Subscriber(
            "/base_camera/aligned_depth_to_color/image_raw", Image
        )

        # proprio
        self._sub_joint = message_filters.Subscriber(
            "/joint_states", JointState
        )

        sync = message_filters.ApproximateTimeSynchronizer(
            [
                self._sub_caminfo_wrist,
                self._sub_rgb_wrist,
                self._sub_depth_wrist,
                self._sub_caminfo_base,
                self._sub_rgb_base,
                self._sub_depth_base,
                self._sub_joint
            ],
            slop=0.1,
            queue_size=50,
        )
        sync.registerCallback(self._obs_callback)

        sync = message_filters.ApproximateTimeSynchronizer(
            [
                self._sub_rgb_wrist,
                self._sub_rgb_base,
            ],
            slop=0.1,
            queue_size=50,
        )
        sync.registerCallback(self.record_video)

    def _obs_callback(
            self, 
            caminfo_msg_wrist, 
            rgb_msg_wrist, 
            depth_msg_wrist, 
            caminfo_msg_base,
            rgb_msg_base, 
            depth_msg_base, 
            joint_msg
            ):
        
        rospy.loginfo_once("obs callback registered")
        
        position, quaternion = self._tf_listener.lookupTransform(
            target_frame="link_base",
            source_frame="wrist_camera_color_optical_frame",
            time=rospy.Time(0),
        )
        T_camera_in_link0 = ttf.quaternion_matrix(quaternion)
        T_camera_in_link0[:3, 3] = position

        bridge = cv_bridge.CvBridge()

        # wrist camera processing
        rgb_wrist = bridge.imgmsg_to_cv2(rgb_msg_wrist, desired_encoding="rgb8")
        depth_wrist = bridge.imgmsg_to_cv2(depth_msg_wrist)
        assert rgb_wrist.dtype == np.uint8 and rgb_wrist.ndim == 3
        assert depth_wrist.dtype == np.uint16 and depth_wrist.ndim == 2
        depth_wrist = depth_wrist.astype(np.float32) / 1000

        K_wrist = np.array(caminfo_msg_wrist.K).reshape(3, 3)

        # base camera processing
        rgb_base = bridge.imgmsg_to_cv2(rgb_msg_base, desired_encoding="rgb8")
        depth_base = bridge.imgmsg_to_cv2(depth_msg_base)
        assert rgb_base.dtype == np.uint8 and rgb_base.ndim == 3
        assert depth_base.dtype == np.uint16 and depth_base.ndim == 2
        depth_base = depth_base.astype(np.float32) / 1000

        K_base = np.array(caminfo_msg_base.K).reshape(3, 3)
        joint_positions = np.array(joint_msg.position, dtype=np.float32)
        joint_velocites = np.array(joint_msg.velocity, dtype=np.float32)

        position, quaternion = self._tf_listener.lookupTransform(
            target_frame="link_base",
            source_frame="link_tcp",
            time=rospy.Time(0),
        )

        # rotate from xyzw to wxyz
        quaternion = np.array(quaternion)[[3, 0, 1, 2]]
        p_ee_in_link0 = np.concatenate([position, quaternion]) # wxyz quaternion

        self._obs = dict(
            stamp=caminfo_msg_wrist.header.stamp,
            rgb=rgb_wrist,
            depth=depth_wrist,
            state=p_ee_in_link0,
            camera_poses=T_camera_in_link0,
            K_matrices=K_wrist,
        )

        vid_frame = np.concatenate([rgb_wrist, rgb_base], axis=1)
        self.vid.append(vid_frame)

    def record_video(self, rgb_msg_wrist, rgb_msg_base):
        rospy.loginfo_once(f"recording video")
        bridge = cv_bridge.CvBridge()

        # wrist camera processing
        rgb_wrist = bridge.imgmsg_to_cv2(rgb_msg_wrist, desired_encoding="rgb8")
        rgb_base = bridge.imgmsg_to_cv2(rgb_msg_base, desired_encoding="rgb8")
        
        vid_frame = np.concatenate([rgb_wrist, rgb_base], axis=1)
        self.vid.append(vid_frame)

    def get_obs(
        self,
        stamp: rospy.Time,
        is_first: bool = False,
    ) -> Dict[str, Any]:
        s = time.time()
        while not self._obs:
            rospy.sleep(0.001)
        assert self._obs is not None
        s1  = time.time()
        s = time.time()
        obs_copy = self._obs.copy()
        
        s = time.time()
        # crop to square
        obs_copy["rgb"] = torch.as_tensor(obs_copy["rgb"]).unsqueeze(0)
        obs_copy["rgb"] = rearrange(obs_copy["rgb"], 'b h w c -> b c h w')

        obs_copy['camera_poses'] = torch.as_tensor(obs_copy['camera_poses']).unsqueeze(0)

        H, W = obs_copy['rgb'].shape[2:]
        sq_size = min(H, W)

        temp_rgb = obs_copy['rgb']
                
        temp_rgb = temp_rgb[:, :, :sq_size, :sq_size]
        temp_rgb = transforms.Resize((224, 224), 
            interpolation = transforms.InterpolationMode.BILINEAR)(temp_rgb)
        
        obs_copy['rgb'] = temp_rgb
        s = time.time()
        temp_depth = torch.as_tensor(obs_copy['depth']).unsqueeze(0).unsqueeze(0)
        
        temp_depth = temp_depth[:, :sq_size, :sq_size]
        temp_depth = transforms.Resize((224, 224), 
            interpolation = transforms.InterpolationMode.NEAREST)(temp_depth)
        obs_copy['depth'] = temp_depth

        obs_copy["state"] = torch.as_tensor(obs_copy["state"]).unsqueeze(0)

        obs_copy['K_matrices'] = torch.as_tensor(obs_copy['K_matrices']).unsqueeze(0)

        s = time.time()
        del obs_copy["stamp"]
        del obs_copy["depth"]
        return obs_copy

    def reset(self) -> Dict[str, Any]:
        self.steps = 0
        ret = self._reset()
        while utils.is_xarm_in_error():
            # print("Reset failed. Trying again.")
            ret = self._reset()
            rospy.sleep(0.1)
        return ret

    def _reset(self) -> Dict[str, Any]:
        utils.recover_xarm_from_error()
        rospy.sleep(0.2)
        if self.agent is not None:
            self.agent.reset()
        self._gripper_action_prev = 1  # open

        self._ri.ungrasp()
        rospy.sleep(1)

        self._robot.reset_pose()
        self._ri.angle_vector(self._robot.angle_vector(), time=5)
        self._ri.wait_interpolation()

        self.vid = []
        self._ri.grasp()
        obs = self.get_obs(rospy.Time.now(), is_first=True)
        return obs

    def step(self, action) -> Dict[str, Any]:
        # if utils.is_xarm_in_error():
        #     utils.recover_xarm_from_error()
        
        self.steps += 1
        if self.steps == self.num_steps:
            raise RuntimeError("Episode is done.")
        
        control = action["control"]
        gripper_action = action["gripper"]

        if gripper_action.item() != self._gripper_action_prev:
            if gripper_action == 0:
                self._ri.ungrasp()
            elif gripper_action == 1:
                self._ri.grasp()
            self._gripper_action_prev = gripper_action

        
        curr_obs = self.get_obs(rospy.Time.now(), is_first=False)
        p_ee_in_link0 = curr_obs["state"]
        T_ee_in_link0 = self.control_to_target_coords(control, p_ee_in_link0)
        target_coords = skrobot.coordinates.Coordinates(
            pos=T_ee_in_link0[:3, 3], rot=T_ee_in_link0[:3, :3]
        )
        av = self._robot.rarm.inverse_kinematics(target_coords)
        
        ik_fail = av is False
        if ik_fail:
            cprint("==> IK is failed", color="red")
            raise RuntimeError("IK failed. reset")

        if not (self.dry or ik_fail):
            if self.wait_interp:
                self._ri.angle_vector(self._robot.angle_vector(), time=.5)
                self._ri.wait_interpolation()
            else:
                self._ri.angle_vector(self._robot.angle_vector(), time=.5)
        return self.get_obs(rospy.Time.now(), is_first=False)

    def act(self, obs):
        if self.traj is not None:
            action = torch.as_tensor(next(self.traj)).unsqueeze(0)
        elif self.agent is not None:
            with torch.no_grad():
                # for no proprio
                if not self.proprio:
                    obs['state'] = torch.as_tensor(np.zeros(7)).unsqueeze(0)
                action = self.agent.act(obs)
        else:
            raise NotImplementedError
        
        act = {}
        act["control"] = action[:, :7]
        act["gripper"] = action[:, 7] > 0.5
        
        return act

    def control_to_target_coords(self, control, p_ee_in_link0):
        """
        control is delta position and delta quaternion.
        """
        p_ee_in_link0 = p_ee_in_link0.clone().to(control.device)
        p_ee_in_link0 = p_ee_in_link0.squeeze(0).cpu().numpy()
        control = control.squeeze(0).cpu().numpy()

        pos = p_ee_in_link0[:3]
        quat = p_ee_in_link0[3:]

        delta_pos = control[:3]
        delta_quat = control[3:]
        delta_quat = delta_quat / np.linalg.norm(delta_quat) # normalize quaternion to unit quaternion

        pose_ee = Pose(pos, quat)
        control_pose = Pose(delta_pos, delta_quat)

        if self.ee_frame_control:
            final_pose = pose_ee * control_pose
        else:
            final_pose = control_pose * pose_ee
            
        return final_pose.to_transformation_matrix()
    

    def save_vid(self):
        import datetime
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        vid_path = self.video_save_dir / f"{timestamp}.mp4"
        mp.write_video(vid_path, self.vid, fps=30)

        rospy.loginfo(f"Saved video to {vid_path}")
        self.vid = []

def main(args):
    print(args)
    agent = DescriptorAgent(**vars(args))
    r = rospy.Rate(5)
    control_pub = rospy.Publisher("/control/status", Bool, queue_size=1)
    while True:
        obs = agent.reset()
        while not rospy.is_shutdown():
            try:
                action = agent.act(obs)
                obs = agent.step(action)
                control_pub.publish(True)
                r.sleep()
            except RuntimeError as e:
                agent.save_vid()
                break
            
        # input("Press enter to continue...")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry", action="store_true")
    parser.add_argument("--ckpt", type=str, default=None)
    parser.add_argument("--traj", type=str, default=None)
    parser.add_argument("--wait_interp", action="store_true")
    parser.add_argument("--num_steps", type=int, default=100)
    parser.add_argument("--ee_frame_control", action="store_true")
    parser.add_argument("--proprio", action="store_true")
    args = parser.parse_args()
    main(args)
