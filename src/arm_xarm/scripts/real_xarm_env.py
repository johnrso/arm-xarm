#!/usr/bin/env python

import pickle

import numpy as np
import path
import skrobot
from termcolor import cprint

import sys
sys.path.insert(0, '/home/stepjam/miniconda/envs/py37/lib/python3.7/site-packages')
import cv_bridge
import imgviz
import message_filters
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import tf
import tf.transformations as ttf

import utilities as utils


class Env:
    def __init__(self):
        # import ipdb; ipdb.set_trace()
        self._gripper_action_prev = 1  # open
        self._robot = utils.XArm()
        self._ri = utils.XArmROSRobotInterface(self._robot, namespace='')
        self._ri.update_robot_state()
        self._robot.angle_vector(self._ri.angle_vector())

        # self._viewer = skrobot.viewers.TrimeshSceneViewer()
        # self._viewer.add(self._robot)
        # self._viewer.show()

        self._tf_listener = tf.listener.TransformListener(
            cache_time=rospy.Duration(30)
        )

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
        sync = message_filters.TimeSynchronizer(
            [self._sub_caminfo, self._sub_rgb, self._sub_depth],
            queue_size=50,
        )
        sync.registerCallback(self._obs_callback)

        self.reset()

    def _obs_callback(self, caminfo_msg, rgb_msg, depth_msg):
        self._tf_listener.waitForTransform(
            target_frame="link_base",
            source_frame="camera_color_optical_frame",
            time=rospy.Time(0),
            timeout=rospy.Duration(1),
        )
        position, quaternion = self._tf_listener.lookupTransform(
            target_frame="link_base",
            source_frame="camera_color_optical_frame",
            time=rospy.Time(0),
        )
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

        self._obs = dict(
            stamp=caminfo_msg.header.stamp,
            K=K,
            rgb=rgb,
            depth=depth,
            T_camera_in_link0=T_camera_in_link0,
            gripper_open=self._gripper_action_prev == 1,
        )

    def get_obs(self, stamp):
        while not self._obs or self._obs["stamp"] < stamp:
            rospy.sleep(0.01)
        return self._obs

    def reset(self):
        ret = self._reset()
        while utils.is_xarm_in_error():
            print('Reset failed. Trying again.')
            ret = self._reset()
            rospy.sleep(0.1)
        return ret

    def _reset(self):
        utils.recover_xarm_from_error()
        rospy.sleep(0.2)
        self._gripper_action_prev = 1  # open

        self._ri.ungrasp()
        # rospy.sleep(1)
        self._robot.reset_pose()

        self._ri.angle_vector(self._robot.angle_vector(), time=2)
        self._ri.wait_interpolation()

        obs = self.get_obs(rospy.Time.now())
        reward = 0
        terminal = False
        print('Reeset donee!')
        return obs, reward, terminal

    def step(self, action):
        utils.recover_xarm_from_error()

        T_ee_in_link0, gripper_action = action

        target_coords = skrobot.coordinates.Coordinates(
            pos=T_ee_in_link0[:3, 3], rot=T_ee_in_link0[:3, :3]
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
            self._ri.angle_vector(self._robot.angle_vector(), time=.5)
            self._ri.wait_interpolation()
            # rospy.sleep(0.5)
            err = utils.is_xarm_in_error()

            if not err and self._gripper_action_prev != gripper_action:
                # actually moving gripper
                if gripper_action == 1:
                    self._ri.ungrasp()
                else:
                    self._ri.grasp()
                # rospy.sleep(1)
            self._gripper_action_prev = gripper_action

        obs = self.get_obs(rospy.Time.now())

        # imgviz.io.cv_imshow(obs["rgb"])
        # key = imgviz.io.cv_waitkey(1000)

        # ik_fail |= utils.is_xarm_in_error()

        # if not ik_fail and not in_collision:
        #     # user_input
        #     #     nothing: reward=0, terminal=0
        #     #   something: reward=1, terminal=1
        #     if key == -1:
        #         reward = 0
        #         terminal = 0
        #     else:
        #         reward = 1
        #         terminal = 1
        # else:
        #     # ignore user_input
        #     reward = -1
        #     terminal = 1
        # print(f"user_input: {key}, reward: {reward}, terminal: {terminal}")

        reward = terminal = 0

        return obs, reward, terminal


class TestAgent:

    saved_dir = path.Path(
        # "/home/stepjam/.ros/arm_xarm/demo_saved/2022-01-18T16:02:40.928052"
        "/home/stepjam/.ros/arm_xarm/demo_saved/2022-02-17T09:16:51.075019"
    )

    def __init__(self, saved_dir=None):
        self._i = 0
        self.saved_dir = path.Path(saved_dir) if saved_dir else self.saved_dir
        self._pkl_files = []
        pkl_files = sorted(self.saved_dir.listdir())
        for i in range(0, len(pkl_files), len(pkl_files) // 20):
            self._pkl_files.append(pkl_files[i])

    def act(self, obs):
        del obs  # not used

        pkl_file = self._pkl_files[self._i]
        with open(pkl_file, "rb") as f:
            data = pickle.load(f)

        self._i += 1

        # import ipdb; ipdb.set_trace()

        p_ee_in_link0 = data["p_ee_in_link0"]
        T_ee_in_link0 = ttf.quaternion_matrix(p_ee_in_link0[3:])
        T_ee_in_link0[:3, 3] = p_ee_in_link0[:3]

        # joint_positions_gripper = data["joint_positions"][-2:]
        gripper_open = data["gripper_open"]
        if gripper_open:
            gripper_action = 1  # open
        else:
            gripper_action = 0  # close

        return T_ee_in_link0, gripper_action


class TestUnsafeAgent:
    def __init__(self):
        self._robot = utils.XArm()
        self._ri = utils.XArmROSRobotInterface(self._robot, namespace='')
        self._ri.update_robot_state()
        # self._robot.angle_vector(self._ri.angle_vector())
        self._robot.angle_vector(self._ri.potentio_vector())

        self._i = 0
        self._ee_coords = self._robot.rarm.end_coords.copy_worldcoords()

    def act(self, obs):
        del obs  # not used

        if self._i == 0:
            self._ee_coords.translate([0.0, 0, -0.2], wrt="world")
        else:
            self._ee_coords.translate([0, 0, -0.02], wrt="world")

        T_ee_in_link0 = self._ee_coords.T()
        gripper_action = 0

        self._i += 1

        return T_ee_in_link0, gripper_action


def main(saved_dir=None):
    rospy.init_node("env_xarm")

    env = Env()
    agent = TestAgent(saved_dir=saved_dir)
    # agent = TestUnsafeAgent()

    cprint(f"==> Running the environment with {agent}")
    cprint("==> Press any button to annotate reward=1, terminal=1")

    obs = env.reset()
    while True:
        try:
            print("acting...")
            action = agent.act(obs)
        except IndexError:
            break
        obs, reward, terminal = env.step(action)
        if terminal:
            break

    obs = env.reset()

    cprint("==> Finishing to run the environment")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--saved_dir", type=str, default="/data/point_reach/")

    args = parser.parse_args()
    main(args.saved_dir)
