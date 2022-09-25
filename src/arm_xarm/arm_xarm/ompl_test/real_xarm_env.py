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

from ompl_test import utils
from ompl_test.utils import MessageSubscriber
from ompl_test.xarm_ros_robot_interface import XArmROSRobotInterface, XArm
from ompl_test.xarm_sim_robot_interface import XArmSimRobotInterface

sys.path.insert(0, '/home/stepjam/miniconda/envs/py37/lib/python3.7/site-packages')
import cv_bridge
import imgviz


class RealXArmEnv(Env):

    def __init__(self, camera_resolution: tuple, episode_length: int):
        super(RealXArmEnv, self).__init__()
        self._episode_length = episode_length
        self._camera_resolution = camera_resolution
        self._gripper_action_prev = 0
        self._wait_for_reset = True
        self._step_i = 0
        self._in_error = False
        self._time_in_state = True

    def launch(self):

        # rospy.init_node("env_xarm")
        self._robot = XArm()
        self._robot.joint_list = self._robot.joint_list[:7]
        self._real_ri = XArmROSRobotInterface(self._robot)
        self._sim_ri = XArmSimRobotInterface(planner="RRTConnect")
        self.real_to_sim_state_update()

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

    # =========================
    def real_to_sim_state_update(self):
        self._real_ri.update_robot_state()
        self._sim_ri.setj(self._real_ri.potentio_vector())
        for attachment in self._sim_ri.attachments:
            attachment.assign()

    def visjs(self, js):
        for j in js:
            for _ in self._sim_ri.movej(j):
                pp.step_simulation()
                time.sleep(1 / 240)

    def movejs(
        self, js, time_scale=None, wait=True, retry=False, wait_callback=None
    ):
        print('movejs')
        if time_scale is None:
            time_scale = 3
        time_scale = 10
        js = np.asarray(js)

        self.real_to_sim_state_update()
        j_init = self._sim_ri.getj()

        self._real_ri.angle_vector_sequence(
            js, time_scale=time_scale,
            # max_pos_accel=1
        )
        if wait:
            success = self.wait_interpolation(callback=wait_callback)
            # success = self._real_ri.wait_interpolation()
            if success or not retry:
                return

            self.real_to_sim_state_update()
            j_curr = self._sim_ri.getj()

            js = np.r_[[j_init], js]

            for i in range(len(js) - 1):
                dj1 = js[i + 1] - j_curr
                dj2 = js[i + 1] - js[i]
                dj1[abs(dj1) < 0.01] = 0
                dj2[abs(dj2) < 0.01] = 0
                if (np.sign(dj1) == np.sign(dj2)).all():
                    break
            else:
                return
            self.movejs(
                js[i + 1 :], time_scale=time_scale, wait=wait, retry=False
            )

    def wait_interpolation(self, callback=None):
        self._sub_points.subscribe()
        controller_actions = self._real_ri.controller_table[self._real_ri.controller_type]
        while True:
            states = [action.get_state() for action in controller_actions]
            if all(s >= GoalStatus.SUCCEEDED for s in states) or self._in_error:
                break
            self.real_to_sim_state_update()
            if callback is not None:
                callback()
            rospy.sleep(0.01)
        self._sub_points.unsubscribe()
        if not all(s == GoalStatus.SUCCEEDED for s in states):
            rospy.logwarn("Some joint control requests have failed")
            return False
        return True

    def _reset_pose(self, *args, **kwargs):
        self.real_to_sim_state_update()
        curr_j = self._sim_ri.getj()
        if not np.allclose(curr_j, self._sim_ri.homej, rtol=0.01):
            curr_j[0] += 0.1
            self.movejs([self._sim_ri.homej], *args, **kwargs)
            self.movejs([self._sim_ri.homej], *args, **kwargs)
        else:
            print('all close')

    def _hide_robot_to_get_map(self, *args, **kwargs):
        hide = np.array(self._sim_ri.homej)
        hide[3] += np.pi * 0.4
        self.movejs([hide], *args, **kwargs)
        self.get_obs(rospy.Time.now())
        self._sim_ri.update_env(self._obs["depth"], self._obs["camera_to_base"], self._obs["K"])
        self.movejs([self._sim_ri.homej], *args, **kwargs)

    # =========================

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
        self._real_ri.ungrasp()
        rospy.sleep(1)

        if self._in_error:
            utils.recover_xarm_from_error()
            self._real_ri.ungrasp()
            rospy.sleep(1)

        self._reset_pose()
        self._gripper_action_prev = 1  # open
        # self._real_ri.ungrasp()

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

        av_prev = self._sim_ri.getj()
        av = self._robot.rarm.inverse_kinematics(target_coords)
        ik_fail = True
        if av is not None and isinstance(av, np.ndarray):
            try:
                state_valid = self._sim_ri.validatej(av)
            except Exception as e:
                print('Caugh Exception')
                print(e)
                state_valid = False
            print(state_valid)
            ik_fail = not state_valid
            av = av.tolist()

        # av_prev = self._sim_ri.getj()
        # av = self._sim_ri.solve_ik((act_result.action[:3], act_result.action[3:7]), validate=True)
        # print('2', av, type(av))
        # ik_fail = av is False or av is None
        # if ik_fail:
        #     cprint("==> IK is failed", color="red")

        # # collision check
        # _, pairs = self._robot.self_collision_check()
        # scene_object_names = [
        #     "table",
        # ]
        # in_collision = False
        # for name_a, name_b in pairs:
        #     if (name_a in scene_object_names) ^ (name_b in scene_object_names):
        #         in_collision = True
        # if in_collision:
        #     cprint("==> Robot may collide with scene", color="red")
        #     self._sim_ri.setj(av_prev)

        # if not ik_fail and not in_collision:
        if not ik_fail:

            # self._sim_ri.update_env(self._obs["depth"], self._obs["camera_to_base"], self._obs["K"])
            path = self._sim_ri.planj(av)
            if path is not None:
                self.movejs(path)
            else:
                print('PATH FAIL ==========')
                ik_fail = True
                self._sim_ri.setj(av_prev)

            # # self._ri.angle_vector(self._robot.angle_vector(), time=2)
            # self._real_ri.angle_vector(self._robot.angle_vector(), time=4)
            # self._real_ri.wait_interpolation()
            # rospy.sleep(0.5)

            if not self._in_error and self._gripper_action_prev != gripper_action and not ik_fail:
                # actually moving gripper
                if gripper_action == 1:
                    self._real_ri.ungrasp()
                else:
                    self._real_ri.grasp()
                rospy.sleep(1)
            self._gripper_action_prev = gripper_action
        else:
            self._sim_ri.setj(av_prev)

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

        self.real_to_sim_state_update()

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

    env = RealXArmEnv((400, 400), 20)
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

    cprint("==> Finishing to run the environment")


if __name__ == "__main__":
    main()
