#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import time
from std_msgs.msg import Bool
import message_filters
from sensor_msgs.msg import CameraInfo, JointState

import tf
import tf.transformations as ttf
import tf2_ros
import geometry_msgs.msg

import copy
import datetime
import pickle
from pathlib import Path
import utilities as utils
import shutil
from geometry_msgs.msg import TransformStamped

class ObsRecorder:
    def __init__(self, demo_dir: str = "/data/demo"):
        # demo recording
        rospy.init_node("obs_recorder", anonymous=True)
        # rospy.loginfo("obs_recorder node started")

        # check if the demo_dir exists. if not, create it
        self._demo_dir = Path(demo_dir)
        self._demo_dir.mkdir(parents=True, exist_ok=True)

        self._control_started_at = None
        self._control_ended_at = None
        self._recorded_dir = None
        self._robot = utils.XArm()
        self._gripper_state = None
        self._tf_listener = tf.listener.TransformListener(
            cache_time=rospy.Duration(30)
        )

        self.record_flag_sub = rospy.Subscriber("/record_demo", Bool, self.record_flag_callback)
        
        # wrist camera
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
            "/wrist_camera/aligned_depth_to_color/camera_info", CameraInfo
        )
        self._sub_rgb_base = message_filters.Subscriber(
            "/wrist_camera/color/image_rect_color", Image
        )
        self._sub_depth_base = message_filters.Subscriber(
            "/wrist_camera/aligned_depth_to_color/image_raw", Image
        )

        # proprio
        self._sub_joint = message_filters.Subscriber(
            "/joint_states", JointState
        )

        self._sub_control = message_filters.Subscriber(
            "/control/command", TransformStamped
        )

        self._sub_gripper = message_filters.Subscriber(
            "/control/gripper", Bool
        )

        self.reecording_pub = rospy.Publisher("/record_demo/status", Bool, queue_size=1)
        sync = message_filters.ApproximateTimeSynchronizer(
            [
                self._sub_caminfo_wrist,
                self._sub_rgb_wrist,
                self._sub_depth_wrist,
                self._sub_caminfo_base,
                self._sub_rgb_base,
                self._sub_depth_base,
                self._sub_joint,
                self._sub_control,
                self._sub_gripper,
            ],
            slop=0.1,
            queue_size=50,
            allow_headerless=True,
        )
        sync.registerCallback(self._demo_recording_callback)
        rospy.loginfo("obs_recorder: node started")
        while True:
            inp = input("d --> delete last recording\n")
            if inp == "d":
                demo_dirs = sorted(os.listdir(self._demo_dir))
                to_remove = self._demo_dir / demo_dirs[-1]

                # remove to_remove, and all files in it
                import shutil
                shutil.rmtree(to_remove, ignore_errors=True)
                print("Removed", to_remove)


    def _demo_recording_callback(
        self, 
        caminfo_msg_wrist, 
        rgb_msg_wrist, 
        depth_msg_wrist, 
        caminfo_msg_base,
        rgb_msg_base, 
        depth_msg_base, 
        joint_msg, 
        control_msg,
        gripper_msg
    ):
        rospy.loginfo_once("obs_recorder: messages synced")
        stamp = caminfo_msg_wrist.header.stamp
        if (
            self._control_started_at is None
            or self._recorded_dir is None
            or stamp < self._control_started_at
            or (
                self._control_ended_at is not None
                and self._control_ended_at > self._control_started_at
                and stamp > self._control_ended_at
            )
        ):
            return
        bridge = CvBridge()

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

        # proprio processing
        joint_positions = np.array(joint_msg.position)
        joint_velocites = np.array(joint_msg.velocity)

        # Need in pose form
        position, quaternion = self._tf_listener.lookupTransform(
            target_frame="link_base",
            source_frame="link_tcp",
            time=rospy.Time(0),
        )

        # quaternion is in xyzw form. convert to axis angle
        
        quaternion = np.array(quaternion)[[3, 0, 1, 2]]
        p_ee_in_link0 = np.concatenate([position, quaternion])
    
        # Notes:
        # T_cam_to_world = T_camera_in_link0: MUST go from camera to world frame transform
        # K_wrist matrix will be recorded in K_wrist form and then inverted during preprocessing by model
        position, quaternion = self._tf_listener.lookupTransform(
            target_frame="link_base",
            # source_frame="camera_link",
            source_frame="wrist_camera_color_optical_frame",
            time=rospy.Time(0),
        )

        T_camera_in_link0 = ttf.quaternion_matrix(quaternion)
        T_camera_in_link0[:3, 3] = position

        p, q = control_msg.transform.translation, control_msg.transform.rotation
        # convert quaternion to wxyz.
        position = [p.x, p.y, p.z]
        quaternion = [q.w, q.x, q.y, q.z]
        control = np.concatenate([position, quaternion])

        dt = datetime.datetime.fromtimestamp(stamp.to_sec())
        try:
            if self._recorded_dir and self._recorded_dir.exists():
                recorded_file = self._recorded_dir / (dt.isoformat() + ".pkl")
                with open(recorded_file, "wb") as f:
                    pickle.dump(
                        dict(
                            timestamp=dt.isoformat(),
                            rgb_wrist=rgb_wrist,
                            depth_wrist=depth_wrist,
                            K_wrist=K_wrist,
                            rgb_base=rgb_base,
                            depth_base=depth_base,
                            K_base=K_base,
                            p_ee_in_link0=p_ee_in_link0,
                            T_camera_in_link0=T_camera_in_link0,
                            joint_positions=joint_positions,
                            joint_velocites=joint_velocites,
                            gripper_state=gripper_msg.data, # true if closed, false if open
                            control=control,
                        ),
                        f
                    )

                rospy.loginfo_throttle(5, "obs_recorder: recorded to %s", recorded_file)
        except Exception as e:
            pass
            self.reecording_pub.publish(True)

    def record_flag_callback(self, msg):
        # get the number of demos in self._demo_dir
        if msg.data and self._control_started_at is None:
            rospy.loginfo("obs_recorder: recording started")
            self._recorded_dir = self._demo_dir / datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
            self._recorded_dir.mkdir(parents=True, exist_ok=True)
            self._control_started_at = rospy.Time.now()
            self._control_ended_at = None
        elif msg.data:
            # delete the current demo
            if self._recorded_dir is not None:
                shutil.rmtree(self._recorded_dir, ignore_errors=True)
            self._recorded_dir = None
            num_demos = len(os.listdir(self._demo_dir))
            self._control_ended_at = rospy.Time.now()
            self._control_started_at = None
            rospy.loginfo_once("obs_recorder: record_flag received")
            rospy.loginfo("obs_recorder: recording deleted: num demos = %d" % num_demos)
        elif not msg.data and self._control_started_at is not None:
            self._control_ended_at = rospy.Time.now()
            self._control_started_at = None
            self._recorded_dir = None
            num_demos = len(os.listdir(self._demo_dir))
            rospy.loginfo_once("obs_recorder: record_flag received")
            rospy.loginfo("obs_recorder: recording saved: num demos = %d" % num_demos)
        else:
            rospy.loginfo("obs_recorder: recording flag ignored")

    # on shutdown, check if recording a demo. If so, save the demo
    def shutdown(self):
        if self._control_started_at is not None:
            self._control_ended_at = rospy.Time.now()
            self._control_started_at = None
            rospy.loginfo("obs_recorder: recording ended")

if __name__ == "__main__":
    import argparse
    # parse for a demo recording directory:
    parser = argparse.ArgumentParser()
    parser.add_argument("--demo_dir", type=str, default="/data/new_demo")
    args = parser.parse_args()

    obs_recorder = ObsRecorder(args.demo_dir)