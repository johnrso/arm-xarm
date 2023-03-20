#!/usr/bin/env python
import mediapy as mp
from pathlib import Path
import numpy as np
import datetime

import cv_bridge
import message_filters
import rospy

from sensor_msgs.msg import Image

class VidRecorder:
    def __init__(self, video_save_dir=None):
        if video_save_dir is not None:
            self.video_save_dir = video_save_dir
        else:
            time = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
            self.video_save_dir = "/data/demo"

        self.video_save_dir = Path(self.video_save_dir)
        self.video_save_dir.mkdir(parents=True, exist_ok=True)
        self.vid = []

        # policy setup
        rospy.init_node("vid_recorder", anonymous=True)
        rospy.on_shutdown(self.save_vid)

        self._sub_rgb_wrist = message_filters.Subscriber(
            "/wrist_camera/color/image_rect_color", Image
        )
        self._sub_rgb_base = message_filters.Subscriber(
            "/base_camera/color/image_rect_color", Image
        )

        sync = message_filters.ApproximateTimeSynchronizer(
            [
                self._sub_rgb_wrist,
                self._sub_rgb_base,
            ],
            slop=0.1,
            queue_size=50,
        )
        sync.registerCallback(self.record_video)

        rospy.spin()

    def record_video(self, rgb_msg_wrist, rgb_msg_base):
        rospy.loginfo_once(f"recording video")
        bridge = cv_bridge.CvBridge()

        # wrist camera processing
        rgb_wrist = bridge.imgmsg_to_cv2(rgb_msg_wrist, desired_encoding="rgb8")
        rgb_base = bridge.imgmsg_to_cv2(rgb_msg_base, desired_encoding="rgb8")
        
        vid_frame = np.concatenate([rgb_wrist, rgb_base], axis=1)
        self.vid.append(vid_frame) 

    def save_vid(self):
        import datetime
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        vid_path = self.video_save_dir / f"{timestamp}.mp4"
        mp.write_video(vid_path, self.vid, fps=30)

        rospy.loginfo(f"Saved video to {vid_path}")
        self.vid = []

def main(args):
    print(args)
    VidRecorder(**vars(args))

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--video_save_dir", type=str, default=None)
    args = parser.parse_args()
    main(args)
