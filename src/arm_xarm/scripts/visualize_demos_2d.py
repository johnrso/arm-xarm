#!/usr/bin/env python

import argparse
import pickle
import sys

import imgviz
import path
import time
import numpy as np
import mediapy

def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("saved_dir", type=path.Path, help="saved dir")
    parser.add_argument("index", type=int, help="index")

    args = parser.parse_args()

    # For cropping: 480, 640
    # xmin, xmax = -400, 640
    # ymin, ymax = 0, 400

    all_obs = []
    for obs_file in sorted(args.saved_dir.listdir()):
        with open(obs_file, "rb") as f:
            obs = pickle.load(f)
        viz = imgviz.tile(
            # [obs["rgb"][ymin:ymax, xmin:xmax], imgviz.depth2rgb(obs["depth"])[ymin:ymax, xmin:xmax]],
            [obs["rgb"], (obs["depth"] * 255).astype(np.uint8)], #, imgviz.depth2rgb(obs["depth"])],
            shape=(1, 2),
            border=(255, 255, 255),
        )

        all_obs.append(viz)
        imgviz.io.cv_imshow(viz)
        key = imgviz.io.cv_waitkey(1000 // (30 * 4))
        if key == ord("q"):
            break
        time.sleep(0.2)

    mediapy.write_video(f"/home/xingyu/xarm/src/arm_xarm/scripts/demos/{args.index}.mp4", all_obs, fps=10)

if __name__ == "__main__":
    main()
