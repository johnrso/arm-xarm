#!/usr/bin/env python

import argparse
import pickle
import sys

import numpy as np

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import imgviz
import path
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("saved_dir", type=path.Path, help="saved dir")
    args = parser.parse_args()

    # 480, 640
    xmin, xmax = -400, 640
    ymin, ymax = 0, 400

    sorted_dir = sorted(args.saved_dir.listdir())
    idxs = np.linspace(0, len(sorted_dir) - 1, 4)
    idxs[2] += 10
    sorted_dir = [sorted_dir[int(i)] for i in idxs]

    frames = []
    for obs_file in sorted_dir:
        with open(obs_file, "rb") as f:
            obs = pickle.load(f)
            print(list(obs.keys()))
        viz = imgviz.tile(
            [obs["rgb"][ymin:ymax, xmin:xmax], imgviz.depth2rgb(obs["depth"])[ymin:ymax, xmin:xmax]],
            # shape=(1, 2),
            shape=(2, 1),
            border=(255, 255, 255),
            border_width=2,
        )
        frames.append(viz)
    combined = np.concatenate(frames, 1)
    imgviz.io.imsave('reach2.png', combined)
    # imgviz.io.cv_imshow(combined)
    # key = imgviz.io.cv_waitkey(-1)


if __name__ == "__main__":
    main()