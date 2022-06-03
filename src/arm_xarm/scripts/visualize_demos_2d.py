#!/usr/bin/env python

import argparse
import pickle
import sys

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

    # For cropping: 480, 640
    # xmin, xmax = -400, 640
    # ymin, ymax = 0, 400

    for obs_file in sorted(args.saved_dir.listdir()):
        with open(obs_file, "rb") as f:
            obs = pickle.load(f)
            print(list(obs.keys()))
        viz = imgviz.tile(
            # [obs["rgb"][ymin:ymax, xmin:xmax], imgviz.depth2rgb(obs["depth"])[ymin:ymax, xmin:xmax]],
            [obs["rgb"], imgviz.depth2rgb(obs["depth"])],
            shape=(1, 2),
            border=(255, 255, 255),
        )
        imgviz.io.cv_imshow(viz)
        key = imgviz.io.cv_waitkey(1000 // (30 * 4))
        if key == ord("q"):
            break


if __name__ == "__main__":
    main()