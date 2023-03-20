#!/usr/bin/env python

import argparse
import pickle
import time

import numpy as np
import path
import trimesh

import utilities as utils
import scipy


def main(saved_dir, stride=1):
    scene = trimesh.Scene()

    # bounds = [-0.05, -0.6, -0.1, 0.75, 0.2, 0.7]
    # bounds = [0.1, -0.6, -0.1, 0.8, -0.1, 0.6]  # Kitchen
    # bounds = [0.1, -0.35, -0.1, 0.8, 0.35, 0.6]

    for i, obs_file in enumerate(sorted(saved_dir.listdir())):

        if i % stride != 0:
            continue

        if len(obs_file) < 4 or obs_file[-4:] != ".pkl":
            continue

        with open(obs_file, "rb") as f:
            obs = pickle.load(f)

        # import ipdb; ipdb.set_trace()

        K = obs["K"]
        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]

        points = utils.pointcloud_from_depth(
            obs["depth"], fx=fx, fy=fy, cx=cx, cy=cy
        )
        keep = np.logical_and(~np.isnan(obs["depth"]), obs["depth"] < 2.0)
        points = points[keep]
        colors = obs["rgb"][keep]

        points = trimesh.transform_points(points, obs["T_camera_in_link0"])

        # keep = points[..., 0] > bounds[0]
        # keep = np.logical_and(keep, points[..., 1] > bounds[1])
        # keep = np.logical_and(keep, points[..., 2] > bounds[2])
        # keep = np.logical_and(keep, points[..., 0] < bounds[3])
        # keep = np.logical_and(keep, points[..., 1] < bounds[4])
        # keep = np.logical_and(keep, points[..., 2] < bounds[5])
        # points = points[keep]
        # colors = colors[keep]

        geom = trimesh.PointCloud(vertices=points, colors=colors)
        scene.add_geometry(geom)

        geom = trimesh.creation.axis(origin_size=0.01)
        scene.add_geometry(geom)

        geom = trimesh.creation.axis(origin_size=0.01)
        p_ee = obs["p_ee_in_link0"]

        quat_matrix = scipy.spatial.transform.Rotation.from_quat(p_ee[3:]).as_matrix()

        T_ee = np.eye(4)
        T_ee[:3, :3] = quat_matrix
        T_ee[:3, 3] = p_ee[:3]

        geom.apply_transform(T_ee)
        scene.add_geometry(geom)

    
    scene.show()        
        


if __name__ == "__main__":

    # collect all demo dirs from /home/xingyu/data/0303_faucet_joso/ using natsorted

    # for i in range(1, 11):
    #     args = argparse.Namespace()
    #     args.saved_dir = path.Path(f"/home/xingyu/data/0303_faucet_joso/demo_{i}")
    #     main(args)

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--saved_dir", type=path.Path, help="saved dir")
    parser.add_argument('--stride', type=int, default=1)
    args = parser.parse_args()

    main(**vars(args))

