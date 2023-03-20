#!/usr/bin/env python

import argparse
import time

import openvr

import tf.transformations as ttf

import utilities as utils


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "controller_id", type=int, choices=[1, 2], help="controller id"
    )
    args = parser.parse_args()

    openvr.init(openvr.VRApplication_Scene)
    vrsystem = openvr.VRSystem()

    while True:
        pose = vrsystem.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, 16
        )
        pose = pose[args.controller_id].mDeviceToAbsoluteTracking

        matrix = utils.matrix_from_openvr_pose(pose)

        result, state = vrsystem.getControllerState(args.controller_id)
        if result == 0:
            print("Waiting for detecting the controller")
        else:
            assert result == 1
            state = utils.dict_from_controller_state(state)

            position = ttf.translation_from_matrix(matrix)
            print(position, state["trackpad_pressed_button"], state["trigger"])

            time.sleep(0.1)

    openvr.shutdown()


if __name__ == "__main__":
    main()