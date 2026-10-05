#!/usr/bin/env python3

import argparse

import cv2
import numpy as np
import robotic as ry

from calib.calibration_scene import CalibrationScene
from calib.cam_view import cam_view
from calib.collect_data import collect_data


def load_images(path, T, C):
    imgs = np.empty((T, C), dtype=object)
    for t in range(T):
        for c in range(C):
            filename = f"{path}/img_{t:04d}_{c}.png"
            print("loading:", filename)
            imgs[t, c] = cv2.imread(filename)
    return imgs


def main():
    parser = argparse.ArgumentParser(description="Utility view baslers")
    parser.add_argument("-view", help="view camera images", action="store_true")
    parser.add_argument("-collect_board", help="collect board data", action="store_true")
    parser.add_argument("-collect_franka", help="collect data with the franka moving to cams", action="store_true")
    parser.add_argument("-intrinsic", help="compute intrinsics from the collected board data", action="store_true")
    parser.add_argument("-extrinsic", help="compute extrinsics from the collected franka data", action="store_true")
    parser.add_argument(
        "-overlay",
        help="overlay rendered images on the collected franka data to check calibration",
        action="store_true",
    )

    parser.add_argument("-ncams", type=int, help="number of cams", nargs="?", default=4)
    args = parser.parse_args()

    if args.view:
        cam_view(args.ncams, True)

    elif args.collect_board:
        C = ry.Config()
        C.addFile("scenes/station.yml")
        C.view(False, 'loaded scene')
        S = CalibrationScene(C, "obj")
        collect_data(C, S, use_franka=False, real=True)

    elif args.collect_franka:
        C = ry.Config()
        C.addFile("scenes/station.yml")
        C.view(False, 'loaded scene')
        S = CalibrationScene(C, "obj")
        collect_data(C, S, use_franka=True, real=True)

    elif args.intrinsic:
        path = "data/board"
        imgs = load_images(path, 30, 4)  # TODO: should know from files how many images
        Fxycxy, Distortion = ry.calib.calibrate_intrinsics(imgs, distortionDofs=2, verbose=2)
        print("INTRINSICS:\n", Fxycxy, Distortion)

    elif args.extrinsic:
        path = "data/franka"
        C = ry.Config()
        C.addFile("scenes/station.yml")
        imgs = load_images(path, 50, 4)
        ids, pts = ry.calib.detect_arucos(imgs, verbose=5)
        qs = np.loadtxt(f"{path}/q_arr.txt")
        qs = qs[: imgs.shape[0], :]

        ry.calib.komo_calibrate(
            C,
            ids,
            pts,
            qs,
            exclude_times=[33, 48, 29, 28, 49, 43, 32],
            calibrate_cams=True,
            calibrate_arucos=True,
            calibrate_joints=True,
            calibrate_objPoses=False,
            calib_joint_regularization=1e1,
            verbose=2,
        )
        return imgs, qs

    elif args.overlay:
        path = "data/franka"
        C = ry.Config()
        C.addFile("scenes/station.yml")
        imgs = load_images(path, 20, 4)
        qs = np.loadtxt(f"{path}/q_arr.txt")

        undistored = ry.calib.undistort_images(C, imgs)
        rendered = ry.calib.render_images(C, qs, T=imgs.shape[0])

        V = ry.ImageViewer()
        for t in range(imgs.shape[0]):
            for c in range(imgs.shape[1]):
                V.view(undistored[t, c] / 4 * 3 + rendered[t, c] / 4, True, zoom=1.0)

    else:
        parser.print_help()
        print('=== PLEASE SPECIFY A COMMAND LINE OPTION ===')

if __name__ == "__main__":
    np.set_printoptions(suppress=True, precision=4)

    main()
