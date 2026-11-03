#!/usr/bin/env python3

import os
import argparse

import cv2
import numpy as np
import robotic as ry
from pathlib import Path

from calib.calibration_scene import CalibrationScene
from calib.cam_view import cam_view
from calib.collect_data import collect_data


def load_images(path: str, C: int):
    T = sum(1 for f in os.listdir(path) if f.lower().endswith(".png")) // C
    imgs = np.empty((T, C), dtype=object)
    for t in range(T):
        for c in range(C):
            filename = f"{path}/img_{t:04d}_{c}.png"
            print("loading:", filename)
            imgs[t, c] = cv2.cvtColor(cv2.imread(filename), cv2.COLOR_BGR2RGB)
    return imgs

def move_calib_files(scene: str):
    os.makedirs(f"scenes/{scene}/calib", exist_ok=True)

    files = ["arucos", "cams", "joints", "box"]

    for file in files:
        src = Path(f"calib_{file}.yml")
        dst = Path(f"scenes/{scene}/calib/{file}.yml")
        if src.exists(): src.rename(dst)

    src = Path(f"calib_cams_intrinsics.yml")
    dst = Path(f"scenes/{scene}/cams_intrinsics.yml")
    if src.exists(): src.rename(dst)


def main():
    parser = argparse.ArgumentParser(description="Utility view baslers")

    # Mode
    parser.add_argument("-view", help="view camera images", action="store_true")
    parser.add_argument("-collect_board", help="collect board data", action="store_true")
    parser.add_argument("-collect_franka", help="collect data with the franka moving to cams", action="store_true")
    parser.add_argument("-intrinsic", help="compute intrinsics from the collected board data", action="store_true")
    parser.add_argument("-extrinsic", help="compute extrinsics from the collected franka data", action="store_true")
    parser.add_argument(
        "-overlay",
        help="overlay rendered images on the collected franka data to check calibration",
        action="store_true"
    )
    parser.add_argument("-track", help="track the black cube for testing", action="store_true")

    # Params
    parser.add_argument("-ncams", type=int, help="number of cams", nargs="?", default=3)
    parser.add_argument("-scene", type=str, help="scene folder", default="single_right")
    args = parser.parse_args()

    if args.view:
        cam_view(args.ncams, True)

    elif args.collect_board or args.collect_franka:
        path = "data/franka" if args.collect_franka else "data/board"
        
        C = ry.Config()
        C.addFile(
            os.path.join("scenes", args.scene, "scene_init.yml")
        )
        C.view(False, 'loaded scene')
        
        # S = CalibrationScene(C, "obj")
        S = CalibrationScene(C, "")
        collect_data(C, S, use_franka=args.collect_franka, data_path=path)

    elif args.intrinsic:
        path = "data/board"
        imgs = load_images(path, args.ncams)
        print(imgs.shape)
        Fxycxy, Distortion = ry.calib.calibrate_intrinsics(imgs, distortionDofs=2, verbose=2)
        print("INTRINSICS:\n", Fxycxy, Distortion)
        move_calib_files(args.scene)

    elif args.extrinsic:
        path = "data/franka"
        C = ry.Config()
        C.addFile(
            os.path.join("scenes", args.scene, "scene_init.yml")
        )
        
        imgs = load_images(path, args.ncams)
        ids, pts = ry.calib.detect_arucos(imgs, verbose=0)
        qs = np.loadtxt(f"{path}/q_arr.txt")
        qs = qs[: imgs.shape[0]]

        # C.selectJoints(["obj"], notThose=True)

        ry.calib.komo_calibrate(
            C,
            ids,
            pts,
            qs,
            exclude_times=[11, 42, 32, 33, 36, 12, 21, 5, 26, 31, 43, 41, 4, 28, 27, 3, 2, 25, 20, 34, 1, 22, 23, 24, 39, 0, 10, 13, 9, 14, 37, 17, 19], #todo make this list a cmd line argument
            calibrate_cams=True,
            calibrate_arucos=True,
            calibrate_joints=True,
            calibrate_objPoses=False,
            calib_joint_regularization=1e1,
            verbose=1,
        )

        move_calib_files(args.scene)

    elif args.overlay:
        path = "data/franka"
        C = ry.Config()
        C.addFile(
            os.path.join("scenes", args.scene, "scene_calib.yml")
        )
        imgs = load_images(path, args.ncams)
        qs = np.loadtxt(f"{path}/q_arr.txt")

        undistored = ry.calib.undistort_images(C, imgs)
        rendered = ry.calib.render_images(C, qs, T=imgs.shape[0])

        V = ry.ImageViewer()
        for t in range(imgs.shape[0]):
            for c in range(imgs.shape[1]):
                V.view(undistored[t, c] / 4 * 3 + rendered[t, c] / 4, True, zoom=1.0)

    elif args.track:
        C = ry.Config()
        C.addFile(
            os.path.join("scenes", args.scene, "scene_cube.yml")
        )

        bot = ry.BotOp(C, False, False)
        bot.launch_baslers([str(i) for i in range(4)], [])
        bot.launch_arucos()
        bot.launch_arucoObjTracker(C, "obj")

        while(True):
                key = bot.sync(C, .05)
                if key==ord('q'):
                        break

    else:
        parser.print_help()
        print('=== PLEASE SPECIFY A COMMAND LINE OPTION ===')

if __name__ == "__main__":
    np.set_printoptions(suppress=True, precision=4)

    main()
