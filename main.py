#!/usr/bin/env python3

import os
import argparse
import shutil
import tempfile

import cv2
import numpy as np
import robotic as ry
from pathlib import Path

from calib.calibration_scene import CalibrationScene
from calib.cam_view import cam_view
from calib.collect_data import collect_data
from calib.reprojection import reprojection


def load_images(path):
    files = [f for f in os.listdir(path) if f.startswith("img_") and f.lower().endswith(".png")]
    C = 1 + max(int(f[:-4].split("_")[2]) for f in files)
    T = len(files) // C
    imgs = np.empty((T, C), dtype=object)
    for t in range(T):
        for c in range(C):
            filename = f"{path}/img_{t:04d}_{c}.png"
            print("loading:", filename)
            imgs[t, c] = cv2.cvtColor(cv2.imread(filename), cv2.COLOR_BGR2RGB)
    return imgs

def move_calib_files(scene_dir):
    os.makedirs(f"{scene_dir}/calib", exist_ok=True)

    files = ["arucos", "cams", "joints", "box"]

    for file in files:
        src = Path(f"calib_{file}.yml")
        dst = Path(f"{scene_dir}/calib/{file}.yml")
        if src.exists(): src.rename(dst)

    src = Path(f"calib_cams_intrinsics.yml")
    dst = Path(f"{scene_dir}/cams_intrinsics.yml")
    if src.exists(): src.rename(dst)


def default_calib(path, scene_dir):
    return f"{path}/calib" if os.path.exists(f"{path}/calib/cams.yml") else f"{scene_dir}/calib"


def calibrated_config(scene_dir, calib_dir):
    """scene_init.yml + the calibration in calib_dir (a separate addFile cannot Edit frames of an earlier one, so this
    writes a scene file that includes both)"""
    lines = [f"Include: <{os.path.abspath(scene_dir)}/scene_init.yml>"]
    for f in ["cams", "arucos", "joints"]:
        if os.path.exists(f"{calib_dir}/{f}.yml"):
            lines.append(f"Include: <{os.path.abspath(calib_dir)}/{f}.yml>")
    with tempfile.NamedTemporaryFile("w", suffix=".yml", delete=False) as fil:
        fil.write("\n".join(lines) + "\n")
    C = ry.Config()
    C.addFile(fil.name)
    os.remove(fil.name)
    return C


def main():
    parser = argparse.ArgumentParser(description="Utility view baslers")

    # Mode
    parser.add_argument("-view", help="view camera images", action="store_true")
    parser.add_argument("-collect_board", help="collect board data", action="store_true")
    parser.add_argument("-collect_franka", "-collect_robot", dest="collect_robot", help="collect data with the robot (franka or trossen, from the scene) moving markers to the cams", action="store_true")
    parser.add_argument("-intrinsic", help="compute intrinsics from the collected board data", action="store_true")
    parser.add_argument("-extrinsic", help="compute extrinsics from the collected franka data", action="store_true")
    parser.add_argument(
        "-overlay",
        help="overlay rendered images on the collected franka data to check calibration",
        action="store_true",
    )
    parser.add_argument("-reproj", help="project the calibrated scene's arucos into the collected robot images: pixel errors + images in .../reproj", action="store_true")
    parser.add_argument("-track", help="track the black cube for testing", action="store_true")
    parser.add_argument("-board", help="with -view: show charuco board detections (corner count green = usable for -intrinsic)", action="store_true")

    # Params
    parser.add_argument("-ncams", type=int, help="number of cams (default: all in the scene)", nargs="?", default=None)
    parser.add_argument("-setup", help="station (scenes/, data/) or e.g. trossen (scenes/trossen/, data/trossen/)", default="station")
    parser.add_argument("-sim", help="with -collect_robot: simulated robot, rendered images (data in .../robot_sim; also for -extrinsic)", action="store_true")
    parser.add_argument("-data", help="robot data folder in data/<setup>/ for -collect_robot/-extrinsic/-overlay/-reproj (default: robot, robot_sim with -sim)", default=None)
    parser.add_argument("-calib", help="with -overlay/-reproj: folder with the calibration (cams.yml, arucos.yml) (default: <data>/calib if -extrinsic wrote one, else scenes/<setup>/calib)", default=None)
    parser.add_argument("-nposes", type=int, help="with -collect_robot: stop after that many poses (default: until 'q')", default=0)
    parser.add_argument("-max_vel", type=float, help="with -collect_robot (trossen): max joint velocity [rad/s]", default=.3)
    parser.add_argument("-joints", type=int, choices=[0, 1], help="with -extrinsic: also calibrate joint (zero) offsets (default: 1, 0 with -sim where there are none)", default=None)
    parser.add_argument("-exclude", type=int, nargs="*", help="with -extrinsic: time slices to exclude (default station: 22 10 20 42, else none)", default=None)
    args = parser.parse_args()

    scene_dir = "scenes" if args.setup == "station" else f"scenes/{args.setup}"
    data_dir = "data" if args.setup == "station" else f"data/{args.setup}"
    robot_dir = f"{data_dir}/franka" if args.setup == "station" else f"{data_dir}/robot" + ("_sim" if args.sim else "")
    if args.data:
        robot_dir = f"{data_dir}/{args.data}"
    exclude = args.exclude if args.exclude is not None else ([22, 10, 20, 42] if args.setup == "station" else [])

    if args.view:
        cam_view(f"{scene_dir}/scene_init.yml", args.ncams, draw_arucos=not args.board, draw_charuco=args.board)

    elif args.collect_board:
        path = f"{data_dir}/board"
        C = ry.Config()
        C.addFile(f"{scene_dir}/scene_init.yml")
        C.view(False, 'loaded scene')
        S = CalibrationScene(C, "obj")
        collect_data(C, S, use_robot=False, data_path=path, ncams=args.ncams)

    elif args.collect_robot:
        path = robot_dir
        C = ry.Config()
        C.addFile(f"{scene_dir}/scene_init.yml")
        C.view(False, 'loaded scene')
        S = CalibrationScene(C, "obj")
        collect_data(C, S, use_robot=True, data_path=path, ncams=args.ncams, sim=args.sim, nposes=args.nposes, max_vel=args.max_vel)

    elif args.intrinsic:
        path = f"{data_dir}/board"
        imgs = load_images(path)
        Fxycxy, Distortion = ry.calib.calibrate_intrinsics(imgs, distortionDofs=2, verbose=2)
        print("INTRINSICS:\n", Fxycxy, Distortion)
        move_calib_files(scene_dir)

    elif args.extrinsic:
        path = robot_dir
        joints = bool(args.joints if args.joints is not None else not args.sim)
        C = ry.Config()
        C.addFile(f"{scene_dir}/scene_init.yml")
        imgs = load_images(path)
        ids, pts = ry.calib.detect_arucos(imgs, verbose=5)
        qs = np.loadtxt(f"{path}/q_arr.txt", ndmin=2)
        qs = qs[: imgs.shape[0], :]
        if args.sim:  # the renderings are ideal pinhole cameras: principal point at the center, no distortion
            for f in CalibrationScene(C, None).baslers:
                a = f.getAttributes()
                f.setAttributes({"fxycxy": [a["fxycxy"][1], a["fxycxy"][1], a["width"] / 2, a["height"] / 2], "distortion": [0.] * 5})

        ry.calib.komo_calibrate(
            C,
            ids,
            pts,
            qs,
            exclude_times=exclude,
            calibrate_cams=True,
            calibrate_arucos=True,
            calibrate_joints=joints,
            calibrate_objPoses=False,
            calib_joint_regularization=1e1,
            verbose=2,
        )

        move_calib_files(scene_dir)
        if not joints:  # no stale joint offsets from an earlier run (scene_calib.yml includes calib/joints.yml)
            with open(f"{scene_dir}/calib/joints.yml", "w") as fil:
                fil.write("# no joint offsets (-extrinsic -joints 0)\n")
        # keep a copy with the data, so each dataset's result can be checked later (-reproj uses it)
        os.makedirs(f"{path}/calib", exist_ok=True)
        for f in ["cams", "arucos", "joints"]:
            if os.path.exists(f"{scene_dir}/calib/{f}.yml"):
                shutil.copy(f"{scene_dir}/calib/{f}.yml", f"{path}/calib/{f}.yml")
        print(f"-- calibration in {scene_dir}/calib/ and {path}/calib/")

    elif args.overlay:
        path = robot_dir
        C = calibrated_config(scene_dir, args.calib or default_calib(path, scene_dir))
        imgs = load_images(path)
        qs = np.loadtxt(f"{path}/q_arr.txt", ndmin=2)

        undistored = ry.calib.undistort_images(C, imgs)
        rendered = ry.calib.render_images(C, qs, T=imgs.shape[0])

        V = ry.ImageViewer()
        for t in range(imgs.shape[0]):
            for c in range(imgs.shape[1]):
                V.view(undistored[t, c] / 4 * 3 + rendered[t, c] / 4, True, zoom=1.0)

    elif args.reproj:
        path = robot_dir
        calib_dir = args.calib or default_calib(path, scene_dir)
        print(f"-- data {path}, calibration {calib_dir}")
        C = calibrated_config(scene_dir, calib_dir)
        imgs = load_images(path)
        ids, pts = ry.calib.detect_arucos(imgs, verbose=0)
        qs = np.loadtxt(f"{path}/q_arr.txt", ndmin=2)
        # own calibration -> <data>/reproj, another one (-calib X) -> <data>/reproj_<X's folder name>
        own = os.path.normpath(calib_dir) == os.path.normpath(f"{path}/calib")
        name = os.path.basename(os.path.normpath(calib_dir))
        if name == "calib":  # e.g. data/trossen/robot/calib -> robot
            name = os.path.basename(os.path.dirname(os.path.normpath(calib_dir)))
        reprojection(C, imgs, qs, ids, pts, out_dir=f"{path}/reproj" + ("" if own else "_" + name))

    elif args.track:
        C = ry.Config()
        C.addFile(f"{scene_dir}/scene_cube.yml")

        bot = ry.BotOp(C, False, False)
        bot.launch_baslers(CalibrationScene(C, None).baslers)
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
