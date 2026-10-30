import time

import numpy as np
import robotic as ry

from .aruco_detector import ArucoDetector
from .calibration_scene import CalibrationScene


def cam_view(scene_file, ncams, draw_arucos=False, draw_charuco=False):
    ry.set_params({"botsim/verbose": 0})

    C = ry.Config()
    C.addFile(scene_file)
    S = CalibrationScene(C, None)
    bot = ry.BotOp(C, False, False)

    baslers = S.baslers[:ncams]
    ncams = len(baslers)
    bot.launch_baslers(baslers)

    V = ry.ImageViewer()
    A = ArucoDetector(detect_board=draw_charuco, verbose=0)

    while True:
        time.sleep(.025)

        rgbs = [bot.getImage(f'camera_{c}') for c in range(ncams)]
        if any(rgb.ndim != 3 for rgb in rgbs):  # cams not streaming yet
            continue
        if draw_arucos or draw_charuco:
            rgbs = [A.draw(rgb) for rgb in rgbs]
        rgb = np.vstack(rgbs)

        key = V.view(rgb, zoom=0.35)
        if key==ord('q'):
            break
