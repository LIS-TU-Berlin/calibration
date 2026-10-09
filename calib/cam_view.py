import time

import numpy as np
import robotic as ry

from .aruco_detector import ArucoDetector


def cam_view(ncams, draw_arucos=False, draw_charuco=False):
    ry.set_params({"botsim/verbose": 0})

    C = ry.Config()
    bot = ry.BotOp(C, False, False)

    cam_list = [f'{i}' for i in range(ncams)]
    bot.launch_baslers(cam_list, [])

    V = ry.ImageViewer()
    A = ArucoDetector(detect_board=draw_charuco, verbose=1)

    while True:
        time.sleep(.025)

        rgbs = []
        for c in range(ncams):
            rgb = bot.getImage(f'camera_{c}')
            if draw_arucos and c==0:
                A.step(rgb, draw_markers=True)
            rgbs.append(rgb)
            rgb = np.hstack(rgbs)

        key = V.view(rgb, zoom=0.35)
        if key==ord('q'):
            break
