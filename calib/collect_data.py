import os
import time

import cv2
import numpy as np
import robotic as ry

from .calibration_scene import CalibrationScene


def collect_data(C: ry.Config, S: CalibrationScene, use_franka=False, data_path="data/"):

    os.makedirs(data_path, exist_ok=True)
    ry.set_params({"botsim/verbose": 0, "bot/useAudio": True})

    V = ry.ImageViewer()

    bot = ry.BotOp(C, True, False)

    if S.basler_ids:
        ncams = len(S.basler_ids)
        bot.launch_baslers(S.baslers)
    
    elif S.realsense_ids:
        ncams = len(S.realsense_ids)
        bot.launch_MultiRealSense(S.realsenses, captureColor=True, captureDepth=True)

    else:
        raise Exception("No camera hardware specified!")

    if use_franka:
        bot.launch_franka(S.panda_ip, S.panda_base)
        bot.home(C)

    t = 0
    Q = []
    stop = False
    while not stop:
        if use_franka:
            feasible = False

            while not feasible:
                q_next = S.get_random_pose()
                path, feasible = S.make_path(bot.get_q(), q_next)  # This avoids hitting the table between poses

            bot.moveAutoTimed(path, 1.0, 1.0)
            C.viewer().raiseWindow()
            key = bot.wait(C)
            if key == ord("q"):
                stop = True
                break

            time.sleep(0.5)

        else:
            # just loop displaying images
            for i in range(40):
                time.sleep(0.025)
                rgbs = []

                # Sometimes not all cameras are active, so we need to wait
                c = 0
                while c < ncams:
                    rgbs.append(bot.getImage(f"camera_{c}"))
                    if not rgbs[-1].shape[0]:
                        rgbs = []
                        c = 0
                        continue
                    c += 1

                rgb = np.hstack(rgbs)
                key = V.view(rgb, zoom=0.4)
                if key == ord("q"):
                    stop = True
                    break

                if i == 30:
                    bot.sound(7, 1.0)

        if stop:
            break

        print("-- capture", t)
        bot.sound(12, 1.0)
        bot.sync(C, 0.0)
        q_now = bot.get_q()
        Q.append(q_now)

        rgbs = []
        for c in range(ncams):
            rgbs.append(bot.getImage(f"camera_{c}"))
        rgb = np.hstack(rgbs)
        key = V.view(rgb, zoom=0.4)

        for c in range(ncams):
            path = os.path.join(data_path, f"img_{t:04d}_{c}.png")
            cv2.imwrite(path, cv2.cvtColor(rgbs[c], cv2.COLOR_RGB2BGR))

        t += 1

    Q = np.stack(Q)
    path = os.path.join(data_path, "q_arr.txt")
    np.savetxt(path, Q)

    if use_franka:
        bot.home(C)
