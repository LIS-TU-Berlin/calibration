import os
import time

import cv2
import numpy as np
import robotic as ry

from .calibration_scene import CalibrationScene


def collect_data(C: ry.Config, S: CalibrationScene, use_robot=False, data_path="data/", ncams=None, sim=False,
                 nposes=0, max_vel=.3):
    """use_robot: move the robot to random poses (franka: station; trossen: scene frame with a 'trossen' ip attribute)
    sim: simulated robot, no cameras -- saves renderings instead; nposes: stop after that many (0: until 'q')"""

    os.makedirs(data_path, exist_ok=True)
    ry.set_params({"botsim/verbose": 0, "bot/useAudio": True})

    V = ry.ImageViewer()
    baslers = S.baslers[:ncams]
    ncams = len(baslers)
    trossen = [n for n in C.getFrameNames() if "trossen" in C.getFrame(n).getAttributes()]

    bot = ry.BotOp(C, not sim, False)
    if sim:
        bot.launch_simulation(C)
    else:
        bot.launch_baslers(baslers)

    if use_robot and not sim:
        if trossen:
            f = C.getFrame(trossen[0])
            bot.launch_trossen([str(f.getAttributes()["trossen"])], [f])
        else:
            bot.launch_franka("172.17.0.2", C.getFrame("r_panda_base"))  # TODO: works only for right panda...

    q_start = bot.get_q()

    t = 0
    Q = []
    stop = False
    try:
        while not stop and (nposes == 0 or t < nposes):
            if use_robot and trossen:
                q_now = bot.get_q()
                while True:
                    q_next, cam, ar = S.get_random_marker_pose(q_now, min_z=.825 + .08)
                    path, feasible = S.plan_path(q_now, q_next)
                    if feasible:
                        break
                print(f"-- pose {t}: {ar} -> {cam}")
                bot.moveAutoTimed(path, max_vel, max_vel)
                C.viewer().raiseWindow()
                key = bot.wait(C)
                if key == ord("q"):
                    stop = True
                    break

                time.sleep(0.5)

            elif use_robot:
                q_next = S.get_random_pose()
                bot.moveTo(q_next)
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
                    for c in range(ncams):
                        rgbs.append(bot.getImage(f"camera_{c}"))
                    rgb = np.vstack(rgbs)
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

            if sim:
                rgbs = S.render_images(q_now)[:ncams]
            else:
                rgbs = []
                for c in range(ncams):
                    rgbs.append(bot.getImage(f"camera_{c}"))
            rgb = np.hstack(rgbs)
            key = V.view(rgb, zoom=0.4)

            for c in range(ncams):
                path = os.path.join(data_path, f"img_{t:04d}_{c}.png")
                cv2.imwrite(path, cv2.cvtColor(rgbs[c], cv2.COLOR_RGB2BGR))

            t += 1

    finally:
        if Q:
            np.savetxt(os.path.join(data_path, "q_arr.txt"), np.stack(Q))

        if use_robot and trossen:
            # back to the start (rest) pose: when botop shuts down, the trossen goes idle (motors off)
            q_now = bot.get_q()
            path, feasible = S.plan_path(q_now, q_start)
            if feasible:
                print("-- back to the start pose")
                bot.moveAutoTimed(path, max_vel, max_vel)
                bot.wait(C, forKeyPressed=False)
            else:
                input("-- !! no collision-free path back to the start pose: support the arm, then press Enter to release it")
        elif use_robot:
            bot.home(C)
