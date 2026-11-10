import os
import time

import cv2
import numpy as np
import robotic as ry

from .calibration_scene import CalibrationScene


def fresh_images(bot, ncams, nframes=2, timeout=3.):
    """the images of all cameras, each at least nframes new frames after this call (getImage returns a camera's
    latest frame, which is old when its grabs stall -- e.g. with too little usbfs memory); None on timeout"""
    last = [bot.getImage(f"camera_{c}") for c in range(ncams)]
    count = [0] * ncams
    t0 = time.time()
    while min(count) < nframes:
        if time.time() - t0 > timeout:
            stalled = [f"camera_{c}" for c in range(ncams) if count[c] < nframes]
            print(f"-- !! no new frames from {stalled} for {timeout}s: pose skipped "
                  "(check: cat /sys/module/usbcore/parameters/usbfs_memory_mb, should be 1000)")
            return None
        time.sleep(.005)
        for c in range(ncams):
            if count[c] < nframes:
                img = bot.getImage(f"camera_{c}")
                if not np.array_equal(img, last[c]):  # sensor noise: a new frame always differs
                    last[c] = img
                    count[c] += 1
    if time.time() - t0 > .5:
        print(f"-- slow cameras: waited {time.time() - t0:.1f}s for new frames")
    return last


def collect_data(C: ry.Config, S: CalibrationScene, use_robot=False, data_path="data/", ncams=None, sim=False,
                 nposes=0, max_vel=.3):
    """use_robot: move the robot to random poses (franka: station; trossen: scene frame with a 'trossen' ip attribute)
    sim: simulated robot, no cameras -- saves renderings instead; nposes: stop after that many (0: until 'q')"""

    if os.path.isdir(data_path) and os.listdir(data_path):  # don't mix with an old run (load_images counts the files)
        old = f"{data_path.rstrip('/')}_{time.strftime('%Y%m%d-%H%M%S')}"
        os.rename(data_path, old)
        print(f"-- moved the previous data to {old}")
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
                while bot.getTimeToEnd() > 0:  # bot.wait also returns on any key: always finish the motion
                    if bot.wait(C) == ord("q"):
                        stop = True
                if stop:
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

            if sim:
                rgbs = S.render_images(q_now)[:ncams]
            else:
                rgbs = fresh_images(bot, ncams)
                if rgbs is None:  # a camera delivered no new frames: an old image would not match q_now
                    continue
            Q.append(q_now)
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
            while bot.getTimeToEnd() > 0:
                bot.sync(C, .1)
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
