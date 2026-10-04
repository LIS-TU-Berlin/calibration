import cv2
import numpy as np
import robotic as ry


def load_images(path, T, C):
    imgs = np.empty((T, C), dtype=object)
    for t in range(T):
        for c in range(C):
            filename = f"{path}/img_{t:04d}_{c}.png"
            print("loading:", filename)
            imgs[t, c] = cv2.imread(filename)
    return imgs


def step1_intrinsics(path="data/board"):
    imgs = load_images(path, 30, 4)
    print(type(imgs), imgs.shape)
    Fxycxy, Distortion = ry.calib.calibrate_intrinsics(imgs, distortionDofs=2, verbose=2)
    print("INTRINSICS:\n", Fxycxy, Distortion)


def step2_extrinsics(C, path="data/franka"):
    imgs = load_images(path, 20, 4)
    ids, pts = ry.calib.detect_arucos(imgs, verbose=5)
    # ids = ids.reshape(imgs.shape)
    # pts = pts.reshape(imgs.shape)
    qs = np.loadtxt(f"{path}/q_arr.txt")
    qs = qs[: imgs.shape[0], :]
    qs = np.hstack((np.zeros((20, 7)), qs, np.ones((20, 7))))

    ry.calib.komo_calibrate(
        C,
        ids,
        pts,
        qs,
        exclude_times=[],
        calibrate_cams=True,
        calibrate_arucos=True,
        calibrate_joints=True,
        calibrate_objPoses=False,
        calib_joint_regularization=1e1,
        verbose=1,
    )
    return imgs, qs


def step3_overlay(C, imgs, qs):
    undistored = ry.calib.undistort_images(C, imgs)
    rendered = ry.calib.render_images(C, qs, T=imgs.shape[0])

    V = ry.ImageViewer()
    for t in range(imgs.shape[0]):
        for c in range(imgs.shape[1]):
            V.view(undistored[t, c] / 4 * 3 + rendered[t, c] / 4, True, zoom=1.0)


def main():
    C = ry.Config()
    C.addFile("scenes/station.yml")

    # step1_intrinsics()

    imgs, qs = step2_extrinsics(C)

    C = ry.Config()
    C.addFile("scenes/station.yml")

    step3_overlay(C, imgs, qs)


if __name__ == "__main__":
    np.set_printoptions(suppress=True, precision=4)

    main()
