import os

import cv2
import numpy as np
import robotic as ry

# aruco corners in the marker frame, in the order of the detections (= rai's arc_<id>_<j> frames)
CORNERS = np.array([[-.0175, .0175, 0], [.0175, .0175, 0], [.0175, -.0175, 0], [-.0175, -.0175, 0]])


def reprojection(C: ry.Config, imgs, qs, ids, pts, out_dir=None, show=True, alpha=.5):
    """for each image: [real (undistorted) | scene rendered from the calibrated camera at joint state qs[t] | both
    blended with alpha], with detected (red) and projected (green) aruco corners and their pixel errors on the blend;
    prints the errors per camera; returns the list of (mean corner error, t, c, aruco id), worst first"""
    arucos = {int(C.getFrame(n).getAttributes()["aruco_id"]): n for n in C.getFrameNames() if "aruco_id" in C.getFrame(n).getAttributes()}
    real = ry.calib.undistort_images(C, imgs)
    sim = ry.calib.render_images(C, qs, T=imgs.shape[0])
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    V = ry.ImageViewer() if show else None
    errors = []
    for t in range(imgs.shape[0]):
        C.setJointState(qs[t])
        for c in range(imgs.shape[1]):
            cam = C.getFrame(f"camera_{c}")
            a = cam.getAttributes()
            fx, fy, cx, cy = a["fxycxy"]
            K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
            D = np.array(a["distortion"][:5])
            R, p = cam.getRotationMatrix(), cam.getPosition()
            rvec, tvec = cv2.Rodrigues(R.T)[0], -R.T @ p
            blend = cv2.addWeighted(real[t, c], 1. - alpha, sim[t, c], alpha, 0.)
            for i, aruco_id in enumerate(ids[t, c]):
                if int(aruco_id) not in arucos:
                    continue
                m = C.getFrame(arucos[int(aruco_id)])
                X = m.getPosition() + CORNERS @ m.getRotationMatrix().T
                proj = cv2.projectPoints(X, rvec, tvec, K, np.zeros(5))[0].reshape(4, 2)  # in the undistorted image
                det = cv2.undistortPoints(pts[t, c][i].reshape(4, 1, 2).astype(np.float64), K, D, P=K).reshape(4, 2)
                e = np.linalg.norm(proj - det, axis=1).mean()
                errors.append((e, t, c, int(aruco_id)))
                for x, y in det:
                    cv2.circle(blend, (int(x), int(y)), 7, (255, 0, 0), 2)
                for x, y in proj:
                    cv2.circle(blend, (int(x), int(y)), 3, (0, 255, 0), -1)
                cv2.putText(blend, f"{aruco_id}: {e:.1f}px", (int(det[0, 0]), int(det[0, 1]) - 12), cv2.FONT_HERSHEY_SIMPLEX, .9, (0, 255, 0), 2)
            panels = [real[t, c].copy(), sim[t, c].copy(), blend]
            for im, label in zip(panels, ["real (undistorted)", "sim (calibrated camera)", "overlay  red: detected, green: scene"]):
                cv2.putText(im, f"pose {t} camera_{c}: {label}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255, 255, 0), 3)
            row = np.hstack(panels)
            if out_dir:
                small = cv2.resize(row, (row.shape[1] // 2, row.shape[0] // 2))
                cv2.imwrite(os.path.join(out_dir, f"reproj_{t:04d}_{c}.jpg"), cv2.cvtColor(small, cv2.COLOR_RGB2BGR))
            if V is not None and V.view(row, True, zoom=.33) == ord("q"):
                V = None

    errors.sort(reverse=True)
    for c in range(imgs.shape[1]):
        e = np.array([x[0] for x in errors if x[2] == c])
        if e.size:
            print(f"camera_{c}: {e.size} marker detections, corner error median {np.median(e):.2f}px mean {e.mean():.2f}px max {e.max():.1f}px")
    print("worst:", ", ".join(f"pose {t} camera_{c} aruco {a}: {e:.1f}px" for e, t, c, a in errors[:8]))
    return errors
