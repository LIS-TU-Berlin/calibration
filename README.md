# LIS Calibration Tools

<img src="./docs/assets/lis.png" alt="LIS logo" align="right" width="100" style="margin-left: 20px;">

The `main.py` is the main calibration tool.
It provides options for
1. Data collection, namely charuco board and franka data for intrinsic/extrinsic calibration.
1. Intrinsic calibration (just calling opencv -- could be done equivalently directly in python)
1. Extrinsic calibration (using py bindings of a KOMO formulation)
1. Rendering synthetic images to overlay with the data and check alignment


## Installation

* Requires [robotic](https://github.com/MarcToussaint/robotic)

### Trossen setup (Basler cams, no Franka)

Requires the newest robotic built from source, with the patch in `patches/botop-basler-serials.patch`.
Without it, `launch_baslers` fails on the numeric `basler:` serials in the scenes, and the cams are not opened by serial.

```
cd ~/git && git clone --recursive https://github.com/MarcToussaint/robotic.git && cd robotic
git -C botop apply ~/git/calibration/patches/botop-basler-serials.patch
cp rai/_make/install.sh . && source ./install.sh vars_only
sudo apt install --yes ${ubuntu_rai} ${ubuntu_botop} ${ubuntu_python}
./install.sh libccd && ./install.sh fcl && ./install.sh libann && ./install.sh opencv && ./install.sh physx
./install.sh trossen_arm   # default version; '-v 1.10.0' fails (the tag is v1.10.0)
./install.sh basler        # pylon, scp from hal

cd ~/git/calibration && uv venv && source .venv/bin/activate
uv pip install numpy pybind11 pybind11-stubgen pyyaml opencv-python

cd ~/git/robotic && ln -f -s _make/CMakeLists-ubuntu.txt CMakeLists.txt
cmake . -B build -DUSE_OPENCV=ON -DUSE_BASLER=ON -DUSE_TROSSEN=ON -DUSE_LIBFRANKA=OFF -DUSE_REALSENSE=OFF
make -C build _robotic docstrings install -j $(nproc --ignore 2)
uv pip install -e .
```

USB access for the cams (once; then replug them):
```
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="2676", MODE:="0666"' | sudo tee /etc/udev/rules.d/69-basler-cameras.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'   # not persistent across reboots
```

Intrinsics (scene in `scenes/trossen/`, data in `data/trossen/`):
```
python3 main.py -setup trossen -view -board    # live charuco detection; corner count green = view usable
python3 main.py -setup trossen -collect_board  # 'q' to stop
python3 main.py -setup trossen -intrinsic      # writes scenes/trossen/cams_intrinsics.yml
```
`tests/cam_test.py` checks the cams directly with pypylon (`uv pip install pypylon`), independent of robotic.

Arm connection: the controller (follower, 192.168.1.5, TCP port 50001) does no DHCP, so give the laptop's Ethernet
port (`eth0` on the Dell dock; check with `ip -br link`) a static address in its subnet:
```
sudo nmcli con add type ethernet ifname eth0 con-name trossen ipv4.method manual ipv4.addresses 192.168.1.10/24 ipv4.never-default yes connection.autoconnect-priority 10
sudo nmcli con up trossen
ping -c 3 192.168.1.5 && nc -zv 192.168.1.5 50001   # nc refused: controller still holds an old connection -> power-cycle it
```
The IP is the `trossen:` attribute in `scenes/trossen/trossen.yml`; botop connects with
`bot.launch_trossen(["192.168.1.5"], [frame])` (or `ry.BotOp(C, True)` launches it from the scene).
When botop shuts down the arm goes idle (motors off) and falls if it is not resting:
start and end in the rest pose (`-collect_robot` drives back to its start pose by itself).
`tests/Trossen_test_botop.py`: keyboard IK teleop of the arm (r/f, w/s, a/d move the target, q quits).

Scene (`scenes/trossen/trossen.yml`, view with `ry.Config().addFile('scenes/trossen/scene_init.yml')` and `C.view(True)`):
* `kit/`: the Stationary AI kit on the workbench, from the 26-sgrl-hhi repo (leader meshes removed);
  the right follower and both leaders are deleted, q = 6 arm joints + gripper (as the driver reports it)
* `camera_0..2`: opened by serial, start poses from solvePnP of the table arucos in one image per camera
  (camera_2 is mounted upside down); intrinsics in `cams_intrinsics.yml`
* arucos: 7 on the table, 3 on the arm (35 mm black square)

Extrinsics:
```
python3 main.py -setup trossen -collect_robot -sim -nposes 5   # simulated arm, rendered images -> data/trossen/robot_sim
python3 main.py -setup trossen -collect_robot -nposes 30       # real arm (start in the rest pose, stand at the e-stop; 'q' stops)
python3 main.py -setup trossen -extrinsic                      # cams + aruco positions (-joints 1: also joint offsets)
python3 main.py -setup trossen -reproj                         # check: [real | sim | overlay] per image + pixel errors
```
`-data <folder>` picks another dataset in `data/trossen/` (default `robot`); `-extrinsic` writes the result to
`scenes/trossen/calib/` (used by `scene_calib.yml`) and keeps a copy in `<data>/calib/`. `-reproj` uses that copy
(or `-calib <folder>`) and saves the rows to `<data>/reproj/` (`<data>/reproj_<other calib>/`):
```
python3 main.py -setup trossen -extrinsic -data robot_stripped -exclude 3 20                    # drop bad poses
python3 main.py -setup trossen -reproj -data robot -calib data/trossen/robot_stripped/calib     # other data, same calib
```
Reading the result: komo's final f(x) is a sum over all residuals (not a mean), so it grows with the number of arm
detections and cannot be compared across datasets; compare the per-camera / worst-detection pixel errors of `-reproj`
instead. Table markers should be at 1-2 px; the arm markers are what fix the cameras relative to the robot.
A pose whose arm markers are off by 100+ px in one camera only (while the others fit) was captured mid-motion
or is otherwise broken: `-exclude` it.

First Trossen calibration (2026-11-10, 30 poses, poses 3 and 20 excluded, f(x) 172): table 1.6 px, arm markers
~10 px median (~5 mm at 0.5 m). ar35 was attached to the wrong link (it does not turn with joint 4) and dominated
the earlier error; the arm markers' in-plane yaw was then measured from the images (0.7-5.7 deg, small effect).
Each pose shows a random arm aruco to a random camera (0.3-0.7 m in front of it, facing it, checked for occlusion in a
rendering), keeps the arm 5 cm off the table and 12 cm off the cameras, and is reached on a collision-free KOMO path
at `-max_vel` (default 0.3 rad/s). In simulation (30 poses), cams + aruco positions come out within 1-2 mm of the
true camera positions, also with 5 mm errors in the hand-measured table arucos; calibrating joint offsets too is worse.


## KOMO Calibration Method (extinsic; jointly cams, arucos & joints via bundle adjustment)

### Input
The scene description is an important input to the method:
It needs to include definitions for all the cameras and all all aruco markers.
They should be roughly in the right position and orientation (+- 10cm), otherwise optimization is not well-behaved.

We assume that we collected data for T time slices from C cameras.
This makes a (T,C) array of images.
In each image we ran aruco detection, returning the 2D image coordinates of all aruco corners and their IDs.

To have good data, something needs to vary across time slices.
In the current setup this is the robot(s), on which are markers.
But the method can easily be changed to calibrate with markers on a free object (charuco board, or cube) that is placed randomly in each time slice.

The current method also takes as input the robot joint angles from each time slice.
This allows to co-calibrate the joint angles.
(If instead we'd use a free object, the solver would solve for the object pose freely in each time slice.)

`komo_calibrate` has arguments

* `C` the configuration, which is parsed for basler cams, arucos, and joints
* `pts`, a (T,C) array of (k,4,2) image coordinates of k aruco findings (4 corners each)
* `ids`, a (T,C) array of (k) integers (aruco ids)
* `qs`, a (T,n) array of joint vectors
* `exclude_times`, a list of time slices t to exclude from the optimization
* `calibrate_cams`, introduces static free (7D) calib dofs before each camera
* `calibrate_arucos`, introduces static xy-translation calib dofs before each aruco marker
* `calibrate_joints`, introduces static hingeX calib dofs before each robot joint (currently hacked? to be dofs 7-14?)
* `calib_joint_regularization`, a regularization to penalize too large joint re-calibrations

### What it solves
The methods solves for all introduced dofs by minimizing observation errors over all points, cameras, and time slices jointly.
The core objective is just these two lines:

      undistort_point(p, CS.Fxycxy(c), CS.Distortion(c));
      komo.addObjective({t+1.}, make_shared<F_PointView>(p, CS.Fxycxy(c)), { STRING("arc_"<<a <<"_" <<j), CS.cams(c)->name }, OT_sos, {1e2});

where p is the 2D point in camera c of the j-th corner of aruco a in time slice t.
The PointView is identical to PositionRel multiplied with the skew matrix

      x(0) = (p(0) - fxycxy(2)) / fxycxy(0);
      x(1) = (p(1) - fxycxy(3)) / fxycxy(1);
      x(2) = 1.;
      x_skew = skew(x);

This means the cross product of this vector x with the relative position of p in c-coordinates is penalized with SOS.
This objetive is standard in bundle adjustment.
If the camera poses would be the only dofs of this problem, there are more direct solution techniques (just solving and SVD).
However, as we want to jointly solve also for aruco and joint calib dofs, the generic formulation as KOMO NLP is convenient.

In addition to setting these objectives, all robot joint states in the KOMO path configuration are set to the provided `qs` and frozen (inactivated), while only calibration dofs are activated.
(When a free object is used as calibration variate, its joint state would be an active non-static dof, allowing KOMO to solve for its pose independently in each time slice in addition to the static calibration dofs.)

### What it outputs
Output are the optimal calibration transformations, but merged with the original transformation. (Technically, `f->parent->get_Q() * f->get_Q()` when f is the calibration dof.)

For convenience, these are output as yml-files that define edits of the original transformations and can directly be included in the scene description, namely `calib_cams.yml`, `calib_arucos.yml`, and `calib_joints.yml`.
	

## Helper methods

`scene_info`: Parse the provided scene for cams, arucos and joints, and output info on them -- allowing to understand what other methods do as first step.

`detect_arucos`: gets as input a (T,C) array of images; outputs (T,C) arrays of pts and ids, exactly as needed by the `komo_calibrate` method. (Fully cv2 based.)

`render_images`: gets as input a scene C (which is parsed for cams) and a (T,n) array of joint states; outputs a (T,C) array of rendered images for the T time slices and the C cameras, typically as wireframes so that they can be overlayed with the UNDISTORTED real images.
This is an ultimate test of consistency of everything (including interpretation of intrinsics as OpenGL cam).

`undistort_images`: gets as input a scene C (just to grab camera infos) and a (T,C) array of images, and undistorts them for overlay with the above. (Fully cv2 based.)

## Intrinsic Calibration

`calibrate_intrinsics`: gets as input a (T,C) array of images (with charuco boards presented to one or multiple cams in each time slice), and outputs the C intrinsic calibrations (also as yml-file).
The number of non-linear calibration dofs (2, 4, or 5) is an option.
Errors should be around 0.2 or smaller.
Run with more verbose to check images and charuco detections.
(Fully cv2 based.)


# Practical pipeline

1. Setup the scene config file, with cams, arucos, roughly positions. Use `ry-view` and `scene_info` to check.
1. Use `ry-view-baslers` to run cams (TODO: add option to continuously show charuco boards and/or aruco markers). Ensure framerate around 60Hz.
1. Position cameras. Upright. Aperture (Blende) not too bright (reduces Tiefenschärfe). Properly focus on center object.
1. Collect charuco board data
```
python3.py main.py -collect_board
```
Stop the data collection by pressing `q`. We recommend 30 to 50 images.
1. Calibrate intrinsics
```
python3.py main.py -intrinsic
```
1. Collect franka calib data
```
python3.py main.py -collect_franka
```
Stop the data collection by pressing `q`. We recommend 30 to 50 images.
1. Calibrate extrinsics and robot joints
```
python3.py main.py -extrinsic
```
1. Check with overlaying rendered and undistored images.
```
python3.py main.py -overlay
```
