import cv2
import numpy as np
import robotic as ry


class CalibrationScene:
    def __init__(self, C: ry.Config, obj_name):
        self.C = C
        self.q_home = C.getJointState()
        self.baslers = []
        self.basler_ids = []
        self.arucos = []
        self.aruco_ids = []
        self.obj = None
        self.Fxycxy = []
        self.Distortion = []
        F = C.getFrames()

        for f in F:
            atts = f.getAttributes()
            if "basler" in atts:
                self.baslers.append(f)
                self.basler_ids.append(int(atts["basler"]))
                self.Fxycxy.append(atts["fxycxy"])
                self.Distortion.append(atts["distortion"])

            if "aruco_id" in atts:
                self.arucos.append(f)
                self.aruco_ids.append(int(atts["aruco_id"]))

        # arucos that move with the joints (on the robot)
        X0 = [f.getPosition() for f in self.arucos]
        C.setJointState(self.q_home + .1)
        self.arm_arucos = [f for f, x in zip(self.arucos, X0) if np.linalg.norm(f.getPosition() - x) > 1e-4]
        C.setJointState(self.q_home)

        if obj_name is not None:
            self.obj = C.getFrame(obj_name)

    def get_random_pose(self):
        gripper = self.C.getFrame("r_gripper")

        self.C.setJointState(self.q_home)

        komo = ry.KOMO(self.C, 1, 1, 0, True)

        # pick random cam
        cam = np.random.choice(self.baslers)
        offset = 0.1 * np.random.randn(3)
        komo.addObjective([1.0], ry.FS.positionRel, [gripper.name, cam.name], ry.OT.sos, [1.0, 1.0, 0.0], offset)
        komo.addObjective([1.0], ry.FS.positionRel, [cam.name, gripper.name], ry.OT.sos, [1.0, 0.0, 1.0])
        komo.addObjective([1.0], ry.FS.positionRel, [gripper.name, "table"], ry.OT.ineq, [0.0, 0.0, -1e2], [0.0, 0.0, 0.1])
        komo.addObjective([1.0], ry.FS.accumulatedCollisions, [], ry.OT.eq, [1e2])

        sol = ry.NLP_Solver()
        sol.setProblem(komo.nlp())
        ret = sol.solve()
        print("===", cam.name, offset, "\n", ret)

        # komo.view(True, "optimized")
        q = komo.getPath().reshape(-1)
        if ret.feasible:
            return q
        else:
            print("-- infeasible!")
            return self.get_random_pose()

    def get_random_marker_pose(self, q_start, min_z, depth=(.3, .7), tries=30):
        """random collision-free robot pose that shows a random arm aruco to a random camera: the marker in front of the
        camera (near the optical axis, at the given depth), facing it, above min_z (world z), and not occluded in a
        rendering of the scene; returns (q, camera name, aruco name)"""
        C = self.planning_config()
        for _ in range(tries):
            cam = np.random.choice(self.baslers)
            ar = np.random.choice(self.arm_arucos)
            offset = .07 * np.random.randn(2)

            C.setJointState(q_start)
            komo = ry.KOMO(C, 1, 1, 0, True)
            komo.addControlObjective([], 0, 1e-1)
            komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
            komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, [1e2])
            self.add_table_margin(komo, [])
            # marker in camera coordinates: near the optical axis, depth within range
            komo.addObjective([1.], ry.FS.positionRel, [ar.name, cam.name], ry.OT.sos, [1e1, 1e1, 0.], [*offset, 0.])
            komo.addObjective([1.], ry.FS.positionRel, [ar.name, cam.name], ry.OT.ineq, [0., 0., -1e1], [0., 0., depth[0]])
            komo.addObjective([1.], ry.FS.positionRel, [ar.name, cam.name], ry.OT.ineq, [0., 0., 1e1], [0., 0., depth[1]])
            # camera in marker coordinates: on the marker's normal, in front of it (= marker faces the camera)
            komo.addObjective([1.], ry.FS.positionRel, [cam.name, ar.name], ry.OT.sos, [3., 3., 0.])
            komo.addObjective([1.], ry.FS.positionRel, [cam.name, ar.name], ry.OT.ineq, [0., 0., -1e1], [0., 0., .1])
            komo.addObjective([1.], ry.FS.position, [ar.name], ry.OT.ineq, [0., 0., -1e1], [0., 0., min_z])

            ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
            if not ret.feasible:
                print(f"-- infeasible: {ar.name} -> {cam.name}")
                continue
            q = komo.getPath().reshape(-1)
            if int(ar.getAttributes()["aruco_id"]) not in self.rendered_arucos(q)[self.baslers.index(cam)]:
                print(f"-- occluded: {ar.name} -> {cam.name}")
                continue
            return q, cam.name, ar.name
        raise RuntimeError(f"no feasible marker pose in {tries} tries")

    def plan_path(self, q_start, q_target, T=30):
        """collision-free joint path from q_start to q_target (KOMO, T slices); returns (path, feasible)"""
        C = self.planning_config()
        C.setJointState(q_start)
        # qItself also lists mimic joints (e.g. the Trossen's second gripper finger): map each joint to its q index
        y_target = np.asarray(q_target)[[C.getFrame(j).getJointQIndex() for j in C.getJointNames()]]
        komo = ry.KOMO(C, 1., T, 2, True)
        komo.addControlObjective([], 2, 1.)
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, [1e2])
        self.add_table_margin(komo, [])
        komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, [1e1], y_target)
        komo.addObjective([1.], ry.FS.qItself, [], ry.OT.eq, [1e1], [], order=1)
        ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()
        return komo.getPath(), ret.feasible

    def add_table_margin(self, komo, times, table="workbench_top", margin=.05):
        """keep all moving collision shapes of the robot at least margin away from the table (the collision constraint
        alone lets them touch it)"""
        C = self.planning_config()
        if not hasattr(self, "moving_shapes"):
            q0 = C.getJointState()
            X0 = {n: C.getFrame(n).getPosition() for n in C.getFrameNames() if C.getFrame(n).getAttributes().get("contact")}
            C.setJointState(q0 + .1)
            self.moving_shapes = [n for n, x in X0.items() if np.linalg.norm(C.getFrame(n).getPosition() - x) > 1e-4]
            C.setJointState(q0)
        for s in self.moving_shapes:
            komo.addObjective(times, ry.FS.negDistance, [s, table], ry.OT.ineq, [1e1], [-margin])

    def planning_config(self, keepout=.12):
        """copy of the scene for motion planning, with collision spheres around the cameras to keep the robot away"""
        if not hasattr(self, "C_plan"):
            self.C_plan = ry.Config()
            self.C_plan.addConfigurationCopy(self.C)
            for cam in self.baslers:
                self.C_plan.addFrame(f"{cam.name}_keepout", cam.name).setShape(ry.ST.sphere, [keepout]).setContact(1)
        return self.C_plan

    def render_images(self, q):
        """renderings of the scene at q for each camera (ideal pinhole camera with the camera's focal length and
        principal point at the image center, no distortion)"""
        if not hasattr(self, "C_render"):
            self.C_render = ry.Config()
            self.C_render.addConfigurationCopy(self.C)
            self.render_views = []
            for cam, f in zip(self.baslers, self.Fxycxy):
                att = cam.getAttributes()
                self.render_views.append(self.C_render.addFrame(f"{cam.name}_render", cam.name).setAttributes(
                    {"focalLength": f[1] / att["height"], "width": att["width"], "height": att["height"], "zRange": [.02, 5.]}))
            self.render_camera = ry.CameraView(self.C_render)
        self.C_render.setJointState(q)
        rgbs = []
        for view in self.render_views:
            self.render_camera.setCamera(view)
            rgbs.append(self.render_camera.computeImageAndDepth(self.C_render)[0])
        return rgbs

    def rendered_arucos(self, q):
        """for each camera, the aruco ids detected in render_images(q)"""
        if not hasattr(self, "aruco_detector"):
            self.aruco_detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50))
        ids = []
        for rgb in self.render_images(q):
            _, found, _ = self.aruco_detector.detectMarkers(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
            ids.append(set() if found is None else set(found.ravel().tolist()))
        return ids
