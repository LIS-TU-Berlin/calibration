import numpy as np
import robotic as ry


class CalibrationScene:
    baslers = []
    basler_ids = []
    realsenses = []
    realsense_ids = []
    arucos = []
    aruco_ids = []
    obj = None
    panda_base = None
    panda_ip = ""

    Fxycxy = []
    Distortion = []

    def __init__(self, C: ry.Config, obj_name: str):
        self.C = C
        self.q_home = C.getJointState()
        F = C.getFrames()

        for f in F:
            atts = f.getAttributes()
            if "basler" in atts:
                self.baslers.append(f)
                self.basler_ids.append(int(atts["basler"]))
                self.Fxycxy.append(atts["fxycxy"])
                self.Distortion.append(atts["distortion"])

            if "realsense" in atts:
                self.realsenses.append(f)
                self.realsense_ids.append(int(atts["realsense"][2:-1]))
                self.Fxycxy.append(atts["fxycxy"])
                self.Distortion.append(atts["distortion"])

            if "aruco_id" in atts:
                self.arucos.append(f)
                self.aruco_ids.append(int(atts["aruco_id"]))

            if "franka" in atts:
                self.panda_base = f
                self.panda_ip = atts["franka"]

        if obj_name is not None:
            self.obj = C.getFrame(obj_name)

        ncams = max(len(self.baslers), len(self.realsenses))
        self.imgs_collected = [0 for _ in range(ncams)]

    def make_path(self, q_current: np.ndarray, q: np.ndarray):

        self.C.setJointState(q_current)
        
        komo = ry.KOMO(self.C, 1, 32, 2, True)

        komo.addControlObjective([], 0, 1e-1)
        komo.addControlObjective([], 1, 1e-1)
        komo.addControlObjective([], 2, 1e-1)
        
        komo.addObjective([1.0], ry.FS.qItself, [], ry.OT.sos, [1.0], q)
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, [1e1])
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
        komo.addObjective([1.0], ry.FS.qItself, [], ry.OT.eq, [1.0], 0., 1)

        sol = ry.NLP_Solver()
        sol.setProblem(komo.nlp())
        ret = sol.solve()

        path = komo.getPath()
        # if ret.feasible:
        #     komo.view(True, "optimized path")

        return path, ret.feasible

    def get_random_pose(self):

        self.C.setJointState(self.q_home)

        komo = ry.KOMO(self.C, 1, 1, 0, True)

        # pick cam
        cam_idx = np.argmin(self.imgs_collected)

        if self.basler_ids:
            cam = self.baslers[cam_idx]
            dist = 0.1
            offset_mag = 0.1
        
        elif self.realsense_ids:
            cam = self.realsenses[cam_idx]
            dist = 0.5
            offset_mag = 0.01
        
        else: Exception("No camera hardware specified!")
        
        # pick random aruco
        aruco = np.random.choice(self.arucos)

        offset = offset_mag * np.random.randn(3)
        
        # komo.addControlObjective([1.0], 0, 1e-1)  # Dont make the poses too crazy
        komo.addObjective([1.0], ry.FS.positionRel, [aruco.name, cam.name], ry.OT.sos, [1e1, 1e1, 0.0], offset)
        komo.addObjective([1.0], ry.FS.positionRel, [cam.name, aruco.name], ry.OT.sos, [1.0, 0.0, 1.0])
        komo.addObjective([1.0], ry.FS.positionRel, [cam.name, aruco.name], ry.OT.ineq, [0.0, 0.0, -1.0], [dist])
        komo.addObjective([1.0], ry.FS.scalarProductZZ, [cam.name, aruco.name], ry.OT.ineq, [1.0], [-0.95])
        komo.addObjective([1.0], ry.FS.accumulatedCollisions, [], ry.OT.eq, [1e1])
        komo.addObjective([1.0], ry.FS.jointLimits, [], ry.OT.ineq)

        sol = ry.NLP_Solver()
        sol.setProblem(komo.nlp())
        ret = sol.solve()
        print("===", cam.name, offset, "\n", ret)

        q = komo.getPath().reshape(-1)
        if ret.feasible:
            print(f"Paired camera '{cam.name}' with aruco '{aruco.name}'")
            # komo.view(True, "optimized")
            self.imgs_collected[cam_idx] += 1
            print("Selected cameras counts: ", self.imgs_collected)
            return q
        else:
            print("-- infeasible!")
            print(f"Failed to pair camera '{cam.name}' with aruco '{aruco.name}'")
            return self.get_random_pose()
