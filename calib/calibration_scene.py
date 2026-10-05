import numpy as np
import robotic as ry


class CalibrationScene:
    baslers = []
    basler_ids = []
    arucos = []
    aruco_ids = []
    obj = None

    Fxycxy = []
    Distortion = []

    def __init__(self, C: ry.Config, obj_name):
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

            if "aruco_id" in atts:
                self.arucos.append(f)
                self.aruco_ids.append(int(atts["aruco_id"]))

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
