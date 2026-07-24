from __future__ import annotations

import os
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin
import casadi


class PiperIK:
    """Headless single-arm Piper IK solver.

    No viewer, CAN device, ROS1 package, or hardware is touched here.
    """

    def __init__(self, urdf_path: str):
        if not os.path.isfile(urdf_path):
            raise FileNotFoundError(urdf_path)
        self.robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path, package_dirs=os.path.dirname(urdf_path)
        )
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=["joint7", "joint8"],
            reference_configuration=np.zeros(self.robot.model.nq),
        )
        first = pin.SE3(np.eye(3), np.zeros(3))
        first.rotation = pin.rpy.rpyToMatrix(0, -1.57, 0)
        last = first * pin.SE3(np.eye(3), np.array([0.13, 0.0, 0.0]))
        self.reduced_robot.model.addFrame(
            pin.Frame("ee", self.reduced_robot.model.getJointId("joint6"), last,
                      pin.FrameType.OP_FRAME)
        )
        # addFrame changes the model layout; rebuild data before any FK/IK call.
        self.reduced_robot.data = self.reduced_robot.model.createData()

        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        self.ctf = casadi.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)
        self.ee_id = self.reduced_robot.model.getFrameId("ee")
        error = cpin.log6(
            self.cdata.oMf[self.ee_id].inverse() * cpin.SE3(self.ctf)
        ).vector
        error_fun = casadi.Function("piper_pose_error", [self.cq, self.ctf], [error])
        opti = casadi.Opti()
        self.var_q = opti.variable(self.reduced_robot.model.nq)
        self.param_tf = opti.parameter(4, 4)
        error_vec = error_fun(self.var_q, self.param_tf)
        opti.subject_to(opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit,
        ))
        opti.minimize(20 * casadi.sumsqr(error_vec[:3]) +
                      0.1 * casadi.sumsqr(error_vec[3:]) +
                      0.01 * casadi.sumsqr(self.var_q))
        opti.solver("ipopt", {"ipopt": {"print_level": 0, "max_iter": 50, "tol": 1e-4},
                               "print_time": False})
        self.opti = opti
        self.last_q = np.zeros(self.reduced_robot.model.nq)

    def forward_ee_pose(self, joints_rad):
        q = np.asarray(joints_rad, dtype=float).reshape(-1)
        if q.size != 6 or not np.all(np.isfinite(q)):
            raise ValueError("expected six finite joint positions")
        pin.framesForwardKinematics(self.reduced_robot.model, self.reduced_robot.data, q)
        return self.reduced_robot.data.oMf[self.ee_id].homogeneous.copy()

    def solve_with_status(self, target: np.ndarray):
        target = np.asarray(target, dtype=float)
        if target.shape != (4, 4) or not np.all(np.isfinite(target)):
            return None, "invalid_target"
        self.opti.set_initial(self.var_q, self.last_q)
        self.opti.set_value(self.param_tf, target)
        try:
            solution = self.opti.solve_limited()
            q = np.asarray(self.opti.value(self.var_q), dtype=float).reshape(-1)
        except Exception:
            return None, "solver_failed"
        if q.size != 6 or not np.all(np.isfinite(q)):
            return None, "solver_failed"
        if np.max(np.abs(q - self.last_q)) > np.deg2rad(45):
            return None, "joint_jump_rejected"
        self.last_q = q
        return q, "ok"

    def solve(self, target: np.ndarray):
        q, status = self.solve_with_status(target)
        return q if status == "ok" else None
