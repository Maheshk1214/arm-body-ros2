"""
Reach a point with arm A's jaw tip: tag geometry and inverse kinematics.

Pure Python (numpy, no ROS) so it can be tested offline. Joints 2-5 of each
arm rotate about parallel axes, so an arm has 5 task-space DOF and cannot
reach an arbitrary 6-D pose. solve_reach() therefore puts the jaw tip on the
goal point (primary) and turns the jaws towards the approach direction as far
as the arm allows (secondary, weighted), with joint6 (roll) held where it is.
"""
import numpy as np

ARM = [f'arm_a_joint{i}' for i in range(1, 7)]
TIP_LINK = 'arm_a_gripper_1'
# Jaw tip centre in arm_a_gripper_1 (from the gripper meshes: jaws end at x = 113.9 mm)
TCP = np.array([0.110, 0.0, 0.0])
DIRECTION_WEIGHT = 0.05   # m of position error worth one unit of approach-direction error


def marker_object_points(size):
    """Marker corners in the marker frame (OpenCV order: TL, TR, BR, BL; z out of the tag)."""
    h = size / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], float)


# Orientation of every published tag frame, in the robot's world frame: arm_a_gripper_1's
# axes with the gripper level and pointing forward. x forward (+x, the jaws' approach),
# y straight up, z = x cross y. Only the position comes from the camera.
TAG_AXES = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])


def tag_frame(position):
    """Return the published tag pose (4x4): the detected position with the fixed TAG_AXES."""
    t = np.eye(4)
    t[:3, :3] = TAG_AXES
    t[:3, 3] = position
    return t


def tcp_pose(robot, q):
    """(jaw tip position, jaw direction) in the URDF root frame."""
    t = robot.link_pose(TIP_LINK, q)
    return t[:3, :3] @ TCP + t[:3, 3], t[:3, 0]


def tag_goal(tag_pose, standoff):
    """
    Gripper goal pose (4x4) for a published tag frame (tag_frame).

    Same orientation as the tag frame, `standoff` m back from the tag along its x axis
    (the jaws' approach direction).
    """
    goal = tag_pose.copy()
    goal[:3, 3] = tag_pose[:3, 3] - standoff * tag_pose[:3, 0]
    return goal


def rotation_angle(a, b):
    """Angle (rad) of the rotation between two 3x3 rotation matrices."""
    return float(np.arccos(np.clip((np.trace(a.T @ b) - 1.0) / 2.0, -1.0, 1.0)))


def solve_reach(robot, q_start, goal, approach, lower, upper, seeds=(),
                iterations=150, damping=1e-3, orientation=None):
    """
    Damped least-squares IK for arm A.

    q_start: current arm A joints (6). lower/upper: the 6 joint limits to stay
    within. Without `orientation`, the jaws are only turned towards `approach`
    and joint6 is kept fixed. With `orientation` (3x3, the goal's axes), the
    whole gripper frame is turned towards it, joint6 included; the arm can't
    always match it exactly (5 DOF).
    Returns (q (6,), position error m, orientation error rad) for the best of the
    seeds: the angle between jaws and `approach`, or the full rotation angle to
    `orientation`.
    """
    q_start = np.asarray(q_start, float)
    lower, upper = np.asarray(lower, float), np.asarray(upper, float)
    approach = np.asarray(approach, float) / np.linalg.norm(approach)
    free = 6 if orientation is not None else 5

    def residual(q):
        t = robot.link_pose(TIP_LINK, dict(zip(ARM, q)))
        r = [t[:3, :3] @ TCP + t[:3, 3] - goal, DIRECTION_WEIGHT * (t[:3, 0] - approach)]
        if orientation is not None:
            r.append(DIRECTION_WEIGHT * (t[:3, 1] - orientation[:, 1]))
        return np.concatenate(r)

    def error(q):
        t = robot.link_pose(TIP_LINK, dict(zip(ARM, q)))
        err = float(np.linalg.norm(t[:3, :3] @ TCP + t[:3, 3] - goal))
        if orientation is not None:
            return err, rotation_angle(t[:3, :3], orientation)
        return err, float(np.arccos(np.clip(t[:3, 0] @ approach, -1.0, 1.0)))

    best = None
    for seed in [q_start, *seeds]:
        q = np.clip(np.asarray(seed, float), lower, upper)
        if free == 5:
            q[5] = q_start[5]
        for _ in range(iterations):
            r = residual(q)
            jac = np.zeros((len(r), free))
            for j in range(free):
                dq = q.copy()
                dq[j] += 1e-6
                jac[:, j] = (residual(dq) - r) / 1e-6
            step = np.linalg.solve(jac.T @ jac + damping * np.eye(free), -jac.T @ r)
            q[:free] = np.clip(q[:free] + step, lower[:free], upper[:free])
            if np.linalg.norm(step) < 1e-7:
                break
        err, angle = error(q)
        # closer wins; within 1 mm the better-aimed one wins
        if (best is None or err < best[1] - 1e-3
                or (err < best[1] + 1e-3 and angle < best[2])):
            best = (q.copy(), err, angle)
    return best
