"""
Find an ArUco tag with arm A's gripper camera and move the jaw tip in front of it.

1. Move arm A to `look_pose` (skipped with an empty list) so the camera faces forward.
2. Detect the tag in `samples` camera frames while the arm is still, estimate its pose
   (solvePnP, IPPE_SQUARE) and express it in `base_frame` through TF.
   Only the position is used: the tag is taken as facing the robot straight on, with the
   fixed orientation of reach.tag_frame (x forward, y up), whatever its real tilt.
3. Goal = `standoff` m back from the tag along x, same orientation. Solve IK for the jaw
   tip on the goal with the gripper turned to that orientation as far as the 5-DOF arm
   allows (exact pitch and roll; yaw is tied to where the goal is).
4. Plan to that joint goal with MoveIt (collision-checked) and execute it, unless dry_run.

Every camera frame is searched and published annotated on /arm_a/camera/aruco_debug.
For RViz: TF frames aruco_tag (live while looking, then frozen at the measured position)
and aruco_reach_goal, and markers on /aruco_reach/markers
(tag, goal, approach arrow, camera line of sight). With stay:=true (default) the node keeps
showing the live detection after the move, until Ctrl-C.

Exit codes: 0 reached (or dry run done), 1 failed while moving, 2 refused before moving.
"""
import sys
import time

from ament_index_python.packages import get_package_share_directory
from arm_motion_demos import reach
from arm_motion_demos.executor import load_robot
from arm_motion_demos.geometry import Robot
from arm_motion_demos.motions import INDEX
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, TransformStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import xacro
import yaml

# the SRDF's aruco_look, taught on the real arm A
# (was elbow_90: [0.0, -0.01, 0.0, -1.55, 1.55, 1.5708])
LOOK_POSE = [0.009, -3.019, 0.143, 0.982, 2.109, 2.022]
STILL_VELOCITY = 0.02      # rad/s, arm considered still for detection
FALLBACK_SPEED = 0.025     # MoveIt scaling if joint_limits.yaml has no main speed
# IK seeds besides the current pose: elbow configurations across the workspace
SEEDS = [[0.0, -1.2, 0.3, 0.3, 0.6, 0.0], [0.0, -0.5, -0.5, 0.0, 1.0, 0.0],
         [0.0, -1.5, 0.5, 0.5, 0.5, 0.0], [0.0, -2.0, 0.8, 0.9, 0.3, 0.0],
         [0.0, -0.9, 0.0, -0.4, 1.4, 0.0], [0.0, -2.4, 1.2, 1.0, 0.2, 0.0]]


class Refused(Exception):
    """Stopped before moving to the tag."""


def _matrix(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = [t.x, t.y, t.z]
    return m


def _quaternion(r):
    """(x, y, z, w) of a rotation matrix."""
    w = np.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return x, y, z, w


class ArucoDetector:
    """OpenCV ArUco detection for both the 4.6 (functions) and >= 4.7 (class) APIs."""

    def __init__(self, dictionary):
        aruco = cv2.aruco
        self.dictionary = aruco.getPredefinedDictionary(getattr(aruco, dictionary))
        if hasattr(aruco, 'ArucoDetector'):
            self.detector = aruco.ArucoDetector(self.dictionary, aruco.DetectorParameters())
        else:
            self.detector = None
            self.params = aruco.DetectorParameters_create()

    def detect(self, gray):
        if self.detector:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary,
                                                      parameters=self.params)
        return corners, ([] if ids is None else ids.flatten().tolist())


class ArucoReach(Node):
    def __init__(self):
        super().__init__('aruco_reach')
        p = self.declare_parameter
        self.marker_id = p('marker_id', 0).value
        self.marker_size = p('marker_size', 0.05).value
        self.standoff = p('standoff', 0.0).value  # m; 0 = jaw tip on the tag
        self.samples = p('samples', 10).value
        self.detect_timeout = p('detect_timeout', 60.0).value
        self.look_pose = list(p('look_pose', LOOK_POSE).value)
        self.base_frame = p('base_frame', 'world').value
        self.camera_frame = p('camera_frame', 'arm_a_camera_optical_frame').value
        self.max_error = p('max_position_error', 0.01).value
        # rad, per joint: how close the arm must stop to a target; real servos sag under load
        self.settle_tolerance = p('settle_tolerance', 0.05).value
        # 0 = MoveIt's main speed (default_*_scaling_factor in joint_limits.yaml), the same
        # as RViz's Plan & Execute; any other value is used as is
        self.speed = p('speed', 0.0).value
        self.dry_run = p('dry_run', False).value
        self.stay = p('stay', True).value
        # true for an arm without a joint6 servo: IK keeps the wrist roll where it is
        self.hold_roll = p('hold_roll', False).value
        image_topic = p('image_topic', '/arm_a/camera/image_raw').value
        info_topic = p('camera_info_topic', '/arm_a/camera/camera_info').value
        self.detector = ArucoDetector(p('dictionary', 'DICT_4X4_50').value)

        self.bridge = CvBridge()
        self.camera = None    # (K, D)
        self.frames = 0       # camera frames received
        self.detection = None  # (frame number, tag pose in the camera frame, still)
        self.status = 'starting'
        self.last_markers = None  # (tag, goal, camera) last published to RViz
        self.measured_tag = None  # once set, aruco_tag stays here (the camera moves after)
        self.stamp = None         # latest /joint_states stamp: sim time in Gazebo
        self.joints = {}
        self.velocity = 0.0
        self.create_subscription(Image, image_topic, self._on_image, 2)
        self.create_subscription(CameraInfo, info_topic, self._on_info, 2)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 20)
        self.debug_pub = self.create_publisher(Image, '/arm_a/camera/aruco_debug', 2)
        self.marker_pub = self.create_publisher(MarkerArray, '/aruco_reach/markers', 2)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static = StaticTransformBroadcaster(self)
        self.tf_dynamic = TransformBroadcaster(self)
        self.create_timer(0.2, self._publish_measured_tag)
        self.move_group = ActionClient(self, MoveGroup, '/move_action')

        desc = get_package_share_directory('arm_body_description')
        urdf = xacro.process_file(f'{desc}/urdf/arm_body.urdf.xacro').toxml()
        self.robot = Robot(urdf, desc)
        limits, _ = load_robot()
        moveit = get_package_share_directory('arm_body_moveit_config')
        with open(f'{moveit}/config/joint_limits.yaml') as f:
            main = yaml.safe_load(f)
        # FALLBACK_SPEED only if joint_limits.yaml has no default_*_scaling_factor
        self.velocity_scaling = self.speed or main.get('default_velocity_scaling_factor',
                                                       FALLBACK_SPEED)
        self.acceleration_scaling = self.speed or main.get(
            'default_acceleration_scaling_factor', FALLBACK_SPEED)
        source = '' if self.speed else ' (main speed in joint_limits.yaml)'
        self.get_logger().info(f'speed: velocity x{self.velocity_scaling:.3f}, acceleration '
                               f'x{self.acceleration_scaling:.3f}{source}')
        idx = [INDEX[j] for j in reach.ARM]
        self.lower, self.upper = limits.safe_lower[idx], limits.safe_upper[idx]

    # --- inputs ------------------------------------------------------------------
    def _on_info(self, msg):
        self.camera = (np.array(msg.k, float).reshape(3, 3), np.array(msg.d, float))

    def _on_joints(self, msg):
        self.stamp = msg.header.stamp
        self.joints.update(zip(msg.name, msg.position))
        if msg.velocity:
            vel = dict(zip(msg.name, msg.velocity))
            self.velocity = max(abs(vel.get(j, 0.0)) for j in reach.ARM)

    def _on_image(self, msg):
        """Search every frame for the tag and publish it annotated."""
        self.frames += 1
        if self.camera is None:
            return
        k, d = self.camera
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        corners, ids = self.detector.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if ids:
            cv2.aruco.drawDetectedMarkers(frame, corners, np.array(ids).reshape(-1, 1))
        text, colour = f'marker {self.marker_id} not seen', (0, 0, 255)
        if self.marker_id in ids:
            img_pts = corners[ids.index(self.marker_id)].reshape(4, 2)
            ok, rvec, tvec = cv2.solvePnP(reach.marker_object_points(self.marker_size),
                                          img_pts, k, d, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                cv2.drawFrameAxes(frame, k, d, rvec, tvec, self.marker_size * 0.75)
                tag_cam = np.eye(4)
                tag_cam[:3, :3] = cv2.Rodrigues(rvec)[0]
                tag_cam[:3, 3] = tvec.flatten()
                still = self.velocity < STILL_VELOCITY
                self.detection = (self.frames, tag_cam, still)
                if still and self.measured_tag is None:
                    self._publish_live_tag(tag_cam)
                x, y, z = tvec.flatten()
                text, colour = (f'marker {self.marker_id}: {z * 100:.1f} cm ahead, '
                                f'{x * 100:+.1f} right, {-y * 100:+.1f} up'), (0, 200, 0)
        for i, (line, c) in enumerate(((text, colour), (self.status, (255, 255, 255)))):
            org = (10, 25 + 25 * i)
            cv2.putText(frame, line, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
            cv2.putText(frame, line, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 1)
        out = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        out.header = msg.header
        self.debug_pub.publish(out)

    def _transform(self, name, m):
        """Build the base_frame -> name transform for pose m, stamped like /joint_states."""
        tf = TransformStamped()
        tf.header.stamp = self.stamp or self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = self.base_frame, name
        tf.transform.translation.x, tf.transform.translation.y, \
            tf.transform.translation.z = (float(v) for v in m[:3, 3])
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = (float(v) for v in _quaternion(m[:3, :3]))
        return tf

    def _publish_live_tag(self, tag_cam):
        """Show aruco_tag where the camera sees it now (while looking)."""
        try:
            cam = _matrix(self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, Time()))
        except Exception:  # noqa: B902 (tf2 raises several unrelated types; next frame)
            return
        tag = reach.tag_frame(cam[:3, :3] @ tag_cam[:3, 3] + cam[:3, 3])
        self.tf_dynamic.sendTransform(self._transform('aruco_tag', tag))

    def _publish_measured_tag(self):
        if self.measured_tag is not None:
            self.tf_dynamic.sendTransform(self._transform('aruco_tag', self.measured_tag))

    def arm(self):
        return np.array([self.joints[j] for j in reach.ARM])

    def say(self, status):
        self.status = status
        self.get_logger().info(status)

    def spin_until(self, ready, timeout, what):
        end = time.monotonic() + timeout
        while not ready():
            if time.monotonic() > end:
                raise Refused(f'timed out waiting for {what}')
            rclpy.spin_once(self, timeout_sec=0.05)

    # --- steps ---------------------------------------------------------------------
    def wait_for_inputs(self):
        self.spin_until(lambda: all(j in self.joints for j in reach.ARM), 15, '/joint_states')
        self.spin_until(lambda: self.camera is not None and self.frames > 0, 15,
                        'camera image and camera_info (is the Gazebo camera bridge or '
                        'camera_stream running?)')
        self.spin_until(lambda: self.move_group.wait_for_server(timeout_sec=0.1), 30,
                        'MoveIt (/move_action)')

    def move(self, name, q):
        """Plan and execute arm A to joint values q with MoveIt; raises on failure."""
        req = MotionPlanRequest(group_name='arm_a', num_planning_attempts=5,
                                allowed_planning_time=5.0,
                                max_velocity_scaling_factor=self.velocity_scaling,
                                max_acceleration_scaling_factor=self.acceleration_scaling)
        req.start_state.is_diff = True
        req.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=float(v), tolerance_above=1e-3,
                            tolerance_below=1e-3, weight=1.0)
            for j, v in zip(reach.ARM, q)])]
        goal = MoveGroup.Goal(request=req)
        goal.planning_options.plan_only = self.dry_run
        self.say(f'[{name}] {"planning" if self.dry_run else "planning + moving"} to '
                 f'{np.round(q, 2).tolist()}')
        future = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError(f'{name}: MoveIt rejected the goal')
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, future)
        result = future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'{name}: MoveIt error code {result.error_code.val}')
        points = result.planned_trajectory.joint_trajectory.points
        self.get_logger().info(f'[{name}] path found: {len(points)} waypoints, '
                               f'{points[-1].time_from_start.sec if points else 0} s')
        if not self.dry_run:  # the Gazebo controllers report success without checking
            self.spin_until(lambda: np.abs(self.arm() - q).max() < self.settle_tolerance
                            and self.velocity < STILL_VELOCITY, 10, f'{name}: arm to settle')

    def detect_tag(self):
        """Tag frame (reach.tag_frame) at the mean position over `samples` still frames."""
        self.say(f'looking for marker {self.marker_id}... (show it to the camera)')
        positions, last = [], self.frames
        end = time.monotonic() + self.detect_timeout
        while len(positions) < self.samples:
            if time.monotonic() > end:
                raise Refused(f'marker {self.marker_id} not seen in {self.detect_timeout:.0f} s')
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.detection is None or self.detection[0] <= last or not self.detection[2]:
                continue
            last, tag_cam, _ = self.detection
            try:  # the arm is still, so the latest camera pose is the one for this image
                cam = _matrix(self.tf_buffer.lookup_transform(
                    self.base_frame, self.camera_frame, Time()))
            except Exception as e:  # noqa: B902 (tf2 raises several unrelated types)
                self.get_logger().warn(f'TF {self.base_frame} -> {self.camera_frame}: {e}')
                continue
            positions.append(cam[:3, :3] @ tag_cam[:3, 3] + cam[:3, 3])
            self.status = f'measuring marker {self.marker_id}: {len(positions)}/{self.samples}'
        p = np.mean(positions, axis=0)
        spread = np.max(np.linalg.norm(np.array(positions) - p, axis=1))
        self.get_logger().info(f'tag in {self.base_frame}: {np.round(p, 4).tolist()} '
                               f'({len(positions)} frames, spread {spread * 1000:.1f} mm)')
        return reach.tag_frame(p), cam[:3, 3]

    def publish_frames(self, tag, goal_pose):
        """Freeze aruco_tag at the measured tag and publish the goal frame."""
        self.measured_tag = tag
        self._publish_measured_tag()
        self.tf_static.sendTransform(self._transform('aruco_reach_goal', goal_pose))

    def publish_markers(self, tag, goal, camera):
        """Tag card, goal sphere, approach arrow and camera line of sight for RViz."""
        self.last_markers = (tag, goal, camera)

        def marker(i, kind, rgba, scale):
            m = Marker(id=i, type=kind, action=Marker.ADD, ns='aruco_reach')
            m.header.frame_id = self.base_frame
            m.scale.x, m.scale.y, m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            m.pose.orientation.w = 1.0
            return m

        def pt(v):
            return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))

        # the tag as it is taken: facing the robot straight on (its x axis is the normal)
        card = marker(0, Marker.CUBE, (1.0, 1.0, 1.0, 0.9),
                      (0.002, self.marker_size * 1.5, self.marker_size * 1.5))
        card.pose.position = pt(tag[:3, 3])
        q = _quaternion(tag[:3, :3])
        card.pose.orientation.x, card.pose.orientation.y, card.pose.orientation.z, \
            card.pose.orientation.w = q
        square = marker(1, Marker.CUBE, (0.0, 0.0, 0.0, 1.0),
                        (0.003, self.marker_size, self.marker_size))
        square.pose = card.pose
        target = marker(2, Marker.SPHERE, (0.1, 0.8, 0.2, 0.8), (0.02, 0.02, 0.02))
        target.pose.position = pt(goal)
        arrow = marker(3, Marker.ARROW, (0.1, 0.8, 0.2, 1.0), (0.006, 0.012, 0.015))
        arrow.points = [pt(goal), pt(tag[:3, 3])]
        sight = marker(4, Marker.LINE_LIST, (1.0, 0.5, 0.0, 0.8), (0.003, 0.0, 0.0))
        sight.points = [pt(camera), pt(tag[:3, 3])]
        label = marker(5, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0), (0.0, 0.0, 0.025))
        label.pose.position = pt(tag[:3, 3] + [0, 0, 0.06])
        label.text = f'ArUco {self.marker_id}'
        self.marker_pub.publish(MarkerArray(markers=[card, square, target, arrow, sight,
                                                     label]))

    def run(self):
        log = self.get_logger()
        self.wait_for_inputs()
        if self.look_pose:
            look = np.array(self.look_pose, float)
            if not ((look >= self.lower) & (look <= self.upper)).all():
                raise Refused(f'look_pose {self.look_pose} is outside the joint limits')
            self.move('look', look)
        if self.dry_run and self.look_pose:
            log.info('dry_run: detecting from the current pose, not the look pose')
        tag, camera = self.detect_tag()
        goal_pose = reach.tag_goal(tag, self.standoff)
        goal = goal_pose[:3, 3]
        self.publish_frames(tag, goal_pose)
        self.publish_markers(tag, goal, camera)
        q, err, angle = reach.solve_reach(self.robot, self.arm(), goal, goal_pose[:3, 0],
                                          self.lower, self.upper,
                                          seeds=[[*s[:5], self.arm()[5]] for s in SEEDS],
                                          orientation=None if self.hold_roll
                                          else goal_pose[:3, :3])
        log.info(f'goal {np.round(goal, 4).tolist()} ({self.standoff:.3f} m in front of the '
                 f'tag): IK error {err * 1000:.1f} mm, gripper {np.degrees(angle):.1f} deg off '
                 f'the tag orientation')
        if err > self.max_error:
            raise Refused(f'goal is out of reach: best IK solution is {err * 1000:.0f} mm '
                          f'away (max_position_error {self.max_error * 1000:.0f} mm). '
                          f'Bring the tag closer to the camera or increase standoff.')
        self.move('reach', q)
        if not self.dry_run:
            tip, _ = reach.tcp_pose(self.robot, dict(zip(reach.ARM, self.arm())))
            self.say(f'reached: jaw tip {np.linalg.norm(tip - goal) * 1000:.1f} mm from the '
                     f'goal, {np.linalg.norm(tip - tag[:3, 3]) * 1000:.0f} mm from the tag')
        else:
            self.say('dry run: path planned, not executed')

    def linger(self, markers):
        """Keep the live camera view and the RViz markers up until Ctrl-C."""
        self.get_logger().info('done; still showing the camera view (Ctrl-C to quit)')
        last = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if markers and time.monotonic() - last > 1.0:
                self.publish_markers(*markers)
                last = time.monotonic()


def main():
    rclpy.init()
    node = ArucoReach()
    code = 0
    try:
        node.run()
    except Refused as e:
        node.say(f'refused: {e}')
        code = 2
    except RuntimeError as e:
        node.say(str(e))
        code = 1
    except KeyboardInterrupt:
        code = 130
    try:
        if node.stay and code != 130:
            node.linger(node.last_markers)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
