"""
Live ArUco tag TF from arm A's camera, for watching detections in RViz.

Every camera frame is searched for `marker_id`; when found, the tag is broadcast as TF
`base_frame` -> `tag_frame` (default world -> aruco_tag) and the annotated image is
published on /arm_a/camera/aruco_debug. Nothing moves.
Only the position comes from the camera; the orientation is fixed (reach.tag_frame):
x forward, y straight up, z flat.
"""
from arm_motion_demos import reach
from arm_motion_demos.aruco_reach import _matrix, _quaternion, ArucoDetector
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


class ArucoTf(Node):
    def __init__(self):
        super().__init__('aruco_tf')
        p = self.declare_parameter
        self.marker_id = p('marker_id', 0).value
        self.marker_size = p('marker_size', 0.05).value
        self.camera_frame = p('camera_frame', 'arm_a_camera_optical_frame').value
        self.tag_frame = p('tag_frame', 'aruco_tag').value
        self.base_frame = p('base_frame', 'world').value
        self.detector = ArucoDetector(p('dictionary', 'DICT_4X4_50').value)
        self.bridge = CvBridge()
        self.camera = None
        self.seen = None   # last logged state, to log only changes
        self.tf = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.debug_pub = self.create_publisher(Image, '/arm_a/camera/aruco_debug', 2)
        self.create_subscription(CameraInfo, '/arm_a/camera/camera_info', self._on_info, 2)
        self.create_subscription(Image, '/arm_a/camera/image_raw', self._on_image, 2)

    def _on_info(self, msg):
        self.camera = (np.array(msg.k, float).reshape(3, 3), np.array(msg.d, float))

    def _on_image(self, msg):
        if self.camera is None:
            return
        k, d = self.camera
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        corners, ids = self.detector.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if ids:
            cv2.aruco.drawDetectedMarkers(frame, corners, np.array(ids).reshape(-1, 1))
        text, colour = f'marker {self.marker_id} not seen', (0, 0, 255)
        if ids:
            text += f' (seeing ids {sorted(ids)}: set marker_id)'
        found = self.marker_id in ids
        if found:
            ok, rvec, tvec = cv2.solvePnP(reach.marker_object_points(self.marker_size),
                                          corners[ids.index(self.marker_id)].reshape(4, 2),
                                          k, d, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            found = ok
        if found:
            cv2.drawFrameAxes(frame, k, d, rvec, tvec, self.marker_size * 0.75)
            x, y, z = tvec.flatten()
            text, colour = (f'marker {self.marker_id}: {z * 100:.1f} cm ahead, '
                            f'{x * 100:+.1f} right, {-y * 100:+.1f} up'), (0, 200, 0)
            try:  # latest camera pose: the camera is still while the tag is shown
                cam = _matrix(self.tf_buffer.lookup_transform(
                    self.base_frame, self.camera_frame, Time()))
            except Exception as e:  # noqa: B902 (tf2 raises several unrelated types)
                self.get_logger().warn(f'TF {self.base_frame} -> {self.camera_frame}: {e}',
                                       throttle_duration_sec=5.0)
                cam = None
            if cam is not None:
                tag = reach.tag_frame(cam[:3, :3] @ tvec.flatten() + cam[:3, 3])
                tf = TransformStamped()
                tf.header.stamp = msg.header.stamp
                tf.header.frame_id, tf.child_frame_id = self.base_frame, self.tag_frame
                tf.transform.translation.x, tf.transform.translation.y, \
                    tf.transform.translation.z = (float(v) for v in tag[:3, 3])
                r = tf.transform.rotation
                r.x, r.y, r.z, r.w = (float(v) for v in _quaternion(tag[:3, :3]))
                self.tf.sendTransform(tf)
        if found != self.seen:
            self.get_logger().info(text if found else f'marker {self.marker_id} lost' + (
                f', seeing ids {sorted(ids)}' if ids else ''))
            self.seen = found
        cv2.putText(frame, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(frame, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 1)
        out = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        out.header = msg.header
        self.debug_pub.publish(out)


def main():
    rclpy.init()
    node = ArucoTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
