"""
Publish a real camera (ESP32 OV3660 MJPEG stream, or a local device) as arm A's camera.

Publishes /arm_a/camera/image_raw and /arm_a/camera/camera_info in the URDF's
arm_a_camera_optical_frame, the same topics as the Gazebo camera, so aruco_reach
works unchanged with a real camera (and a simulated or real arm).

Intrinsics: `calibration_file` (a ROS camera_calibration / camera_info_manager YAML) if
given, otherwise a pinhole estimate from `horizontal_fov` with no distortion. The estimate
is fine for a first test; calibrate before trusting distances to the centimetre.

`rotation` (0, 90, 180, 270 deg clockwise) turns the image upright when the camera is
mounted on its side or upside down; the intrinsics are rotated with it, so calibrate the
camera unrotated.
"""
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
import yaml


ROTATIONS = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
             270: cv2.ROTATE_90_COUNTERCLOCKWISE}


def rotate_intrinsics(k, d, width, height, rotation):
    """K and D of an image of width x height after cv2.rotate by `rotation` deg clockwise."""
    fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
    d = np.array(d, float)
    p1, p2 = (d[2], d[3]) if len(d) >= 4 else (0.0, 0.0)
    if rotation == 90:      # (u, v) -> (h-1-v, u)
        fx, fy, cx, cy, p1, p2 = fy, fx, height - 1 - cy, cx, p2, -p1
    elif rotation == 180:   # (u, v) -> (w-1-u, h-1-v)
        cx, cy, p1, p2 = width - 1 - cx, height - 1 - cy, -p1, -p2
    elif rotation == 270:   # (u, v) -> (v, w-1-u)
        fx, fy, cx, cy, p1, p2 = fy, fx, cy, width - 1 - cx, -p2, p1
    if len(d) >= 4:
        d[2], d[3] = p1, p2
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]), d


def load_calibration(path):
    """(K (3x3), D, width, height) from a ROS camera calibration YAML."""
    with open(path) as f:
        c = yaml.safe_load(f)
    k = np.array(c['camera_matrix']['data'], float).reshape(3, 3)
    d = np.array(c['distortion_coefficients']['data'], float)
    return k, d, int(c['image_width']), int(c['image_height'])


class CameraStream(Node):
    def __init__(self):
        super().__init__('camera_stream')
        p = self.declare_parameter
        self.source = p('source', 'http://192.168.4.1:81/stream').value
        self.frame_id = p('frame_id', 'arm_a_camera_optical_frame').value
        self.hfov = p('horizontal_fov', 0.96).value
        self.max_rate = p('max_rate', 15.0).value   # Hz; also paces video files
        self.rotation = p('rotation', 0).value % 360
        if p('rotate_180', False).value:  # older name for rotation:=180
            self.rotation = (self.rotation + 180) % 360
        if self.rotation not in (0, *ROTATIONS):
            raise ValueError(f'rotation must be 0, 90, 180 or 270, not {self.rotation}')
        # e.g. MJPG for a USB camera whose default raw format is too slow for its link
        self.fourcc = p('fourcc', '').value
        calibration = p('calibration_file', '').value
        self.calibration = load_calibration(calibration) if calibration else None
        self.image_pub = self.create_publisher(Image, '/arm_a/camera/image_raw', 2)
        self.info_pub = self.create_publisher(CameraInfo, '/arm_a/camera/camera_info', 2)
        self.bridge = CvBridge()
        self.capture = None

    def connect(self):
        if self.source.isdigit():
            self.capture = cv2.VideoCapture(int(self.source), cv2.CAP_V4L2)
            if self.fourcc:
                self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        else:  # FFmpeg handles MJPEG over HTTP and files; GStreamer can crash on them
            self.capture = cv2.VideoCapture(self.source, cv2.CAP_FFMPEG)
        if not self.capture.isOpened():
            self.get_logger().warn(f'cannot open {self.source}, retrying in 2 s')
            self.capture = None
            return False
        self.get_logger().info(f'streaming {self.source}')
        return True

    def camera_info(self, width, height):
        """Camera info for the published (rotated) image of a width x height camera frame."""
        if self.calibration:
            k, d, cw, ch = self.calibration
            if (cw, ch) != (width, height):  # same lens, other resolution: scale K
                k = k * np.array([[width / cw], [height / ch], [1.0]])
        else:
            f = width / 2.0 / np.tan(self.hfov / 2.0)
            k = np.array([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1]])
            d = np.zeros(5)
        k, d = rotate_intrinsics(k, d, width, height, self.rotation)
        if self.rotation in (90, 270):
            width, height = height, width
        info = CameraInfo(width=width, height=height, distortion_model='plumb_bob')
        info.k = [float(v) for v in k.flatten()]
        info.d = [float(v) for v in d]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [float(k[0, 0]), 0.0, float(k[0, 2]), 0.0,
                  0.0, float(k[1, 1]), float(k[1, 2]), 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def run(self):
        warned, last = False, 0.0
        while rclpy.ok():
            time.sleep(max(0.0, last + 1.0 / self.max_rate - time.monotonic()))
            last = time.monotonic()
            if self.capture is None and not self.connect():
                time.sleep(2.0)
                continue
            ok, frame = self.capture.read()
            if not ok:
                self.get_logger().warn('stream stopped, reconnecting')
                self.capture.release()
                self.capture = None
                continue
            info = self.camera_info(frame.shape[1], frame.shape[0])
            if self.rotation:
                frame = cv2.rotate(frame, ROTATIONS[self.rotation])
            if not warned and self.calibration is None:
                self.get_logger().warn(
                    f'no calibration_file: using a {np.degrees(self.hfov):.0f} deg '
                    f'horizontal FOV estimate, so distances may be off by ~10 %')
                warned = True
            stamp = self.get_clock().now().to_msg()
            image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            image.header.stamp = info.header.stamp = stamp
            image.header.frame_id = info.header.frame_id = self.frame_id
            self.image_pub.publish(image)
            self.info_pub.publish(info)
            rclpy.spin_once(self, timeout_sec=0)


def main():
    rclpy.init()
    node = CameraStream()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
