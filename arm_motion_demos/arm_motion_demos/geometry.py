"""
Forward kinematics over the URDF collision meshes (used by the tests).

Mesh vertices are reduced to a 1 cm voxel grid, which is plenty for checking
floor clearance and the distances between the arms and the body.
"""
import struct
import xml.etree.ElementTree as ET

import numpy as np

VOXEL = 0.01


def _rot(axis, angle):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k


def _origin(element):
    xyz, rpy = [0.0] * 3, [0.0] * 3
    if element is not None:
        xyz = [float(v) for v in (element.get('xyz') or '0 0 0').split()]
        rpy = [float(v) for v in (element.get('rpy') or '0 0 0').split()]
    t = np.eye(4)
    t[:3, :3] = _rot([0, 0, 1], rpy[2]) @ _rot([0, 1, 0], rpy[1]) @ _rot([1, 0, 0], rpy[0])
    t[:3, 3] = xyz
    return t


def _load_stl(path):
    data = open(path, 'rb').read()
    n = struct.unpack('<I', data[80:84])[0]
    if 84 + n * 50 == len(data):  # binary
        rec = np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')])
        return np.frombuffer(data[84:], dtype=rec, count=n)['v'].reshape(-1, 3).astype(float)
    return np.array([[float(x) for x in line.split()[1:4]]
                     for line in data.decode().splitlines()
                     if line.strip().startswith('vertex')])


class Robot:
    def __init__(self, urdf_xml, package_dir):
        root = ET.fromstring(urdf_xml)
        self.joints = {}
        for j in root.findall('joint'):  # not the ros2_control <joint>s
            axis = j.find('axis')
            self.joints[j.find('child').get('link')] = (
                j.get('name'), j.get('type'), j.find('parent').get('link'),
                _origin(j.find('origin')),
                [float(v) for v in axis.get('xyz').split()] if axis is not None else None)
        self.points = {}
        for link in root.findall('link'):
            col = link.find('collision')
            if col is None:
                continue
            mesh = col.find('geometry/mesh')
            path = mesh.get('filename').replace('package://arm_body_description', package_dir)
            v = _load_stl(path) * [float(s) for s in (mesh.get('scale') or '1 1 1').split()]
            v = np.unique(np.round(v / VOXEL), axis=0) * VOXEL
            t = _origin(col.find('origin'))
            self.points[link.get('name')] = v @ t[:3, :3].T + t[:3, 3]

    def _world(self, link, q, cache):
        if link not in cache:
            if link not in self.joints:
                cache[link] = np.eye(4)
            else:
                name, kind, parent, origin, axis = self.joints[link]
                t = self._world(parent, q, cache) @ origin
                if kind == 'revolute':
                    r = np.eye(4)
                    r[:3, :3] = _rot(axis, q.get(name, 0.0))
                    t = t @ r
                cache[link] = t
        return cache[link]

    def link_pose(self, link, q):
        """4x4 pose of `link` in the URDF root frame for joint values `q` (name -> rad)."""
        return self._world(link, q, {})

    def clearances(self, q):
        """(floor, arm-arm, arm_a-body, arm_b-body) minimum distances in m."""
        cache, world = {}, {}
        for link, v in self.points.items():
            t = self._world(link, q, cache)
            world[link] = v @ t[:3, :3].T + t[:3, 3]

        def arm(prefix, skip_shoulder=False):
            # joint1's rotating part sits on the body by design; skip it vs the body
            return np.vstack([v for k, v in world.items() if k.startswith(prefix)
                              and not (skip_shoulder and k in (f'{prefix}_servo_base',
                                                               f'{prefix}_merge'))])

        def dist(a, b):
            return float(np.sqrt(((a[:, None, :] - b[None, :, :]) ** 2).sum(-1)).min())

        a, b, body = arm('arm_a'), arm('arm_b'), world['base_link']
        return (float(min(a[:, 2].min(), b[:, 2].min())), dist(a, b),
                dist(arm('arm_a', True), body), dist(arm('arm_b', True), body))
