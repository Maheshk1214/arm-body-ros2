#!/usr/bin/env python3
"""Add the Gazebo ground plane to MoveIt's planning scene.

The body stands on the floor at z = 0 and the hanging arms clear it by only ~2 cm, so
without this MoveIt happily plans the grippers through the floor that Gazebo collides with.
Contact between the floor and base_link (which rests on it) is allowed.
"""
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, PlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

GROUND_ID = 'ground_plane'
SIZE_XY = 4.0
THICKNESS = 0.02  # top face at z = 0, like Gazebo's ground plane
TOUCHING_LINKS = ['base_link']


def call(node, client, request, timeout=10.0):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result()


def main():
    rclpy.init()
    node = Node('add_ground_plane')
    get_scene = node.create_client(GetPlanningScene, '/get_planning_scene')
    apply_scene = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    if not (get_scene.wait_for_service(timeout_sec=60.0)
            and apply_scene.wait_for_service(timeout_sec=60.0)):
        node.get_logger().error('move_group planning scene services not available')
        sys.exit(1)

    ground = CollisionObject()
    ground.header.frame_id = 'world'
    ground.id = GROUND_ID
    ground.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                        dimensions=[SIZE_XY, SIZE_XY, THICKNESS])]
    pose = Pose()
    pose.position.z = -THICKNESS / 2.0
    pose.orientation.w = 1.0
    ground.primitive_poses = [pose]
    ground.operation = CollisionObject.ADD

    # A diff's ACM replaces the whole matrix, so extend the current one
    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    acm = call(node, get_scene, req).scene.allowed_collision_matrix
    if GROUND_ID not in acm.entry_names:
        for entry in acm.entry_values:
            entry.enabled.append(False)
        acm.entry_names.append(GROUND_ID)
        acm.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(acm.entry_names)))
    g = acm.entry_names.index(GROUND_ID)
    for link in TOUCHING_LINKS:
        i = acm.entry_names.index(link)
        acm.entry_values[i].enabled[g] = True
        acm.entry_values[g].enabled[i] = True

    scene = PlanningScene(is_diff=True)
    scene.world.collision_objects = [ground]
    scene.allowed_collision_matrix = acm
    ok = call(node, apply_scene, ApplyPlanningScene.Request(scene=scene)).success
    if ok:
        node.get_logger().info(f'Added {GROUND_ID} (top at z=0, contact allowed with '
                               f'{", ".join(TOUCHING_LINKS)}) to the planning scene')
    else:
        node.get_logger().error(f'Failed to add {GROUND_ID} to the planning scene')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
