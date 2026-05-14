#!/usr/bin/env python3

import heapq
import math
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


GridCell3D = Tuple[int, int, int]
Point3D = Tuple[float, float, float]


class AstarGoalFollower(Node):
    """
    3D A* follower using occupied voxel centers from OctoMap.

    Input:
      - /octomap_point_cloud_centers : occupied voxel centers from octomap_server
      - /goal_pose                   : RViz 2D Goal Pose
      - /mavros/local_position/pose  : current vehicle pose

    Output:
      - /astar_path
      - /mavros/setpoint_position/local

    Notes:
      - This node plans in 3D.
      - Non-occupied cells inside the planning bounding box are treated as free.
      - For RViz 2D Goal Pose, z is usually 0.0, so goal_z parameter is used.
    """

    def __init__(self):
        super().__init__('astar_goal_follower')

        # Topics
        self.declare_parameter('cloud_topic', '/octomap_point_cloud_centers')
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('setpoint_topic', '/mavros/setpoint_position/local')

        # Frames
        self.declare_parameter('map_frame', 'map')

        # Execution
        self.declare_parameter('execute', False)

        # 3D planning parameters
        self.declare_parameter('grid_resolution', 0.20)
        self.declare_parameter('min_z', 0.20)
        self.declare_parameter('max_z', 2.80)
        self.declare_parameter('goal_z', 1.50)

        # Planning bounds padding
        self.declare_parameter('padding_xy', 1.0)
        self.declare_parameter('padding_z', 0.5)

        # Obstacle inflation
        self.declare_parameter('inflation_radius', 0.35)

        # Path following
        self.declare_parameter('waypoint_spacing', 0.60)
        self.declare_parameter('reach_threshold', 0.35)

        # Safety / behavior
        self.declare_parameter('hold_position_when_idle', True)

        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.setpoint_topic = self.get_parameter('setpoint_topic').value
        self.map_frame = self.get_parameter('map_frame').value

        self.execute = bool(self.get_parameter('execute').value)

        self.grid_resolution = float(self.get_parameter('grid_resolution').value)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.goal_z = float(self.get_parameter('goal_z').value)

        self.padding_xy = float(self.get_parameter('padding_xy').value)
        self.padding_z = float(self.get_parameter('padding_z').value)

        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.waypoint_spacing = float(self.get_parameter('waypoint_spacing').value)
        self.reach_threshold = float(self.get_parameter('reach_threshold').value)
        self.hold_position_when_idle = bool(self.get_parameter('hold_position_when_idle').value)

        self.occupied_points: List[Point3D] = []
        self.pose: Optional[PoseStamped] = None
        self.state: Optional[State] = None

        self.path_world: List[Point3D] = []
        self.wp_idx = 0
        self.active = False

        self.connected_time = None
        self.last_request_time = self.get_clock().now()

        # Octomap server usually publishes latched/transient data.
        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cloud_cb,
            cloud_qos,
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_cb,
            pose_qos,
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_cb,
            10,
        )

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10,
        )

        self.path_pub = self.create_publisher(Path, '/astar_path', 10)

        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            self.setpoint_topic,
            10,
        )

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.timer = self.create_timer(0.1, self.timer_cb)

        self.get_logger().info('3D A* goal follower started.')
        self.get_logger().info(f'execute={self.execute}')
        self.get_logger().info(f'cloud_topic={self.cloud_topic}')
        self.get_logger().info(f'grid_resolution={self.grid_resolution}')
        self.get_logger().info(f'z range=[{self.min_z}, {self.max_z}], default goal_z={self.goal_z}')
        self.get_logger().info('Use RViz 2D Goal Pose. Goal z uses parameter goal_z unless goal pose z > 0.05.')

    def cloud_cb(self, msg: PointCloud2):
        points: List[Point3D] = []

        for p in point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        ):
            x = float(p[0])
            y = float(p[1])
            z = float(p[2])

            if self.min_z <= z <= self.max_z:
                points.append((x, y, z))

        self.occupied_points = points

        self.get_logger().info(
            f'Received occupied voxel centers: {len(self.occupied_points)} points '
            f'after z filter [{self.min_z}, {self.max_z}]'
        )

    def pose_cb(self, msg: PoseStamped):
        self.pose = msg

    def state_cb(self, msg: State):
        self.state = msg

    def goal_cb(self, msg: PoseStamped):
        if self.pose is None:
            self.get_logger().warn('No MAVROS local pose received yet.')
            return

        if not self.occupied_points:
            self.get_logger().warn(
                f'No occupied voxel centers received from {self.cloud_topic}. '
                'Check octomap_server and /octomap_point_cloud_centers.'
            )
            return

        start = (
            float(self.pose.pose.position.x),
            float(self.pose.pose.position.y),
            self.clamp(float(self.pose.pose.position.z), self.min_z, self.max_z),
        )

        # RViz 2D Goal Pose usually sends z=0.0.
        clicked_z = float(msg.pose.position.z)
        if clicked_z > 0.05:
            goal_z = self.clamp(clicked_z, self.min_z, self.max_z)
        else:
            goal_z = self.clamp(self.goal_z, self.min_z, self.max_z)

        goal = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            goal_z,
        )

        self.get_logger().info(
            f'Planning 3D A*: start=({start[0]:.2f}, {start[1]:.2f}, {start[2]:.2f}), '
            f'goal=({goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f})'
        )

        planner = Grid3DBuilder(
            occupied_points=self.occupied_points,
            start=start,
            goal=goal,
            resolution=self.grid_resolution,
            padding_xy=self.padding_xy,
            padding_z=self.padding_z,
            inflation_radius=self.inflation_radius,
            min_z=self.min_z,
            max_z=self.max_z,
        )

        start_cell = planner.world_to_grid(start)
        goal_cell = planner.world_to_grid(goal)

        if start_cell is None:
            self.get_logger().warn('Start is outside 3D planning grid.')
            return

        if goal_cell is None:
            self.get_logger().warn('Goal is outside 3D planning grid.')
            return

        path_cells = self.astar_3d(start_cell, goal_cell, planner)

        if not path_cells:
            self.get_logger().warn('3D A* failed: no path found.')
            self.active = False
            self.path_world = []
            self.publish_path([])
            return

        dense_path = [planner.grid_to_world(c) for c in path_cells]
        self.path_world = self.sparsify_path_3d(dense_path)
        self.wp_idx = 0
        self.active = True

        self.publish_path(self.path_world)

        self.get_logger().info(
            f'3D A* path found: {len(path_cells)} cells, '
            f'{len(self.path_world)} waypoints.'
        )

        for i, p in enumerate(self.path_world):
            self.get_logger().info(
                f'  wp[{i}] = ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})'
            )

        if not self.execute:
            self.get_logger().info('execute=false, path preview only. Drone will not move.')

    def astar_3d(
        self,
        start: GridCell3D,
        goal: GridCell3D,
        planner: 'Grid3DBuilder',
    ) -> List[GridCell3D]:
        if planner.is_blocked(start):
            self.get_logger().warn('Start cell is blocked. Reduce inflation_radius or move start.')
            return []

        if planner.is_blocked(goal):
            self.get_logger().warn('Goal cell is blocked. Click a free 3D area or change goal_z.')
            return []

        neighbors = self.make_26_neighbors()

        def heuristic(a: GridCell3D, b: GridCell3D) -> float:
            return math.sqrt(
                (a[0] - b[0]) ** 2 +
                (a[1] - b[1]) ** 2 +
                (a[2] - b[2]) ** 2
            )

        open_heap = []
        heapq.heappush(open_heap, (0.0, start))

        came_from: Dict[GridCell3D, GridCell3D] = {}
        g_score: Dict[GridCell3D, float] = {start: 0.0}
        visited: Set[GridCell3D] = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current in visited:
                continue

            visited.add(current)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dx, dy, dz, move_cost in neighbors:
                nxt = (current[0] + dx, current[1] + dy, current[2] + dz)

                if not planner.in_bounds(nxt):
                    continue

                if planner.is_blocked(nxt):
                    continue

                tentative = g_score[current] + move_cost

                if tentative < g_score.get(nxt, float('inf')):
                    came_from[nxt] = current
                    g_score[nxt] = tentative
                    f = tentative + heuristic(nxt, goal)
                    heapq.heappush(open_heap, (f, nxt))

        return []

    def make_26_neighbors(self):
        neighbors = []

        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue

                    cost = math.sqrt(dx * dx + dy * dy + dz * dz)
                    neighbors.append((dx, dy, dz, cost))

        return neighbors

    def reconstruct_path(
        self,
        came_from: Dict[GridCell3D, GridCell3D],
        current: GridCell3D,
    ) -> List[GridCell3D]:
        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path

    def sparsify_path_3d(self, path: List[Point3D]) -> List[Point3D]:
        if not path:
            return []

        sparse = [path[0]]
        last = path[0]

        for p in path[1:]:
            dist = math.sqrt(
                (p[0] - last[0]) ** 2 +
                (p[1] - last[1]) ** 2 +
                (p[2] - last[2]) ** 2
            )
            if dist >= self.waypoint_spacing:
                sparse.append(p)
                last = p

        if sparse[-1] != path[-1]:
            sparse.append(path[-1])

        return sparse

    def publish_path(self, path_world: List[Point3D]):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        for x, y, z in path_world:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)

    def make_setpoint(self, x: float, y: float, z: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0

        return msg

    def request_offboard(self):
        if not self.mode_client.service_is_ready():
            return

        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = 'OFFBOARD'

        self.mode_client.call_async(req)
        self.get_logger().info('Requested OFFBOARD mode.')

    def request_arm(self):
        if not self.arm_client.service_is_ready():
            return

        req = CommandBool.Request()
        req.value = True

        self.arm_client.call_async(req)
        self.get_logger().info('Requested arm.')

    def timer_cb(self):
        if self.pose is None:
            return

        if not self.execute:
            if self.path_world:
                self.publish_path(self.path_world)
            return

        # Do not switch to OFFBOARD/ARM until a valid path is active.
        if not self.active or not self.path_world:
            if self.hold_position_when_idle:
                p = self.pose.pose.position
                self.setpoint_pub.publish(
                    self.make_setpoint(p.x, p.y, max(p.z, self.min_z))
                )
            return

        now = self.get_clock().now()

        if self.state is not None and self.state.connected:
            if self.connected_time is None:
                self.connected_time = now

            # Send setpoints for a short period before OFFBOARD/ARM.
            if (now - self.connected_time).nanoseconds * 1e-9 > 2.0:
                if (now - self.last_request_time).nanoseconds * 1e-9 > 2.0:
                    if self.state.mode != 'OFFBOARD':
                        self.request_offboard()
                        self.last_request_time = now
                    elif not self.state.armed:
                        self.request_arm()
                        self.last_request_time = now

        target_x, target_y, target_z = self.path_world[self.wp_idx]
        self.setpoint_pub.publish(self.make_setpoint(target_x, target_y, target_z))

        p = self.pose.pose.position
        dist = math.sqrt(
            (p.x - target_x) ** 2 +
            (p.y - target_y) ** 2 +
            (p.z - target_z) ** 2
        )

        if dist < self.reach_threshold:
            if self.wp_idx < len(self.path_world) - 1:
                self.wp_idx += 1
                nx, ny, nz = self.path_world[self.wp_idx]
                self.get_logger().info(
                    f'Next waypoint {self.wp_idx}/{len(self.path_world)-1}: '
                    f'({nx:.2f}, {ny:.2f}, {nz:.2f})'
                )
            else:
                self.get_logger().info('Goal reached. Holding position.')
                self.active = False

    @staticmethod
    def clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))


class Grid3DBuilder:
    def __init__(
        self,
        occupied_points: List[Point3D],
        start: Point3D,
        goal: Point3D,
        resolution: float,
        padding_xy: float,
        padding_z: float,
        inflation_radius: float,
        min_z: float,
        max_z: float,
    ):
        self.resolution = resolution
        self.min_z_limit = min_z
        self.max_z_limit = max_z

        xs = [p[0] for p in occupied_points] + [start[0], goal[0]]
        ys = [p[1] for p in occupied_points] + [start[1], goal[1]]
        zs = [p[2] for p in occupied_points] + [start[2], goal[2]]

        self.min_x = min(xs) - padding_xy
        self.max_x = max(xs) + padding_xy
        self.min_y = min(ys) - padding_xy
        self.max_y = max(ys) + padding_xy

        self.min_z = max(min(zs) - padding_z, min_z)
        self.max_z = min(max(zs) + padding_z, max_z)

        self.size_x = int(math.ceil((self.max_x - self.min_x) / resolution)) + 1
        self.size_y = int(math.ceil((self.max_y - self.min_y) / resolution)) + 1
        self.size_z = int(math.ceil((self.max_z - self.min_z) / resolution)) + 1

        self.blocked: Set[GridCell3D] = set()
        self.build_blocked_set(occupied_points, inflation_radius)

    def build_blocked_set(self, occupied_points: List[Point3D], inflation_radius: float):
        inflation_cells = max(0, int(math.ceil(inflation_radius / self.resolution)))
        r2 = inflation_cells * inflation_cells

        for p in occupied_points:
            c = self.world_to_grid(p)
            if c is None:
                continue

            cx, cy, cz = c

            for dz in range(-inflation_cells, inflation_cells + 1):
                for dy in range(-inflation_cells, inflation_cells + 1):
                    for dx in range(-inflation_cells, inflation_cells + 1):
                        if dx * dx + dy * dy + dz * dz > r2:
                            continue

                        nc = (cx + dx, cy + dy, cz + dz)
                        if self.in_bounds(nc):
                            self.blocked.add(nc)

    def world_to_grid(self, p: Point3D) -> Optional[GridCell3D]:
        x, y, z = p

        ix = int(math.floor((x - self.min_x) / self.resolution))
        iy = int(math.floor((y - self.min_y) / self.resolution))
        iz = int(math.floor((z - self.min_z) / self.resolution))

        c = (ix, iy, iz)

        if not self.in_bounds(c):
            return None

        return c

    def grid_to_world(self, c: GridCell3D) -> Point3D:
        ix, iy, iz = c

        x = self.min_x + (ix + 0.5) * self.resolution
        y = self.min_y + (iy + 0.5) * self.resolution
        z = self.min_z + (iz + 0.5) * self.resolution

        return x, y, z

    def in_bounds(self, c: GridCell3D) -> bool:
        ix, iy, iz = c

        return (
            0 <= ix < self.size_x and
            0 <= iy < self.size_y and
            0 <= iz < self.size_z
        )

    def is_blocked(self, c: GridCell3D) -> bool:
        return c in self.blocked


def main(args=None):
    rclpy.init(args=args)
    node = AstarGoalFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()