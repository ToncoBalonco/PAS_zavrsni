#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener, TransformException
import numpy as np
import heapq
import math

class AStarNavigator(Node):
    def __init__(self):
        super().__init__('astar_full_navigator')

        # Deklaracija parametara
        self.declare_parameter('cmd_vel_topic', '/diff_drive_base_controller/cmd_vel_unstamped')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # Pretplate i Publisheri
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # TF2 Listener za lokalizaciju robota
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Varijable mape
        self.map_data = None
        self.width = 0
        self.height = 0
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0

        # Navigacijske varijable
        self.path_waypoints = []
        self.current_target_idx = 0
        self.is_moving = False

        # Petlja za upravljanje kretanjem (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"A* Navigator pokrenut. S šaljem naredbe na: {cmd_vel_topic}")

    def map_callback(self, msg: OccupancyGrid):
        """Učitava i ažurira kartu iz SLAM-a."""
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        
        grid = np.array(msg.data, dtype=np.int8).reshape((self.height, self.width))
        self.map_data = self.inflate_map(grid, radius_cells=4)

    def inflate_map(self, grid, radius_cells):
        """Dodaje sigurnosnu zonu oko prepreka."""
        inflated = grid.copy()
        obstacles = np.argwhere(grid > 50)
        for r, c in obstacles:
            for dr in range(-radius_cells, radius_cells + 1):
                for dc in range(-radius_cells, radius_cells + 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < self.height and 0 <= nc < self.width:
                        inflated[nr, nc] = 100
        return inflated

    def get_robot_pose(self):
        """Dohvaća lokaciju robota iz TF stabla."""
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return x, y, yaw
        except TransformException as ex:
            self.get_logger().warn(f"Čekam TF ({self.map_frame} -> {self.base_frame}): {ex}", throttle_duration_sec=2.0)
            return None

    def world_to_grid(self, x, y):
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return row, col

    def grid_to_world(self, row, col):
        x = col * self.resolution + self.origin_x + (self.resolution / 2.0)
        y = row * self.resolution + self.origin_y + (self.resolution / 2.0)
        return x, y

    def prune_path(self, grid_path):
        """Uklanja nepotrebne točke u ravnini radi glađeg kretanja."""
        if len(grid_path) <= 2:
            return grid_path

        pruned = [grid_path[0]]
        for i in range(1, len(grid_path) - 1):
            prev_r, prev_c = pruned[-1]
            curr_r, curr_c = grid_path[i]
            next_r, next_c = grid_path[i + 1]

            # Provjera jesu li tri točke na istoj liniji (isti smjer)
            dir1 = (curr_r - prev_r, curr_c - prev_c)
            dir2 = (next_r - curr_r, next_c - curr_c)

            if dir1 != dir2:
                pruned.append(grid_path[i])

        pruned.append(grid_path[-1])
        return pruned

    def goal_callback(self, msg: PoseStamped):
        if self.map_data is None:
            self.get_logger().error("Karta još nije primljena!")
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        rx, ry, _ = robot_pose
        gx, gy = msg.pose.position.x, msg.pose.position.y

        start_node = self.world_to_grid(rx, ry)
        goal_node = self.world_to_grid(gx, gy)

        if not (0 <= start_node[0] < self.height and 0 <= start_node[1] < self.width):
            self.get_logger().error("Robot je izvan granica mape!")
            return
        if not (0 <= goal_node[0] < self.height and 0 <= goal_node[1] < self.width):
            self.get_logger().error("Cilj je izvan granica mape!")
            return

        self.get_logger().info(f"Planiram A* putanju od {start_node} do {goal_node}...")
        grid_path = self.run_astar(start_node, goal_node)

        if grid_path:
            # Redukcija redundantnih točaka
            pruned_grid_path = self.prune_path(grid_path)
            self.path_waypoints = [self.grid_to_world(r, c) for r, c in pruned_grid_path]
            self.publish_path_msg(self.path_waypoints)
            self.current_target_idx = 0
            self.is_moving = True
            self.get_logger().info(f"Putanja generirana ({len(self.path_waypoints)} prelomnih točaka). Pokrećem vožnju.")
        else:
            self.get_logger().warn("A* nije uspio pronaći putanju!")

    def run_astar(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        
        neighbors = [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for dr, dc in neighbors:
                neighbor = (current[0] + dr, current[1] + dc)
                nr, nc = neighbor

                if 0 <= nr < self.height and 0 <= nc < self.width:
                    if self.map_data[nr, nc] > 50:
                        continue
                    
                    cost = math.hypot(dr, dc)
                    tentative_g = g_score[current] + cost

                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f = tentative_g + math.hypot(nr - goal[0], nc - goal[1])
                        heapq.heappush(open_set, (f, neighbor))

        return None

    def publish_path_msg(self, waypoints):
        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def control_loop(self):
        if not self.is_moving or not self.path_waypoints:
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        rx, ry, ryaw = robot_pose
        target_x, target_y = self.path_waypoints[self.current_target_idx]

        dx = target_x - rx
        dy = target_y - ry
        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)

        angle_error = target_yaw - ryaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Prelazak na sljedeću točku
        if distance < 0.25:
            self.current_target_idx += 1
            if self.current_target_idx >= len(self.path_waypoints):
                self.get_logger().info("Cilj postignut!")
                self.stop_robot()
                self.is_moving = False
                return

        cmd = Twist()
        # Zaokret u mjestu ako je kutna greška velika
        if abs(angle_error) > 0.5:
            cmd.angular.z = 0.4 if angle_error > 0 else -0.4
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = min(0.25, 0.4 * distance)
            cmd.angular.z = 0.8 * angle_error

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AStarNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
