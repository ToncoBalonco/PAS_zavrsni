#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

import tf2_ros


class PurePursuitFollower(Node):
    """
    Pure Pursuit kontroler koji sluša publishanu putanju (/plan)
    i šalje brzinske naredbe.

    Ako se lookahead točka nalazi iza robota, robot se prvo
    okreće u mjestu prema toj točki. Kada je dovoljno poravnat,
    nastavlja s normalnim Pure Pursuit praćenjem.

    Topici:
        Pretplate:
            /plan             (nav_msgs/Path)
        TF:
            map -> odom -> base_footprint
        Publicira:
            /cmd_vel_nav      (geometry_msgs/Twist)
    """

    def __init__(self):
        super().__init__('pure_pursuit_follower')

        # =====================================================
        # PARAMETRI
        # =====================================================

        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')

        # TF frames
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        # Pure Pursuit parametri
        self.declare_parameter('lookahead_distance', 0.35)
        self.declare_parameter('max_linear_velocity', 0.25)
        self.declare_parameter('min_linear_velocity', 0.05)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('goal_tolerance', 0.10)

        # Usporavanje
        self.declare_parameter('slowdown_distance', 0.850)
        self.declare_parameter('slowdown_min_factor', 0.30)

        self.declare_parameter('control_rate', 20.0)

        # =====================================================
        # PARAMETRI - UČITAVANJE
        # =====================================================

        plan_topic = self.get_parameter('plan_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.lookahead_distance = self.get_parameter(
            'lookahead_distance'
        ).value

        self.max_linear_velocity = self.get_parameter(
            'max_linear_velocity'
        ).value

        self.min_linear_velocity = self.get_parameter(
            'min_linear_velocity'
        ).value

        self.max_angular_velocity = self.get_parameter(
            'max_angular_velocity'
        ).value

        self.goal_tolerance = self.get_parameter(
            'goal_tolerance'
        ).value

        self.slowdown_distance = self.get_parameter(
            'slowdown_distance'
        ).value

        self.slowdown_min_factor = self.get_parameter(
            'slowdown_min_factor'
        ).value

        self.control_rate = self.get_parameter(
            'control_rate'
        ).value

        # =====================================================
        # VARIJABLE
        # =====================================================

        self.path = None
        self.current_index = 0

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        self.finished = True

        # =====================================================
        # TF
        # =====================================================

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # =====================================================
        # PUBLISHER
        # =====================================================

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10
        )

        # =====================================================
        # SUBSCRIBER
        # =====================================================

        self.plan_sub = self.create_subscription(
            Path,
            plan_topic,
            self.plan_callback,
            10
        )

        # =====================================================
        # CONTROL LOOP
        # =====================================================

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )

        # =====================================================
        # INFO
        # =====================================================

        self.get_logger().info(
            '========================================'
        )
        self.get_logger().info(
            'ASTRO PURE PURSUIT FOLLOWER'
        )
        self.get_logger().info(
            '========================================'
        )
        self.get_logger().info(
            f'Plan topic:    {plan_topic}'
        )
        self.get_logger().info(
            f'Cmd vel topic: {cmd_vel_topic}'
        )
        self.get_logger().info(
            f'TF:            '
            f'{self.map_frame} -> {self.base_frame}'
        )
        self.get_logger().info(
            f'Lookahead:     '
            f'{self.lookahead_distance:.2f} m'
        )
        self.get_logger().info(
            f'Max velocity:  '
            f'{self.max_linear_velocity:.2f} m/s'
        )
        self.get_logger().info(
            f'Goal tolerance:'
            f'{self.goal_tolerance:.2f} m'
        )
        self.get_logger().info(
            'Čekam putanju na /plan...'
        )

    # =========================================================
    # PLAN CALLBACK
    # =========================================================

    def plan_callback(self, msg: Path):
        """
        Prima novu putanju od planera.
        Svaka nova putanja resetira praćenje od početka.
        """

        if len(msg.poses) == 0:
            self.get_logger().warn(
                'Primio sam praznu putanju. Zanemarujem.'
            )
            return

        self.path = msg
        self.current_index = 0
        self.finished = False

        self.get_logger().info(
            '========================================'
        )
        self.get_logger().info(
            f'Nova putanja primljena: '
            f'{len(msg.poses)} waypointa.'
        )
        self.get_logger().info(
            'Počinjem praćenje putanje...'
        )
        self.get_logger().info(
            '========================================'
        )

    # =========================================================
    # UPDATE ROBOT POSE
    # =========================================================

    def update_robot_pose(self) -> bool:
        """
        Dohvaća poziciju robota iz TF stabla
        map -> base_footprint.
        """

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )

            t = transform.transform.translation
            r = transform.transform.rotation

            self.robot_x = t.x
            self.robot_y = t.y

            self.robot_yaw = self.quaternion_to_yaw(
                r.x,
                r.y,
                r.z,
                r.w
            )

            return True

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):
            return False

    # =========================================================
    # QUATERNION -> YAW
    # =========================================================

    def quaternion_to_yaw(
        self,
        x,
        y,
        z,
        w
    ) -> float:

        siny_cosp = 2.0 * (
            w * z + x * y
        )

        cosy_cosp = 1.0 - 2.0 * (
            y * y + z * z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    # =========================================================
    # NORMALIZE ANGLE
    # =========================================================

    def normalize_angle(
        self,
        angle: float
    ) -> float:

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # =========================================================
    # FIND LOOKAHEAD POINT
    # =========================================================

    def find_lookahead_point(self):
        """
        Traži prvu točku na putanju koja je udaljena barem
        lookahead_distance od robota, počevši od current_index.
        """

        if self.path is None:
            return None

        poses = self.path.poses

        if len(poses) == 0:
            return None

        for i in range(
            self.current_index,
            len(poses)
        ):

            wx = poses[i].pose.position.x
            wy = poses[i].pose.position.y

            dx = wx - self.robot_x
            dy = wy - self.robot_y

            distance = math.hypot(
                dx,
                dy
            )

            if distance >= self.lookahead_distance:

                self.current_index = i

                return wx, wy

        # Svi waypointovi su bliže od lookahead distance
        last = poses[-1]

        return (
            last.pose.position.x,
            last.pose.position.y
        )

    # =========================================================
    # PURE PURSUIT - IZRAČUN BRZINA
    # =========================================================

    def calculate_command(
        self,
        target_x: float,
        target_y: float
    ):
        """
        Računa linearu i kutnu brzinu.

        Ako je cilj iza robota:
            robot se okreće u mjestu.

        Kada cilj dođe ispred robota:
            nastavlja normalni Pure Pursuit.
        """

        # -----------------------------------------------------
        # VEKTOR ROBOT -> CILJ
        # -----------------------------------------------------

        dx = target_x - self.robot_x
        dy = target_y - self.robot_y

        # -----------------------------------------------------
        # APSOLUTNI KUT PREMA CILJU
        # -----------------------------------------------------

        target_yaw = math.atan2(
            dy,
            dx
        )

        angle_error = self.normalize_angle(
            target_yaw - self.robot_yaw
        )

        # -----------------------------------------------------
        # TRANSFORMACIJA CILJA U LOKALNI FRAME ROBOTA
        # -----------------------------------------------------

        cos_yaw = math.cos(
            self.robot_yaw
        )

        sin_yaw = math.sin(
            self.robot_yaw
        )

        local_x = (
            cos_yaw * dx
            + sin_yaw * dy
        )

        local_y = (
            -sin_yaw * dx
            + cos_yaw * dy
        )

        # -----------------------------------------------------
        # AKO JE CILJ IZA ROBOTA
        # -----------------------------------------------------

        if local_x <= 0.0:

            # Ako smo dovoljno poravnati s ciljem,
            # kreni naprijed.
            if abs(angle_error) < 0.15:

                self.get_logger().debug(
                    'Cilj je praktički ispred robota.'
                )

                return (
                    self.min_linear_velocity,
                    0.0
                )

            # -------------------------------------------------
            # ROTACIJA U MJESTU
            # -------------------------------------------------

            if angle_error > 0.0:
                omega = self.max_angular_velocity
            else:
                omega = -self.max_angular_velocity

            self.get_logger().debug(
                f'Cilj iza robota. '
                f'Rotacija u mjestu. '
                f'Angle error: '
                f'{math.degrees(angle_error):.1f} deg'
            )

            return (
                0.0,
                omega
            )

        # -----------------------------------------------------
        # NORMALNI PURE PURSUIT
        # -----------------------------------------------------

        distance_sq = (
            local_x * local_x
            + local_y * local_y
        )

        distance = math.sqrt(
            max(
                distance_sq,
                1e-6
            )
        )

        # -----------------------------------------------------
        # PURE PURSUIT CURVATURE
        # -----------------------------------------------------

        curvature = (
            2.0 * local_y
            / max(
                distance_sq,
                1e-6
            )
        )

        # -----------------------------------------------------
        # LINEARNA BRZINA
        # -----------------------------------------------------

        v = self.max_linear_velocity

        # -----------------------------------------------------
        # SLOWDOWN U BLIZINI GOALA
        # -----------------------------------------------------

        final_pose = self.path.poses[-1]

        final_x = (
            final_pose.pose.position.x
        )

        final_y = (
            final_pose.pose.position.y
        )

        dist_to_goal = math.hypot(
            final_x - self.robot_x,
            final_y - self.robot_y
        )

        if dist_to_goal < self.slowdown_distance:

            scale = (
                dist_to_goal
                / self.slowdown_distance
            )

            v *= max(
                self.slowdown_min_factor,
                scale
            )

        # -----------------------------------------------------
        # MINIMALNA BRZINA
        # -----------------------------------------------------

        v = max(
            v,
            self.min_linear_velocity
        )

        # -----------------------------------------------------
        # KUTNA BRZINA
        # -----------------------------------------------------

        omega = curvature * v

        omega = max(
            -self.max_angular_velocity,
            min(
                omega,
                self.max_angular_velocity
            )
        )

        self.get_logger().debug(
            f'Target: '
            f'({target_x:.3f}, {target_y:.3f}) | '
            f'local: '
            f'({local_x:.3f}, {local_y:.3f}) | '
            f'angle error: '
            f'{math.degrees(angle_error):.1f} deg | '
            f'kappa: '
            f'{curvature:.3f} | '
            f'v: '
            f'{v:.3f} m/s | '
            f'omega: '
            f'{omega:.3f} rad/s'
        )

        return (
            v,
            omega
        )

    # =========================================================
    # CHECK GOAL REACHED
    # =========================================================

    def check_goal_reached(self) -> bool:
        """
        Provjerava je li robot stigao do zadnjeg waypointa.
        """

        if self.path is None:
            return False

        poses = self.path.poses

        if len(poses) == 0:
            return False

        # Ne provjeravaj cilj prije 90% putanje
        progress = (
            self.current_index
            / float(len(poses))
        )

        if progress < 0.90:
            return False

        final_pose = poses[-1]

        gx = final_pose.pose.position.x
        gy = final_pose.pose.position.y

        dist = math.hypot(
            gx - self.robot_x,
            gy - self.robot_y
        )

        return dist < self.goal_tolerance

    # =========================================================
    # STOP ROBOT
    # =========================================================

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(
            cmd
        )

    # =========================================================
    # CONTROL LOOP
    # =========================================================

    def control_loop(self):
        """
        Glavna upravljačka petlja.
        """

        # -----------------------------------------------------
        # UPDATE TF
        # -----------------------------------------------------

        tf_ok = self.update_robot_pose()

        if not tf_ok:

            self.stop_robot()

            return

        # -----------------------------------------------------
        # ČEKANJE PUTANJE
        # -----------------------------------------------------

        if (
            self.path is None
            or self.finished
        ):

            self.stop_robot()

            return

        # -----------------------------------------------------
        # PROVJERA GOALA
        # -----------------------------------------------------

        if self.check_goal_reached():

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'GOAL DOSTIGNUT!'
            )

            self.get_logger().info(
                'Robot staje.'
            )

            self.get_logger().info(
                'Čekam novu putanju...'
            )

            self.get_logger().info(
                '========================================'
            )

            self.finished = True
            self.path = None

            self.stop_robot()

            return

        # -----------------------------------------------------
        # LOOKAHEAD
        # -----------------------------------------------------

        target = self.find_lookahead_point()

        if target is None:

            self.stop_robot()

            return

        target_x, target_y = target

        # -----------------------------------------------------
        # IZRAČUN KOMANDE
        # -----------------------------------------------------

        v, omega = self.calculate_command(
            target_x,
            target_y
        )

        # -----------------------------------------------------
        # PUBLISH CMD_VEL
        # -----------------------------------------------------

        cmd = Twist()

        cmd.linear.x = v
        cmd.angular.z = omega

        self.cmd_vel_pub.publish(
            cmd
        )


# =========================================================
# MAIN
# =========================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = PurePursuitFollower()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            'Keyboard interrupt. Gasim follower.'
        )

    finally:

        node.stop_robot()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
