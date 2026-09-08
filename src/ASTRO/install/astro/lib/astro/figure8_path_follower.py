#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist


class Figure8Follower(Node):

    def __init__(self):
        super().__init__('figure8_follower')

        # =========================================================
        # PARAMETRI
        # =========================================================

        # Lookahead udaljenost [m]
        self.declare_parameter(
            'lookahead_distance',
            0.3
        )

        # Maksimalna linearna brzina [m/s]
        self.declare_parameter(
            'max_linear_velocity',
            0.20
        )

        # Maksimalna kutna brzina [rad/s]
        self.declare_parameter(
            'max_angular_velocity',
            1.0
        )

        # Minimalna linearna brzina [m/s]
        self.declare_parameter(
            'min_linear_velocity',
            0.02
        )

        # Frekvencija kontrolera [Hz]
        self.declare_parameter(
            'control_rate',
            30.0
        )

        # Tolerancija završne točke [m]
        self.declare_parameter(
            'goal_tolerance',
            0.10
        )

        # Topic s putanjom
        self.declare_parameter(
            'path_topic',
            '/figure8_path'
        )

        # Topic s odometrijom
        self.declare_parameter(
            'odom_topic',
            '/diff_drive_base_controller/odom'
        )

        # Topic za upravljanje
        self.declare_parameter(
            'cmd_vel_topic',
            '/diff_drive_base_controller/cmd_vel_unstamped'
        )

        # =========================================================
        # ČITANJE PARAMETARA
        # =========================================================

        self.lookahead_distance = self.get_parameter(
            'lookahead_distance'
        ).value

        self.max_linear_velocity = self.get_parameter(
            'max_linear_velocity'
        ).value

        self.max_angular_velocity = self.get_parameter(
            'max_angular_velocity'
        ).value

        self.min_linear_velocity = self.get_parameter(
            'min_linear_velocity'
        ).value

        self.control_rate = self.get_parameter(
            'control_rate'
        ).value

        self.goal_tolerance = self.get_parameter(
            'goal_tolerance'
        ).value

        path_topic = self.get_parameter(
            'path_topic'
        ).value

        odom_topic = self.get_parameter(
            'odom_topic'
        ).value

        cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic'
        ).value

        # =========================================================
        # VARIJABLE
        # =========================================================

        self.path = None

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        self.finished = False

        # Trenutni indeks putanje.
        #
        # VAŽNO:
        # Ovaj indeks se nikada ne smije smanjivati.
        self.current_index = 0

        # Brojač za periodički debug ispis
        self.last_debug_index = -1

        # =========================================================
        # PUBLISHER
        # =========================================================

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10
        )

        # =========================================================
        # SUBSCRIBERS
        # =========================================================

        self.path_sub = self.create_subscription(
            Path,
            path_topic,
            self.path_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        # =========================================================
        # CONTROL LOOP
        # =========================================================

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )

        # =========================================================
        # LOG
        # =========================================================

        self.get_logger().info(
            'Figure 8 follower started.'
        )

        self.get_logger().info(
            f'Lookahead: '
            f'{self.lookahead_distance:.2f} m'
        )

        self.get_logger().info(
            f'Max linear velocity: '
            f'{self.max_linear_velocity:.2f} m/s'
        )

        self.get_logger().info(
            f'Max angular velocity: '
            f'{self.max_angular_velocity:.2f} rad/s'
        )

        self.get_logger().info(
            f'Goal tolerance: '
            f'{self.goal_tolerance:.2f} m'
        )

    # =============================================================
    # PATH CALLBACK
    # =============================================================

    def path_callback(self, msg):

        # Prvu putanju prihvati.
        if self.path is None:

            self.path = msg

            self.current_index = 0
            self.finished = False
            self.last_debug_index = -1

            self.get_logger().info(
                f'Received path with '
                f'{len(msg.poses)} points.'
            )

            return

        # Ako publisher ponovno šalje istu putanju,
        # NE resetiramo current_index.
        #
        # Ovo je vrlo važno.
        #
        # Publisher može periodički ponovno objavljivati
        # istu Path poruku.

    # =============================================================
    # ODOM CALLBACK
    # =============================================================

    def odom_callback(self, msg):

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        self.robot_yaw = self.quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

    # =============================================================
    # QUATERNION -> YAW
    # =============================================================

    def quaternion_to_yaw(
        self,
        x,
        y,
        z,
        w
    ):

        siny_cosp = 2.0 * (
            w * z +
            x * y
        )

        cosy_cosp = 1.0 - 2.0 * (
            y * y +
            z * z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    # =============================================================
    # NORMALIZACIJA KUTA
    # =============================================================

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # =============================================================
    # PRONAĐI LOOKAHEAD TOČKU
    # =============================================================

    def find_lookahead_point(self):

        if self.path is None:
            return None

        poses = self.path.poses

        if len(poses) == 0:
            return None

        # ---------------------------------------------------------
        # PRONAĐI NAJBLIŽU TOČKU
        # ---------------------------------------------------------
        #
        # Tražimo najbližu točku samo ispred current_index.
        #
        # Ne tražimo po cijeloj putanji jer bi se kod figure-8
        # robot mogao vratiti na drugi dio osmice koji je
        # prostorno blizu, ali vremenski nije sljedeći dio putanje.
        #

        closest_index = self.current_index
        closest_distance = float('inf')

        # Maksimalan broj točaka koje gledamo unaprijed.
        search_end = min(
            self.current_index + 100,
            len(poses)
        )

        for i in range(
            self.current_index,
            search_end
        ):

            px = poses[i].pose.position.x
            py = poses[i].pose.position.y

            dx = px - self.robot_x
            dy = py - self.robot_y

            distance = math.sqrt(
                dx * dx +
                dy * dy
            )

            if distance < closest_distance:

                closest_distance = distance
                closest_index = i

        # ---------------------------------------------------------
        # CURRENT INDEX SE MOŽE SAMO POVEĆAVATI
        # ---------------------------------------------------------

        if closest_index > self.current_index:

            self.current_index = closest_index

        # ---------------------------------------------------------
        # PRONAĐI LOOKAHEAD TOČKU
        # ---------------------------------------------------------

        for i in range(
            self.current_index,
            len(poses)
        ):

            px = poses[i].pose.position.x
            py = poses[i].pose.position.y

            dx = px - self.robot_x
            dy = py - self.robot_y

            distance = math.sqrt(
                dx * dx +
                dy * dy
            )

            if distance >= self.lookahead_distance:

                self.current_index = max(
                    self.current_index,
                    i
                )

                return px, py

        # ---------------------------------------------------------
        # NEMA VIŠE LOOKAHEAD TOČAKA
        #
        # Robot je na kraju putanje.
        # ---------------------------------------------------------

        self.current_index = len(poses) - 1

        last = poses[-1]

        return (
            last.pose.position.x,
            last.pose.position.y
        )

    # =============================================================
    # PURE PURSUIT
    # =============================================================

    def calculate_command(
        self,
        target_x,
        target_y
    ):

        # ---------------------------------------------------------
        # RELATIVNA POZICIJA CILJA
        # ---------------------------------------------------------

        dx = target_x - self.robot_x
        dy = target_y - self.robot_y

        # ---------------------------------------------------------
        # TRANSFORMACIJA U ROBOTOV LOKALNI KOORDINATNI SUSTAV
        # ---------------------------------------------------------

        cos_yaw = math.cos(
            self.robot_yaw
        )

        sin_yaw = math.sin(
            self.robot_yaw
        )

        local_x = (
            cos_yaw * dx +
            sin_yaw * dy
        )

        local_y = (
            -sin_yaw * dx +
            cos_yaw * dy
        )

        # ---------------------------------------------------------
        # PURE PURSUIT
        #
        # curvature = 2*y / L²
        # ---------------------------------------------------------

        distance_squared = (
            local_x * local_x +
            local_y * local_y
        )

        curvature = (
            2.0 * local_y /
            max(
                distance_squared,
                1e-6
            )
        )

        # ---------------------------------------------------------
        # LINEARNA BRZINA
        # ---------------------------------------------------------

        v = self.max_linear_velocity

        # ---------------------------------------------------------
        # KUTNA BRZINA
        # ---------------------------------------------------------

        omega = curvature * v

        # Ograničenje kutne brzine
        omega = max(
            -self.max_angular_velocity,
            min(
                omega,
                self.max_angular_velocity
            )
        )

        # ---------------------------------------------------------
        # USPORAVANJE PREMA CILJU
        # ---------------------------------------------------------

        distance = math.sqrt(
            distance_squared
        )

        if distance < 0.30:

            scale = distance / 0.30

            v *= scale

            # Vrlo mala minimalna brzina.
            #
            # Ne koristimo originalnih 0.05 m/s jer želimo
            # omogućiti followeru da precizno dođe do završne
            # točke.
            v = max(
                v,
                self.min_linear_velocity
            )

        return v, omega

    # =============================================================
    # PROVJERA JE LI ROBOT NA KRAJU PUTANJE
    # =============================================================

    def check_goal(self):

        if self.path is None:
            return False

        if len(self.path.poses) == 0:
            return False

        # ---------------------------------------------------------
        # NAPREDAK PO PUTANJI
        # ---------------------------------------------------------

        total_points = len(
            self.path.poses
        )

        if total_points <= 1:
            return False

        progress = (
            self.current_index /
            float(total_points - 1)
        )

        # ---------------------------------------------------------
        # ROBOT MORA BITI U ZADNJIH 10 %
        # ---------------------------------------------------------

        if progress < 0.90:
            return False

        # ---------------------------------------------------------
        # ZADNJA TOČKA
        # ---------------------------------------------------------

        goal = self.path.poses[-1]

        gx = goal.pose.position.x
        gy = goal.pose.position.y

        dx = gx - self.robot_x
        dy = gy - self.robot_y

        distance = math.sqrt(
            dx * dx +
            dy * dy
        )

        # ---------------------------------------------------------
        # DEBUG
        # ---------------------------------------------------------

        if (
            self.current_index !=
            self.last_debug_index
        ):

            if progress >= 0.90:

                self.get_logger().info(
                    f'Final section: '
                    f'{progress * 100.0:.1f}% '
                    f'| distance to goal: '
                    f'{distance:.3f} m'
                )

                self.last_debug_index = (
                    self.current_index
                )

        # ---------------------------------------------------------
        # GOAL
        # ---------------------------------------------------------

        if distance < self.goal_tolerance:

            self.get_logger().info(
                f'Goal reached. '
                f'Progress: '
                f'{progress * 100.0:.1f}% '
                f'| distance: '
                f'{distance:.3f} m'
            )

            return True

        return False

    # =============================================================
    # STOP ROBOT
    # =============================================================

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

    # =============================================================
    # GLAVNA KONTROLNA PETLJA
    # =============================================================

    def control_loop(self):

        # ---------------------------------------------------------
        # NEMAMO PUTANJU
        # ---------------------------------------------------------

        if self.path is None:
            return

        # ---------------------------------------------------------
        # NEMAMO ODOMETRIJU
        # ---------------------------------------------------------

        if self.robot_x is None:
            return

        if self.robot_y is None:
            return

        if self.robot_yaw is None:
            return

        # ---------------------------------------------------------
        # AKO SMO ZAVRŠILI
        # ---------------------------------------------------------

        if self.finished:

            self.stop_robot()

            return

        # ---------------------------------------------------------
        # PRONAĐI LOOKAHEAD
        #
        # Važno:
        # prvo ažuriramo current_index.
        # Tek nakon toga provjeravamo goal.
        # ---------------------------------------------------------

        target = self.find_lookahead_point()

        if target is None:
            return

        # ---------------------------------------------------------
        # PROVJERI KRAJ PUTANJE
        # ---------------------------------------------------------

        if self.check_goal():

            self.get_logger().info(
                'Figure 8 completed. '
                'Stopping robot.'
            )

            self.finished = True

            self.stop_robot()

            return

        # ---------------------------------------------------------
        # TARGET
        # ---------------------------------------------------------

        target_x, target_y = target

        # ---------------------------------------------------------
        # IZRAČUN BRZINA
        # ---------------------------------------------------------

        v, omega = self.calculate_command(
            target_x,
            target_y
        )

        # ---------------------------------------------------------
        # PUBLISH TWIST
        # ---------------------------------------------------------

        cmd = Twist()

        cmd.linear.x = v
        cmd.angular.z = omega

        self.cmd_vel_pub.publish(
            cmd
        )

        # ---------------------------------------------------------
        # DEBUG NAPRETKOM
        # ---------------------------------------------------------

        if (
            self.current_index !=
            self.last_debug_index
        ):

            total_points = len(
                self.path.poses
            )

            if total_points > 1:

                progress = (
                    self.current_index /
                    float(total_points - 1)
                )

                # Ispis svakih približno 10 %
                debug_step = (
                    int(progress * 10)
                )

                previous_step = (
                    int(
                        self.last_debug_index /
                        max(
                            total_points - 1,
                            1
                        ) * 10
                    )
                    if self.last_debug_index >= 0
                    else -1
                )

                if debug_step != previous_step:

                    self.get_logger().info(
                        f'Path progress: '
                        f'{progress * 100.0:.1f}% '
                        f'| index: '
                        f'{self.current_index}/'
                        f'{total_points - 1}'
                    )

                    self.last_debug_index = (
                        self.current_index
                    )


# ================================================================
# MAIN
# ================================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = Figure8Follower()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.stop_robot()

        node.destroy_node()

        rclpy.shutdown()


# ================================================================
# START
# ================================================================

if __name__ == '__main__':

    main()


