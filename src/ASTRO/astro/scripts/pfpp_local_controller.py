#!/usr/bin/env python3
"""
Potential Field Pure Pursuit - lokalni kontroler (nova, čista struktura)
============================================================================
Lokalno praćenje globalnog A* puta (/plan) uz kontinuirano,
udaljenošću-modulirano izbjegavanje prepreka iz LiDAR-a (/scan):

    PURE PURSUIT (privlačni vektor, prati /plan)
                    +
    POTENTIAL FIELD (odbojni vektor, iz /scan)
                    =
         REZULTANTNI SMJER -> v, omega -> /cmd_vel

KLJUČNA RAZLIKA u odnosu na pf_pure_pursuit.py (prijašnja verzija) -
zašto je stara verzija titrala/naglo skretala:

    1) Stara odbojna sila je rasla kao ~1/r^3 (klasična APF formula
       gain*(1/r - 1/R)/r^2). To znači da je preko ~80% dosega
       (`influence_radius`) sila bila zanemariva, a onda je
       eksplodirala u zadnjih ~15-20% udaljenosti - iz robotove
       perspektive to izgleda kao skoro-binarni prekidač, ne kao
       glatko, postupno guranje. Ovdje se koristi ograničen,
       kvadratni "ease-in" profil (0 na influence_radius, gain na
       min_obstacle_distance) - svaka zraka doprinosi najviše
       `repulsive_gain`, bez obzira koliko je blizu prepreka.

    2) Stara verzija je imala TVRDI prekidač "vozi normalno" /
       "rotiraj u mjestu" na temelju praga kuta. Prepreka bi
       kombinirani vektor gurnula preko praga i robot bi u JEDNOM
       ciklusu prešao s vožnje na rotaciju skoro punom kutnom
       brzinom - to je uzrok "naglog/silovitog skretanja". Ovdje
       nema tog prekidača: v i omega se računaju KONTINUIRANO iz
       heading errora (v prirodno ide prema 0 kako heading error
       raste prema 90-180°, umjesto da skokovito padne na 0).
       Jedina preostala "zaključana" rotacija je uski rubni slučaj
       kad je cilj/rezultanta gotovo točno iza robota (>~155°) -
       tada je smjer rotacije (lijevo/desno) suštinski
       neodređen pa se privremeno zaključa da ne bi treperio, ali
       čak i tu je v već ~0 iz kontinuirane formule, pa nema
       diskontinuiteta u linearnoj brzini.

    3) Stara verzija nije normirala odbojni vektor po broju
       doprinoseći zraka - jačina je ovisila o `scan_stride` i
       kutnoj rezoluciji lasera (koliko zraka upadne u konus).
       Ovdje se koristi PROSJEK doprinosećih zraka, pa je
       `repulsive_gain` prenosiv između različitih lasera/stride
       postavki.

    4) Kad je prepreka gotovo točno ispred (simetrična), bočna
       (y) komponenta odbojnog vektora je blizu nule i šum
       lasera može promijeniti njen predznak iz skena u sken -
       robot bi nasumično birao stranu zaobilaženja svaki put
       iznova. Ovdje postoji "side bias": kad je odbojni vektor
       jak ali bočno dvosmislen, blago se nagne prema strani koja
       je zadnji put stvarno bila izabrana (`last_avoidance_side`)
       - jednom izabrana strana zaobilaska se zadržava.

    5) Rate-limit je postojao samo za kutnu brzinu. Ovdje postoji
       i za linearnu (`max_linear_acceleration`), pa v ne skače
       skokovito na prijelazima (ulaz/izlaz iz izbjegavanja,
       usporavanje pred prepreku).

    6) TF (map -> base_frame) greška je odmah zaustavljala robota.
       U multi-machine postavu bez zajedničkog NTP-a (isti razlog
       zbog kojeg cmd_safety.py/dynamic_ob_layer.py izbjegavaju TF)
       to zna izazvati kratke, nepotrebne zastoje. Ovdje se kratko
       vrijeme (`tf_grace_period`) koristi zadnja poznata pozicija
       prije nego što se robot stvarno zaustavi.

    Hard-stop failsafe (zadnja linija obrane, neovisna o
    potencijalnom polju) je zadržan, ali sad zahtijeva N uzastopnih
    skenova prije okidanja/otpuštanja (debounce) da ga jedna
    šumovita zraka ne aktivira/deaktivira nasumično.

Topici:
    Pretplate:
        /plan   (nav_msgs/Path)          <- A* planer
        /scan   (sensor_msgs/LaserScan)  <- potencijalno polje
    TF:
        map -> odom -> base_frame
    Publicira:
        /cmd_vel (geometry_msgs/Twist)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import tf2_ros


def clamp(value, lo, hi):
    return max(lo, min(value, hi))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


# =================================================================
# PURE PURSUIT - privlačna komponenta
# =================================================================

class PurePursuit:
    """Traži lookahead točku na putu i vraća privlačni jedinični
    vektor (lokalni frame robota) prema njoj. Lookahead udaljenost
    je adaptivna - raste s brzinom (dulji doseg pri većoj brzini
    smanjuje rezanje uglova, kraći doseg pri maloj brzini smanjuje
    titranje)."""

    def __init__(
        self,
        lookahead_min: float,
        lookahead_max: float,
        lookahead_speed_gain: float,
        attractive_gain: float,
        goal_tolerance: float,
    ):
        self.lookahead_min = lookahead_min
        self.lookahead_max = lookahead_max
        self.lookahead_speed_gain = lookahead_speed_gain
        self.attractive_gain = attractive_gain
        self.goal_tolerance = goal_tolerance

        self.path = None
        self.current_index = 0

    def set_path(self, path: Path):
        self.path = path
        self.current_index = 0

    def clear(self):
        self.path = None
        self.current_index = 0

    def has_path(self) -> bool:
        return self.path is not None and len(self.path.poses) > 0

    def lookahead_distance_for_speed(self, speed: float) -> float:
        look = self.lookahead_min + self.lookahead_speed_gain * speed
        return clamp(look, self.lookahead_min, self.lookahead_max)

    def find_lookahead_point(self, robot_x, robot_y, lookahead_dist):
        """Vraća (x, y) lookahead točke, napredujući current_index
        monotono naprijed duž puta kako se robot kreće."""

        poses = self.path.poses

        while self.current_index < len(poses) - 1:
            px = poses[self.current_index].pose.position.x
            py = poses[self.current_index].pose.position.y
            if math.hypot(px - robot_x, py - robot_y) >= lookahead_dist:
                break
            self.current_index += 1

        for idx in range(self.current_index, len(poses)):
            px = poses[idx].pose.position.x
            py = poses[idx].pose.position.y
            if math.hypot(px - robot_x, py - robot_y) >= lookahead_dist:
                self.current_index = idx
                return px, py

        final_pose = poses[-1]
        return final_pose.pose.position.x, final_pose.pose.position.y

    def goal_point(self):
        final_pose = self.path.poses[-1]
        return final_pose.pose.position.x, final_pose.pose.position.y

    def progress_fraction(self) -> float:
        return self.current_index / float(len(self.path.poses))

    def attractive_vector_local(self, target_x, target_y, robot_x, robot_y, robot_yaw):
        """Privlačni jedinični vektor prema lookahead točki, u
        lokalnom frameu robota (x = naprijed, y = lijevo)."""

        dx = target_x - robot_x
        dy = target_y - robot_y

        cos_yaw = math.cos(-robot_yaw)
        sin_yaw = math.sin(-robot_yaw)

        local_x = dx * cos_yaw - dy * sin_yaw
        local_y = dx * sin_yaw + dy * cos_yaw

        norm = math.hypot(local_x, local_y)
        if norm > 1e-6:
            local_x /= norm
            local_y /= norm

        return local_x * self.attractive_gain, local_y * self.attractive_gain


# =================================================================
# POTENTIAL FIELD - odbojna komponenta
# =================================================================

class PotentialField:
    """Iz sirovog /scan računa ograničen, glatko-rastući odbojni
    vektor (samo prednji konus) i neovisan hard-stop failsafe signal
    (uži konus, s debounceom protiv šumovitih očitanja)."""

    def __init__(
        self,
        influence_radius: float,
        min_obstacle_distance: float,
        repulsive_gain: float,
        repulsive_smoothing_alpha: float,
        scan_stride: int,
        scan_angle_offset_deg: float,
        pf_front_angle_deg: float,
        hard_stop_distance: float,
        hard_stop_front_angle_deg: float,
        hard_stop_confirm_cycles: int,
        hard_stop_release_cycles: int,
        side_bias_activation_ratio: float,
        side_bias_ambiguous_ratio: float,
        side_bias_strength: float,
    ):
        self.influence_radius = influence_radius
        self.min_obstacle_distance = min_obstacle_distance
        self.repulsive_gain = repulsive_gain
        self.smoothing_alpha = repulsive_smoothing_alpha
        self.scan_stride = max(1, int(scan_stride))
        self.scan_angle_offset = math.radians(scan_angle_offset_deg)
        self.pf_half_cone = math.radians(pf_front_angle_deg) / 2.0
        self.hard_stop_distance = hard_stop_distance
        self.hard_stop_half_cone = math.radians(hard_stop_front_angle_deg) / 2.0
        self.hard_stop_confirm_cycles = max(1, int(hard_stop_confirm_cycles))
        self.hard_stop_release_cycles = max(1, int(hard_stop_release_cycles))

        self.side_bias_activation_ratio = side_bias_activation_ratio
        self.side_bias_ambiguous_ratio = side_bias_ambiguous_ratio
        self.side_bias_strength = side_bias_strength

        self.rep_x = 0.0
        self.rep_y = 0.0
        self.rep_magnitude = 0.0

        self.min_front_distance = float('inf')

        self._close_count = 0
        self._clear_count = 0
        self.hard_stop_active = False

        # Zadnja strana kojom je robot stvarno zaobišao prepreku
        # (+1 = lijevo, -1 = desno) - koristi se kao tie-breaker
        # kad je nova prepreka bočno dvosmislena (skoro točno
        # ispred).
        self.last_avoidance_side = 1.0

    def process_scan(self, msg: LaserScan):

        rep_x_sum = 0.0
        rep_y_sum = 0.0
        rep_count = 0

        min_front = float('inf')
        r_min = max(msg.range_min, 0.03)

        span = self.influence_radius - self.min_obstacle_distance
        span = max(span, 1e-3)

        for i in range(0, len(msg.ranges), self.scan_stride):

            r = msg.ranges[i]

            if not math.isfinite(r) or r < msg.range_min:
                continue

            angle = normalize_angle(
                msg.angle_min + i * msg.angle_increment - self.scan_angle_offset
            )

            if abs(angle) <= self.hard_stop_half_cone and r < min_front:
                min_front = r

            if abs(angle) > self.pf_half_cone or r >= self.influence_radius:
                continue

            d = clamp(r, r_min, self.influence_radius)
            d = max(d, self.min_obstacle_distance)

            # Ograničen "ease-in" profil: 0 na influence_radius,
            # puni gain na min_obstacle_distance. Kvadrat daje blag
            # početak i čvršći odgovor kad je prepreka stvarno
            # blizu, bez singulariteta klasične 1/r formule.
            s = (self.influence_radius - d) / span
            s = clamp(s, 0.0, 1.0)
            magnitude = self.repulsive_gain * s * s

            rep_x_sum += -magnitude * math.cos(angle)
            rep_y_sum += -magnitude * math.sin(angle)
            rep_count += 1

        if rep_count > 0:
            raw_rep_x = rep_x_sum / rep_count
            raw_rep_y = rep_y_sum / rep_count
        else:
            raw_rep_x = 0.0
            raw_rep_y = 0.0

        alpha = self.smoothing_alpha
        self.rep_x = alpha * raw_rep_x + (1.0 - alpha) * self.rep_x
        self.rep_y = alpha * raw_rep_y + (1.0 - alpha) * self.rep_y

        self._apply_side_bias()

        self.rep_magnitude = math.hypot(self.rep_x, self.rep_y)
        self.min_front_distance = min_front

        self._update_hard_stop_state(min_front)

    def _apply_side_bias(self):

        magnitude = math.hypot(self.rep_x, self.rep_y)

        if magnitude < self.side_bias_activation_ratio * self.repulsive_gain:
            return

        if abs(self.rep_y) < self.side_bias_ambiguous_ratio * magnitude:
            self.rep_y += (
                self.side_bias_strength * magnitude * self.last_avoidance_side
            )
        else:
            self.last_avoidance_side = 1.0 if self.rep_y > 0.0 else -1.0

    def _update_hard_stop_state(self, min_front: float):

        if min_front < self.hard_stop_distance:
            self._close_count += 1
            self._clear_count = 0
        else:
            self._clear_count += 1
            self._close_count = 0

        if not self.hard_stop_active:
            if self._close_count >= self.hard_stop_confirm_cycles:
                self.hard_stop_active = True
        else:
            if self._clear_count >= self.hard_stop_release_cycles:
                self.hard_stop_active = False

    def severity(self) -> float:
        """Normirana [0, 1] jačina odbijanja - koristi se za
        usporavanje linearne brzine."""
        if self.repulsive_gain <= 1e-6:
            return 0.0
        return clamp(self.rep_magnitude / self.repulsive_gain, 0.0, 1.0)


# =================================================================
# COMMAND SHAPER - blending, upravljanje kutom, profil brzine,
# rate-limiting
# =================================================================

class CommandShaper:

    def __init__(
        self,
        heading_kp: float,
        rotation_lock_deadband_deg: float,
        rotation_lock_hysteresis_deg: float,
        rotation_lock_omega_scale: float,
        max_linear_velocity: float,
        min_linear_velocity: float,
        max_angular_velocity: float,
        max_linear_acceleration: float,
        max_angular_acceleration: float,
        slowdown_distance: float,
        slowdown_min_factor: float,
        obstacle_slowdown_min_factor: float,
        control_rate: float,
    ):
        self.heading_kp = heading_kp

        self.lock_enter = math.pi - math.radians(rotation_lock_deadband_deg)
        self.lock_exit = self.lock_enter - math.radians(
            rotation_lock_hysteresis_deg
        )
        self.rotation_lock_omega_scale = rotation_lock_omega_scale

        self.max_linear_velocity = max_linear_velocity
        self.min_linear_velocity = min_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.max_linear_acceleration = max_linear_acceleration
        self.max_angular_acceleration = max_angular_acceleration

        self.slowdown_distance = slowdown_distance
        self.slowdown_min_factor = slowdown_min_factor
        self.obstacle_slowdown_min_factor = obstacle_slowdown_min_factor

        self.control_rate = control_rate

        self.rotation_locked = False
        self.rotation_lock_direction = 1.0

        self.prev_v = 0.0
        self.prev_omega = 0.0

    def reset_rate_limiters(self):
        self.prev_v = 0.0
        self.prev_omega = 0.0

    def compute(
        self,
        comb_x: float,
        comb_y: float,
        dist_to_goal: float,
        obstacle_severity: float,
        fallback_direction: float,
    ):
        """Glavni izračun v, omega iz kombiniranog (privlačni +
        odbojni) vektora. Sve promjene su kontinuirane funkcije
        heading errora/udaljenosti - nema tvrdih prekidača osim
        uskog "rotation lock" ruba za skoro-točno-iza-robota slučaj
        (heading_error > lock_enter)."""

        magnitude = math.hypot(comb_x, comb_y)

        if magnitude < 1e-6:
            # Privlačna i odbojna sila se poništavaju (lokalni
            # minimum) - lagano rotiraj prema zadnjoj poznatoj
            # strani zaobilaska da se izađe iz zastoja.
            omega_target = (
                fallback_direction * self.max_angular_velocity * 0.3
            )
            v = self._rate_limit_v(0.0)
            omega = self._rate_limit_omega(omega_target)
            return v, omega

        heading_error = math.atan2(comb_y, comb_x)

        if self.rotation_locked:
            if abs(heading_error) < self.lock_exit:
                self.rotation_locked = False
        elif abs(heading_error) > self.lock_enter:
            self.rotation_locked = True
            self.rotation_lock_direction = (
                1.0 if heading_error > 0.0 else -1.0
            )

        if self.rotation_locked:
            omega_target = (
                self.rotation_lock_direction
                * self.max_angular_velocity
                * self.rotation_lock_omega_scale
            )
        else:
            omega_target = clamp(
                self.heading_kp * heading_error,
                -self.max_angular_velocity,
                self.max_angular_velocity,
            )

        # Linearna brzina: kontinuirano pada prema 0 kako heading
        # error raste prema +-90 deg i dalje - "rotiraj u mjestu"
        # ponašanje se prirodno pojavljuje iz ove formule, bez
        # zasebnog moda.
        v_target = self.max_linear_velocity * max(0.0, math.cos(heading_error))

        if dist_to_goal < self.slowdown_distance:
            scale = dist_to_goal / self.slowdown_distance
            v_target *= max(self.slowdown_min_factor, scale)

        v_target *= 1.0 - (
            (1.0 - self.obstacle_slowdown_min_factor) * obstacle_severity
        )

        if v_target > 1e-3:
            v_target = max(v_target, self.min_linear_velocity)
        else:
            v_target = 0.0

        v = self._rate_limit_v(v_target)
        omega = self._rate_limit_omega(omega_target)

        return v, omega

    def _rate_limit_v(self, target: float) -> float:
        max_delta = self.max_linear_acceleration / self.control_rate
        v = clamp(target, self.prev_v - max_delta, self.prev_v + max_delta)
        self.prev_v = v
        return v

    def _rate_limit_omega(self, target: float) -> float:
        max_delta = self.max_angular_acceleration / self.control_rate
        omega = clamp(
            target, self.prev_omega - max_delta, self.prev_omega + max_delta
        )
        self.prev_omega = omega
        return omega


# =================================================================
# ROS NODE
# =================================================================

class PFPPLocalController(Node):

    def __init__(self):
        super().__init__('pfpp_local_controller')

        # =====================================================
        # PARAMETRI
        # =====================================================

        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        # --- Pure Pursuit (privlačna komponenta) ---
        # Lookahead se adaptivno skalira s trenutnom brzinom
        # (lookahead_min + lookahead_speed_gain * v, ograničeno na
        # [min, max]) - kraći doseg pri maloj brzini smanjuje
        # titranje, dulji doseg pri velikoj brzini smanjuje rezanje
        # uglova.
        self.declare_parameter('lookahead_distance_min', 0.35)
        self.declare_parameter('lookahead_distance_max', 0.80)
        self.declare_parameter('lookahead_speed_gain', 0.5)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('attractive_gain', 1.1)

        # --- Brzine i ograničenja ---
        self.declare_parameter('max_linear_velocity', 0.25)
        self.declare_parameter('min_linear_velocity', 0.05)
        self.declare_parameter('max_angular_velocity', 2.1)

        # Maksimalna promjena LINEARNE brzine po sekundi [m/s^2].
        # Bez ovoga v može skokovito promijeniti (npr. pri
        # ulasku/izlasku iz usporavanja) - ovo tu promjenu širi
        # kroz više kontrolnih ciklusa.
        self.declare_parameter('max_linear_acceleration', 0.6)

        # Maksimalna promjena KUTNE brzine po sekundi [rad/s^2].
        self.declare_parameter('max_angular_acceleration', 3.2)

        # --- Usporavanje pri približavanju cilju ---
        self.declare_parameter('slowdown_distance', 0.85)
        self.declare_parameter('slowdown_min_factor', 0.30)

        # --- Usporavanje zbog prepreke ---
        # Pri maksimalnoj odbojnoj jačini (severity=1), v se
        # množi ovim faktorom (ne padne na 0 samo zbog prepreke -
        # za to postoji zaseban hard-stop).
        self.declare_parameter('obstacle_slowdown_min_factor', 0.35)

        # --- Upravljanje kutom ---
        # P-pojačanje: heading_error [rad] -> omega [rad/s].
        self.declare_parameter('heading_kp', 1.2)

        # Rub slučaja: kad je rezultantni smjer gotovo točno iza
        # robota (180 - ovaj broj stupnjeva ili više), smjer
        # rotacije (lijevo/desno) je suštinski neodređen pa bi šum
        # mogao izazvati treperenje - tada se smjer rotacije
        # nakratko "zaključa".
        self.declare_parameter('rotation_lock_deadband_deg', 25.0)
        self.declare_parameter('rotation_lock_hysteresis_deg', 10.0)
        self.declare_parameter('rotation_lock_omega_scale', 0.4)

        # --- Potencijalno polje (odbojna komponenta) ---
        self.declare_parameter('influence_radius', 0.9)

        # Udaljenost na kojoj odbojna sila doseže puni
        # repulsive_gain (zasićenje - sprječava da sila eksplodira
        # kako r ide prema 0, za razliku od klasične 1/r formule).
        self.declare_parameter('min_obstacle_distance', 0.15)

        self.declare_parameter('repulsive_gain', 1.3)

        # Eksponencijalno zaglađivanje odbojnog vektora između
        # /scan poruka (0-1). Manje = glađe ali sporije reagira.
        self.declare_parameter('repulsive_smoothing_alpha', 0.3)

        self.declare_parameter('scan_stride', 2)

        # Kut 0 skenera se pretpostavlja "naprijed" - podesi ako
        # laser nije poravnat s robotom (npr. 180 ako je montiran
        # zarotiran, vidi lidar.xacro).
        self.declare_parameter('scan_angle_offset_deg', 180.0)

        # Širina prednjeg konusa u kojem zrake doprinose odbojnom
        # vektoru (npr. 100 = +-50 deg). Bočne/stražnje zrake se
        # ignoriraju za odbijanje.
        self.declare_parameter('pf_front_angle_deg', 100.0)

        # --- Zadržavanje strane zaobilaska (anti-oscilacija) ---
        # Iznad ovog udjela repulsive_gain, odbojni vektor se
        # smatra dovoljno jakim da razmatra/ažurira stranu
        # zaobilaska.
        self.declare_parameter('side_bias_activation_ratio', 0.25)

        # Ako je bočna (y) komponenta manja od ovog udjela ukupne
        # jačine, prepreka se smatra bočno dvosmislenom (skoro
        # točno ispred) i nagne se prema zadnjoj izabranoj strani.
        self.declare_parameter('side_bias_ambiguous_ratio', 0.15)

        # Jačina nagiba prema zadnjoj izabranoj strani (udio
        # ukupne jačine odbijanja).
        self.declare_parameter('side_bias_strength', 0.20)

        # --- Hard-stop failsafe (zadnja linija obrane) ---
        self.declare_parameter('hard_stop_distance', 0.24)
        self.declare_parameter('hard_stop_front_angle_deg', 40.0)

        # Broj uzastopnih skenova s preprekom unutar hard_stop_distance
        # prije nego se hard-stop stvarno aktivira (debounce protiv
        # jedne šumovite zrake).
        self.declare_parameter('hard_stop_confirm_cycles', 2)

        # Broj uzastopnih "čistih" skenova prije nego se hard-stop
        # otpusti (sprječava treperenje stani/kreni na granici).
        self.declare_parameter('hard_stop_release_cycles', 3)

        # --- TF robusnost ---
        # Koliko dugo (sekunde) kontroler smije koristiti zadnju
        # poznatu pozu ako TF (map -> base_frame) lookup ne uspije,
        # prije nego stvarno zaustavi robota. Relevantno za
        # multi-machine postav bez zajedničkog NTP-a gdje TF zna
        # kratkotrajno kasniti/ispasti.
        self.declare_parameter('tf_grace_period', 0.4)

        self.declare_parameter('control_rate', 20.0)

        # =====================================================
        # PARAMETRI - UČITAVANJE
        # =====================================================

        plan_topic = self.get_parameter('plan_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.tf_grace_period = self.get_parameter('tf_grace_period').value
        self.control_rate = self.get_parameter('control_rate').value

        self.pure_pursuit = PurePursuit(
            lookahead_min=self.get_parameter('lookahead_distance_min').value,
            lookahead_max=self.get_parameter('lookahead_distance_max').value,
            lookahead_speed_gain=self.get_parameter(
                'lookahead_speed_gain'
            ).value,
            attractive_gain=self.get_parameter('attractive_gain').value,
            goal_tolerance=self.goal_tolerance,
        )

        self.potential_field = PotentialField(
            influence_radius=self.get_parameter('influence_radius').value,
            min_obstacle_distance=self.get_parameter(
                'min_obstacle_distance'
            ).value,
            repulsive_gain=self.get_parameter('repulsive_gain').value,
            repulsive_smoothing_alpha=self.get_parameter(
                'repulsive_smoothing_alpha'
            ).value,
            scan_stride=self.get_parameter('scan_stride').value,
            scan_angle_offset_deg=self.get_parameter(
                'scan_angle_offset_deg'
            ).value,
            pf_front_angle_deg=self.get_parameter(
                'pf_front_angle_deg'
            ).value,
            hard_stop_distance=self.get_parameter(
                'hard_stop_distance'
            ).value,
            hard_stop_front_angle_deg=self.get_parameter(
                'hard_stop_front_angle_deg'
            ).value,
            hard_stop_confirm_cycles=self.get_parameter(
                'hard_stop_confirm_cycles'
            ).value,
            hard_stop_release_cycles=self.get_parameter(
                'hard_stop_release_cycles'
            ).value,
            side_bias_activation_ratio=self.get_parameter(
                'side_bias_activation_ratio'
            ).value,
            side_bias_ambiguous_ratio=self.get_parameter(
                'side_bias_ambiguous_ratio'
            ).value,
            side_bias_strength=self.get_parameter(
                'side_bias_strength'
            ).value,
        )

        self.shaper = CommandShaper(
            heading_kp=self.get_parameter('heading_kp').value,
            rotation_lock_deadband_deg=self.get_parameter(
                'rotation_lock_deadband_deg'
            ).value,
            rotation_lock_hysteresis_deg=self.get_parameter(
                'rotation_lock_hysteresis_deg'
            ).value,
            rotation_lock_omega_scale=self.get_parameter(
                'rotation_lock_omega_scale'
            ).value,
            max_linear_velocity=self.get_parameter(
                'max_linear_velocity'
            ).value,
            min_linear_velocity=self.get_parameter(
                'min_linear_velocity'
            ).value,
            max_angular_velocity=self.get_parameter(
                'max_angular_velocity'
            ).value,
            max_linear_acceleration=self.get_parameter(
                'max_linear_acceleration'
            ).value,
            max_angular_acceleration=self.get_parameter(
                'max_angular_acceleration'
            ).value,
            slowdown_distance=self.get_parameter(
                'slowdown_distance'
            ).value,
            slowdown_min_factor=self.get_parameter(
                'slowdown_min_factor'
            ).value,
            obstacle_slowdown_min_factor=self.get_parameter(
                'obstacle_slowdown_min_factor'
            ).value,
            control_rate=self.control_rate,
        )

        # =====================================================
        # STANJE
        # =====================================================

        self.finished = True

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.pose_valid_ever = False
        self.last_pose_time = None

        self.last_commanded_v = 0.0

        # =====================================================
        # TF
        # =====================================================

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # =====================================================
        # SUB / PUB
        # =====================================================

        self.plan_sub = self.create_subscription(
            Path, plan_topic, self.plan_callback, 10
        )

        # sensor-data QoS (best-effort, volatile) - kompatibilno s
        # velikom većinom LiDAR drivera bez obzira publiciraju li
        # reliable ili best-effort (reliable subscriber NE bi mogao
        # primati s best-effort publishera).
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data
        )

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # =====================================================
        # CONTROL LOOP
        # =====================================================

        self.timer = self.create_timer(
            1.0 / self.control_rate, self.control_loop
        )

        self.get_logger().info(
            'ASTRO POTENTIAL FIELD PURE PURSUIT (v2) spreman. '
            f'plan={plan_topic} scan={scan_topic} cmd_vel={cmd_vel_topic}. '
            'Čekam putanju na /plan...'
        )

    # =========================================================
    # PLAN CALLBACK
    # =========================================================

    def plan_callback(self, msg: Path):

        if len(msg.poses) == 0:
            self.get_logger().warn('Primio sam praznu putanju. Zanemarujem.')
            return

        self.pure_pursuit.set_path(msg)
        self.finished = False

        self.get_logger().info(
            f'Nova putanja primljena: {len(msg.poses)} waypointa. '
            'Počinjem praćenje (A* + potential field)...'
        )

    # =========================================================
    # SCAN CALLBACK
    # =========================================================

    def scan_callback(self, msg: LaserScan):
        self.potential_field.process_scan(msg)

    # =========================================================
    # ROBOT POSE (TF s grace periodom)
    # =========================================================

    def _update_robot_pose(self) -> bool:

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )

            t = transform.transform.translation
            r = transform.transform.rotation

            self.robot_x = t.x
            self.robot_y = t.y
            self.robot_yaw = quaternion_to_yaw(r.x, r.y, r.z, r.w)
            self.last_pose_time = self.get_clock().now()
            self.pose_valid_ever = True

            return True

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            if not self.pose_valid_ever:
                return False

            elapsed = (
                self.get_clock().now() - self.last_pose_time
            ).nanoseconds / 1e9

            if elapsed < self.tf_grace_period:
                self.get_logger().warn(
                    f'TF nedostupan ({elapsed:.2f}s) - koristim zadnju '
                    'poznatu poziciju.',
                    throttle_duration_sec=1.0,
                )
                return True

            self.get_logger().error(
                'TF izgubljen dulje od grace perioda - zaustavljam robota.',
                throttle_duration_sec=1.0,
            )
            return False

    # =========================================================
    # STOP ROBOT (trenutni, sigurnosni zastoj)
    # =========================================================

    def _stop_robot(self):
        """Trenutni (ne-zaglađeni) zastoj - koristi se za stvarno
        terminalna/sigurnosna stanja (TF izgubljen, cilj dostignut,
        hard-stop, nema puta). Reset rate-limitera osigurava da
        sljedeće kretanje krene glatko od 0, a ne od stare v/omega
        vrijednosti."""

        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.shaper.reset_rate_limiters()
        self.last_commanded_v = 0.0

    # =========================================================
    # CONTROL LOOP
    # =========================================================

    def control_loop(self):

        if not self._update_robot_pose():
            self._stop_robot()
            return

        if self.potential_field.hard_stop_active:
            self.get_logger().warn(
                f'HARD STOP! Prepreka na '
                f'{self.potential_field.min_front_distance:.2f} m ispred.',
                throttle_duration_sec=1.0,
            )
            self._stop_robot()
            return

        if not self.pure_pursuit.has_path() or self.finished:
            self._stop_robot()
            return

        goal_x, goal_y = self.pure_pursuit.goal_point()
        dist_to_goal = math.hypot(
            goal_x - self.robot_x, goal_y - self.robot_y
        )

        if (
            self.pure_pursuit.progress_fraction() >= 0.90
            and dist_to_goal < self.goal_tolerance
        ):
            self.get_logger().info('GOAL DOSTIGNUT! Čekam novu putanju...')
            self.finished = True
            self.pure_pursuit.clear()
            self._stop_robot()
            return

        lookahead_dist = self.pure_pursuit.lookahead_distance_for_speed(
            self.last_commanded_v
        )
        target_x, target_y = self.pure_pursuit.find_lookahead_point(
            self.robot_x, self.robot_y, lookahead_dist
        )

        att_x, att_y = self.pure_pursuit.attractive_vector_local(
            target_x, target_y, self.robot_x, self.robot_y, self.robot_yaw
        )

        comb_x = att_x + self.potential_field.rep_x
        comb_y = att_y + self.potential_field.rep_y

        v, omega = self.shaper.compute(
            comb_x=comb_x,
            comb_y=comb_y,
            dist_to_goal=dist_to_goal,
            obstacle_severity=self.potential_field.severity(),
            fallback_direction=self.potential_field.last_avoidance_side,
        )

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.cmd_vel_pub.publish(cmd)

        self.last_commanded_v = v

        self.get_logger().debug(
            f'att=({att_x:.2f},{att_y:.2f}) '
            f'rep=({self.potential_field.rep_x:.2f},'
            f'{self.potential_field.rep_y:.2f}) '
            f'v={v:.2f} omega={omega:.2f}'
        )


def main(args=None):

    rclpy.init(args=args)
    node = PFPPLocalController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            'Keyboard interrupt. Gasim PFPP lokalni kontroler.'
        )
    finally:
        node._stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

