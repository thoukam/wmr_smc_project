"""#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path


class WMRController(Node):
    def __init__(self):
        super().__init__('wmr_controller')

        # === Paramètres ROS ===
        self.declare_parameter('controller_type', 'NSMC')   # 'NSMC' ou 'BSMC'
        self.declare_parameter('trajectory', 'circle')      # 'circle' ou 'figure8'
        self.declare_parameter('R', 0.6)
        self.declare_parameter('Omega', 0.20)
        self.declare_parameter('Vmax', 0.15)   # vitesses réalistes pour Turtlebot3
        self.declare_parameter('Wmax', 1.8)
        self.declare_parameter('Ts', 0.02)     # période cible (pour info)

        # lecture paramètres
        self.ctrl_type = self.get_parameter(
            'controller_type').get_parameter_value().string_value
        self.traj_type = self.get_parameter(
            'trajectory').get_parameter_value().string_value
        self.R = self.get_parameter('R').get_parameter_value().double_value
        self.Omg = self.get_parameter('Omega').get_parameter_value().double_value
        self.Vmax = self.get_parameter('Vmax').get_parameter_value().double_value
        self.Wmax = self.get_parameter('Wmax').get_parameter_value().double_value
        self.Ts = self.get_parameter('Ts').get_parameter_value().double_value

        # Gains (version plus douce pour la simu)
        self.a11 = 15.0
        self.a12 = 13.0
        self.a21 = 15.0
        self.a22 = 13.0
        self.p11 = 5
        self.p12 = 9
        self.p21 = 7
        self.p22 = 13
        self.q11 = 9
        self.q12 = 5
        self.q21 = 9
        self.q22 = 5
        self.k1 = 3.0
        self.k2 = 5.0
        self.eps = 2.0     # tanh relativement douce

        # état du robot
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.odom_received = False

        # centre de la trajectoire (aligné sur position initiale)
        self.xc = 0.0
        self.yc = 0.0
        self.center_initialized = False

        # pour omega_r numérique
        self.prev_thr = 0.0
        self.omega_r = 0.0

        # temps interne
        self.t = 0.0
        self.prev_time = self.get_clock().now()

        # ROS I/O
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.ref_path_pub = self.create_publisher(Path, '/wmr/ref_path', 10)
        self.robot_path_pub = self.create_publisher(Path, '/wmr/robot_path', 10)

        # chemins pour visualisation
        self.ref_path = Path()
        self.ref_path.header.frame_id = 'odom'
        self.robot_path = Path()
        self.robot_path.header.frame_id = 'odom'

        # timer périodique (mais on utilisera dt réel)
        self.timer = self.create_timer(self.Ts, self.control_loop)
        self.get_logger().info(
            f'WMR controller started: ctrl={self.ctrl_type}, traj={self.traj_type}')

    # ====== Odom callback : récupérer x,y,theta ======
    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y

        q = pose.orientation
        xq, yq, zq, wq = q.x, q.y, q.z, q.w

        # calcul du yaw à partir du quaternion
        sin_yaw = 2.0 * (wq * zq + xq * yq)
        cos_yaw = 1.0 - 2.0 * (yq * yq + zq * zq)
        yaw = math.atan2(sin_yaw, cos_yaw)

        # petit filtrage pour éviter que le bruit odom excite trop le SMC
        if self.odom_received:
            self.th = 0.9 * self.th + 0.1 * yaw

        else:
            self.th = yaw

        self.odom_received = True

        # Alignement du centre de la trajectoire sur la position initiale
        if not self.center_initialized:
            self.xc = self.x
            self.yc = self.y
            self.t = 0.0
            self.center_initialized = True
            self.prev_thr = self.th
            self.prev_time = self.get_clock().now()
            self.get_logger().info(
                f'Centre de la trajectoire fixé à xc={self.xc:.2f}, yc={self.yc:.2f}')

    # ====== Fonctions utilitaires ======
    def sigmoid(self, s: float) -> float:
        # version bornée et stable numériquement
        # on sature la surface pour éviter les énormes exponentielles
        s = max(-2.0, min(2.0, s))
        return math.tanh(0.5 * self.eps * s)

    def angle_wrap(self, a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def yaw_to_quat(self, yaw: float):
        half = 0.5 * yaw
        cz = math.cos(half)
        sz = math.sin(half)
        return (0.0, 0.0, sz, cz)  # (x,y,z,w)

    # ====== Référence cercle / 8 (alignée sur xc,yc) ======
    def reference(self, t: float):
        if self.traj_type == 'circle':
            xr = self.xc + self.R * math.cos(self.Omg * t)
            yr = self.yc + self.R * math.sin(self.Omg * t)
            dx = -self.R * self.Omg * math.sin(self.Omg * t)
            dy = self.R * self.Omg * math.cos(self.Omg * t)
            thr = math.atan2(dy, dx)
            vr = math.hypot(dx, dy)
            omegar = self.Omg
        else:  # figure8 Gerono
            xr = self.xc + self.R * math.sin(self.Omg * t)
            yr = self.yc + self.R * math.sin(self.Omg * t) * math.cos(self.Omg * t)
            dx = self.R * self.Omg * math.cos(self.Omg * t)
            dy = self.R * self.Omg * math.cos(2.0 * self.Omg * t)
            thr = math.atan2(dy, dx)
            vr = math.hypot(dx, dy)
            omegar = float('nan')  # dérivée numérique
        return xr, yr, thr, vr, omegar

    # ====== NSMC ======
    def ctrl_nsmc(self, ex, ey, et, vr, omegar):
        s1 = et
        s2 = self.k1 * ex - self.k2 * omegar * ey

        sig1 = self.sigmoid(s1)
        sig2 = self.sigmoid(s2)

        omega = omegar \
            + self.a11 * (abs(s1) ** (self.p11 / self.q11)) * sig1 \
            + self.a12 * (abs(s1) ** (self.p12 / self.q12)) * sig1

        nu = ey * omegar + vr \
            + (1.0 / self.k1) * (
                self.k2 * (omegar ** 2) * ex
                + self.a21 * (abs(s2) ** (self.p21 / self.q21)) * sig2
                + self.a22 * (abs(s2) ** (self.p22 / self.q22)) * sig2
            )
        return nu, omega

    # ====== BSMC ======
    def ctrl_bsmc(self, ex, ey, et, vr, omegar):
        s1 = ex
        psi = math.atan(vr * ey)
        s2 = et + psi

        dpsi_dye = vr / (1.0 + (vr * ey) ** 2)
        dpsi_dvr = ey / (1.0 + (vr * ey) ** 2)
        dvr = 0.0  # simplification

        sig1 = self.sigmoid(s1)
        sig2 = self.sigmoid(s2)

        nu = ey * omegar + vr * math.cos(et) \
            + self.a11 * (abs(s1) ** (self.p11 / self.q11)) * sig1 \
            + self.a12 * (abs(s1) ** (self.p12 / self.q12)) * sig1

        omega = omegar \
            + dpsi_dye * vr * math.sin(et) + dpsi_dvr * dvr \
            + self.a21 * (abs(s2) ** (self.p21 / self.q21)) * sig2 \
            + self.a22 * (abs(s2) ** (self.p22 / self.q22)) * sig2
        return nu, omega

    # ====== Boucle de contrôle principale ======
    def control_loop(self):
        if not self.odom_received or not self.center_initialized:
            return   # on attend la première odom et l'init du centre

        # dt réel (le timer ROS n'est pas exactement Ts)
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 0.2:  # gros saut de temps -> on saute un tour
            self.prev_time = now
            return
        self.prev_time = now
        self.t += dt

        # --- Référence ---
        xr, yr, thr_ref, vr, omegar_ref = self.reference(self.t)

        # omega_r
        if self.traj_type == 'circle':
            self.omega_r = omegar_ref
        else:
            dth = self.angle_wrap(thr_ref - self.prev_thr)
            self.omega_r = dth / dt
        self.prev_thr = thr_ref

        # --- Erreurs locales ---
        ct = math.cos(self.th)
        st = math.sin(self.th)
        ex = (xr - self.x) * ct + (yr - self.y) * st
        ey = -(xr - self.x) * st + (yr - self.y) * ct
        et = self.angle_wrap(thr_ref - self.th)

        # --- Choix du contrôleur ---
        if self.ctrl_type.upper() == 'NSMC':
            nu, omega = self.ctrl_nsmc(ex, ey, et, vr, self.omega_r)
        else:
            nu, omega = self.ctrl_bsmc(ex, ey, et, vr, self.omega_r)

        # --- Saturations (sécurité Turtlebot) ---
        nu = max(-self.Vmax, min(self.Vmax, nu))
        omega = max(-self.Wmax, min(self.Wmax, omega))

        # --- Publication cmd_vel ---
        cmd = Twist()
        cmd.linear.x = float(nu)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

        # --- Mise à jour des chemins pour RViz ---
        now_msg = now.to_msg()

        # référence
        ref_pose = PoseStamped()
        ref_pose.header.stamp = now_msg
        ref_pose.header.frame_id = 'odom'
        ref_pose.pose.position.x = xr
        ref_pose.pose.position.y = yr
        qx, qy, qz, qw = self.yaw_to_quat(thr_ref)
        ref_pose.pose.orientation.x = qx
        ref_pose.pose.orientation.y = qy
        ref_pose.pose.orientation.z = qz
        ref_pose.pose.orientation.w = qw
        self.ref_path.header.stamp = now_msg
        self.ref_path.poses.append(ref_pose)

        # robot
        rob_pose = PoseStamped()
        rob_pose.header.stamp = now_msg
        rob_pose.header.frame_id = 'odom'
        rob_pose.pose.position.x = self.x
        rob_pose.pose.position.y = self.y
        qx, qy, qz, qw = self.yaw_to_quat(self.th)
        rob_pose.pose.orientation.x = qx
        rob_pose.pose.orientation.y = qy
        rob_pose.pose.orientation.z = qz
        rob_pose.pose.orientation.w = qw
        self.robot_path.header.stamp = now_msg
        self.robot_path.poses.append(rob_pose)

        self.ref_path_pub.publish(self.ref_path)
        self.robot_path_pub.publish(self.robot_path)


def main(args=None):
    rclpy.init(args=args)
    node = WMRController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
"""

"""
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path


class WMRController(Node):
    def __init__(self):
        super().__init__('wmr_controller')

        # === Paramètres ROS ===
        self.declare_parameter('controller_type', 'NSMC')   # 'NSMC' ou 'BSMC'
        self.declare_parameter('trajectory', 'circle')      # 'circle' ou 'figure8'
        self.declare_parameter('R', 0.6)
        self.declare_parameter('Omega', 0.20)
        self.declare_parameter('Vmax', 0.22)   # Vitesse standard pour le rattrapage
        self.declare_parameter('Wmax', 1.8)
        self.declare_parameter('Ts', 0.02)

        # lecture paramètres
        self.ctrl_type = self.get_parameter(
            'controller_type').get_parameter_value().string_value
        self.traj_type = self.get_parameter(
            'trajectory').get_parameter_value().string_value
        self.R = self.get_parameter('R').get_parameter_value().double_value
        self.Omg = self.get_parameter('Omega').get_parameter_value().double_value
        self.Vmax = self.get_parameter('Vmax').get_parameter_value().double_value
        self.Wmax = self.get_parameter('Wmax').get_parameter_value().double_value
        self.Ts = self.get_parameter('Ts').get_parameter_value().double_value

        # Gains NSMC (Optimisation pour une meilleure convergence après 1 tour)
        # Augmentation des gains angulaires (a11, a12)
        self.a11 = 8.0     # Force de correction angulaire AUGMENTÉE
        self.a12 = 5.0     # Force de correction angulaire AUGMENTÉE
        self.a21 = 4.0     # Force de correction linéaire/latérale
        self.a22 = 3.0
        
        # Exposants (tirés du papier)
        self.p11 = 5
        self.p12 = 9
        self.p21 = 7
        self.p22 = 13
        self.q11 = 9
        self.q12 = 5
        self.q21 = 9
        self.q22 = 5
        
        # Poids dans s2 (tirés du papier: k1=10, k2=15, mais nous utilisons des valeurs plus modérées pour ROS)
        self.k1 = 5.0      # Poids erreur longitudinale (ex)
        self.k2 = 7.0      # Poids erreur latérale (ey)
        self.eps = 2.0     # Douceur de la fonction tanh (anti-chattering, réduire la valeur si trop de chattering)

        # état du robot
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.odom_received = False

        # centre de la trajectoire (aligné sur position initiale)
        self.xc = 0.0
        self.yc = 0.0
        self.center_initialized = False

        # pour omega_r numérique
        self.prev_thr = 0.0
        self.omega_r = 0.0

        # temps interne
        self.t = 0.0
        self.prev_time = self.get_clock().now()

        # ROS I/O
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.ref_path_pub = self.create_publisher(Path, '/wmr/ref_path', 10)
        self.robot_path_pub = self.create_publisher(Path, '/wmr/robot_path', 10)

        # chemins pour visualisation
        self.ref_path = Path()
        self.ref_path.header.frame_id = 'odom'
        self.robot_path = Path()
        self.robot_path.header.frame_id = 'odom'

        # timer périodique (mais on utilisera dt réel)
        self.timer = self.create_timer(self.Ts, self.control_loop)
        self.get_logger().info(
            f'WMR controller started: ctrl={self.ctrl_type}, traj={self.traj_type}')

    # ====== Odom callback : récupérer x,y,theta ======
    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y

        q = pose.orientation
        xq, yq, zq, wq = q.x, q.y, q.z, q.w

        # calcul du yaw à partir du quaternion
        sin_yaw = 2.0 * (wq * zq + xq * yq)
        cos_yaw = 1.0 - 2.0 * (yq * yq + zq * zq)
        yaw = math.atan2(sin_yaw, cos_yaw)

        # CORRECTION MAJEURE: On enlève le LPF d'angle pour éviter le déphasage
        self.th = yaw 
        self.odom_received = True

        # Alignement du centre de la trajectoire sur la position initiale
        if not self.center_initialized:
            self.xc = self.x
            self.yc = self.y
            self.t = 0.0
            self.center_initialized = True
            self.prev_thr = self.th
            self.prev_time = self.get_clock().now()
            self.get_logger().info(
                f'Centre de la trajectoire fixé à xc={self.xc:.2f}, yc={self.yc:.2f}')

    # ====== Fonctions utilitaires ======
    def sigmoid(self, s: float) -> float:
        # Correspond à la fonction tanh dans la pratique (proche de la saturation fonctionnelle)
        # On sature la surface pour éviter les énormes exponentielles.
        s = max(-2.0, min(2.0, s)) # Limitation manuelle pour la stabilité numérique
        # Utilisation de tanh (similaire à sigmoid, avec de meilleures propriétés numériques)
        return math.tanh(0.5 * self.eps * s) 

    def angle_wrap(self, a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def yaw_to_quat(self, yaw: float):
        half = 0.5 * yaw
        cz = math.cos(half)
        sz = math.sin(half)
        return (0.0, 0.0, sz, cz)  # (x,y,z,w)

    # ====== Référence cercle / 8 (alignée sur xc,yc) ======
    def reference(self, t: float):
        if self.traj_type == 'circle':
            xr = self.xc + self.R * math.cos(self.Omg * t)
            yr = self.yc + self.R * math.sin(self.Omg * t)
            dx = -self.R * self.Omg * math.sin(self.Omg * t)
            dy = self.R * self.Omg * math.cos(self.Omg * t)
            thr = math.atan2(dy, dx)
            vr = math.hypot(dx, dy)
            omegar = self.Omg
        else:  # figure8 Gerono
            xr = self.xc + self.R * math.sin(self.Omg * t)
            yr = self.yc + self.R * math.sin(self.Omg * t) * math.cos(self.Omg * t)
            dx = self.R * self.Omg * math.cos(self.Omg * t)
            dy = self.R * self.Omg * math.cos(2.0 * self.Omg * t)
            thr = math.atan2(dy, dx)
            vr = math.hypot(dx, dy)
            omegar = float('nan')  # dérivée numérique
        return xr, yr, thr, vr, omegar

    # ====== NSMC (Novel Sliding Mode Controller) ======
    def ctrl_nsmc(self, ex, ey, et, vr, omegar):
        s1 = et
        s2 = self.k1 * ex - self.k2 * omegar * ey # Eq 18
        
        # Fonctions de switching (avec approximation continue)
        sig1 = self.sigmoid(s1)
        sig2 = self.sigmoid(s2)

        # Calcul de Omega (vitesse angulaire) - Eq 42
        # s1 = et
        omega = omegar \
            + self.a11 * (abs(s1) ** (self.p11 / self.q11)) * sig1 \
            + self.a12 * (abs(s1) ** (self.p12 / self.q12)) * sig1

        # Calcul de Nu (vitesse linéaire) - Eq 42
        nu = ey * omegar + vr \
            + (1.0 / self.k1) * (
                self.k2 * (omegar ** 2) * ex  # Terme k2*wr²*xe/k1 du papier
                + self.a21 * (abs(s2) ** (self.p21 / self.q21)) * sig2
                + self.a22 * (abs(s2) ** (self.p22 / self.q22)) * sig2
            )
        return nu, omega

    # ====== BSMC (Backstepping Sliding Mode Controller) - Non modifiée ======
    def ctrl_bsmc(self, ex, ey, et, vr, omegar):
        s1 = ex
        psi = math.atan(vr * ey)
        s2 = et + psi
        
        # Dérivées partielles (Eq 638)
        dpsi_dye = vr / (1.0 + (vr * ey) ** 2)
        dpsi_dvr = ey / (1.0 + (vr * ey) ** 2)
        dvr = 0.0  # Simplification (dérivée de vr par rapport au temps)

        sig1 = self.sigmoid(s1)
        sig2 = self.sigmoid(s2)

        # Calcul de Nu (vitesse linéaire) - Eq 41
        nu = ey * omegar + vr * math.cos(et) \
            + self.a11 * (abs(s1) ** (self.p11 / self.q11)) * sig1 \
            + self.a12 * (abs(s1) ** (self.p12 / self.q12)) * sig1

        # Calcul de Omega (vitesse angulaire) - Eq 41
        omega = omegar \
            + dpsi_dye * vr * math.sin(et) + dpsi_dvr * dvr \
            + self.a21 * (abs(s2) ** (self.p21 / self.q21)) * sig2 \
            + self.a22 * (abs(s2) ** (self.p22 / self.q22)) * sig2
        return nu, omega

    # ====== Boucle de contrôle principale ======
    def control_loop(self):
        if not self.odom_received or not self.center_initialized:
            return   # on attend la première odom et l'init du centre

        # dt réel (le timer ROS n'est pas exactement Ts)
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        
        # self.get_logger().info(f'dt: {dt:.4f}') # Déplacement de la ligne pour le debug

        if dt <= 0.0 or dt > 0.2:  # gros saut de temps -> on saute un tour
            self.prev_time = now
            return
            
        self.prev_time = now
        self.t += dt

        # --- Référence ---
        xr, yr, thr_ref, vr, omegar_ref = self.reference(self.t)

        # omega_r
        if self.traj_type == 'circle':
            self.omega_r = omegar_ref
        else:
            dth = self.angle_wrap(thr_ref - self.prev_thr)
            self.omega_r = dth / dt
        self.prev_thr = thr_ref

        # --- Erreurs locales ---
        ct = math.cos(self.th)
        st = math.sin(self.th)
        # Ex = (Xr - X) * cos(theta) + (Yr - Y) * sin(theta)
        ex = (xr - self.x) * ct + (yr - self.y) * st 
        # Ey = -(Xr - X) * sin(theta) + (Yr - Y) * cos(theta)
        ey = -(xr - self.x) * st + (yr - self.y) * ct
        # Et = angle_wrap(Thr - Theta)
        et = self.angle_wrap(thr_ref - self.th)

        # --- Choix du contrôleur ---
        if self.ctrl_type.upper() == 'NSMC':
            nu, omega = self.ctrl_nsmc(ex, ey, et, vr, self.omega_r)
        else:
            nu, omega = self.ctrl_bsmc(ex, ey, et, vr, self.omega_r)

        # --- Saturations (sécurité Turtlebot) ---
        nu = max(-self.Vmax, min(self.Vmax, nu))
        omega = max(-self.Wmax, min(self.Wmax, omega))

        # --- Publication cmd_vel ---
        cmd = Twist()
        cmd.linear.x = float(nu)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

        # --- Mise à jour des chemins pour RViz ---
        now_msg = now.to_msg()

        # référence
        ref_pose = PoseStamped()
        ref_pose.header.stamp = now_msg
        ref_pose.header.frame_id = 'odom'
        ref_pose.pose.position.x = xr
        ref_pose.pose.position.y = yr
        qx, qy, qz, qw = self.yaw_to_quat(thr_ref)
        ref_pose.pose.orientation.x = qx
        ref_pose.pose.orientation.y = qy
        ref_pose.pose.orientation.z = qz
        ref_pose.pose.orientation.w = qw
        self.ref_path.header.stamp = now_msg
        self.ref_path.poses.append(ref_pose)

        # robot
        rob_pose = PoseStamped()
        rob_pose.header.stamp = now_msg
        rob_pose.header.frame_id = 'odom'
        rob_pose.pose.position.x = self.x
        rob_pose.pose.position.y = self.y
        qx, qy, qz, qw = self.yaw_to_quat(self.th)
        rob_pose.pose.orientation.x = qx
        rob_pose.pose.orientation.y = qy
        rob_pose.pose.orientation.z = qz
        rob_pose.pose.orientation.w = qw
        self.robot_path.header.stamp = now_msg
        self.robot_path.poses.append(rob_pose)

        self.ref_path_pub.publish(self.ref_path)
        self.robot_path_pub.publish(self.robot_path)


def main(args=None):
    rclpy.init(args=args)
    node = WMRController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    
"""

#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path


class WMRController(Node):
    def __init__(self):
        super().__init__('wmr_controller')

        # === Paramètres ROS ===
        self.declare_parameter('controller_type', 'NSMC')   # 'NSMC' ou 'BSMC'
        self.declare_parameter('trajectory', 'circle')      # 'circle' ou 'figure8'
        self.declare_parameter('R', 0.6)
        self.declare_parameter('Omega', 0.20)
        self.declare_parameter('Vmax', 0.30)   # vitesses réalistes pour Turtlebot3
        self.declare_parameter('Wmax', 1.8)
        self.declare_parameter('Ts', 0.02)

        # perturbations optionnelles
        self.declare_parameter('use_disturbance', False)
        self.declare_parameter('dist_lin', 0.0)   # amplitude pertubation sur nu
        self.declare_parameter('dist_ang', 0.0)   # amplitude pertubation sur omega

        # lecture paramètres
        self.ctrl_type = self.get_parameter(
            'controller_type').get_parameter_value().string_value
        self.traj_type = self.get_parameter(
            'trajectory').get_parameter_value().string_value
        self.R = self.get_parameter('R').get_parameter_value().double_value
        self.Omg = self.get_parameter('Omega').get_parameter_value().double_value
        self.Vmax = self.get_parameter('Vmax').get_parameter_value().double_value
        self.Wmax = self.get_parameter('Wmax').get_parameter_value().double_value
        self.Ts = self.get_parameter('Ts').get_parameter_value().double_value

        self.use_dist = self.get_parameter(
            'use_disturbance').get_parameter_value().bool_value
        self.dist_lin = self.get_parameter(
            'dist_lin').get_parameter_value().double_value
        self.dist_ang = self.get_parameter(
            'dist_ang').get_parameter_value().double_value

        # Gains NSMC
        self.a11 = 8.0
        self.a12 = 5.0
        self.a21 = 4.0
        self.a22 = 3.0

        self.p11 = 5
        self.p12 = 9
        self.p21 = 7
        self.p22 = 13
        self.q11 = 9
        self.q12 = 5
        self.q21 = 9
        self.q22 = 5

        self.k1 = 5.0
        self.k2 = 7.0
        self.eps = 2.0     # douceur tanh

        # état du robot
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.odom_received = False

        # centre de la trajectoire FIXE : (0,0) dans 'odom'
        self.xc = 0.0
        self.yc = 0.0
        self.center_initialized = False

        # pour omega_r numérique
        self.prev_thr = 0.0
        self.omega_r = 0.0

        # temps interne
        self.t = 0.0
        self.prev_time = self.get_clock().now()

        # ROS I/O
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.ref_path_pub = self.create_publisher(Path, '/wmr/ref_path', 10)
        self.robot_path_pub = self.create_publisher(Path, '/wmr/robot_path', 10)

        # chemins pour visualisation
        self.ref_path = Path()
        self.ref_path.header.frame_id = 'odom'
        self.robot_path = Path()
        self.robot_path.header.frame_id = 'odom'

        # timer périodique (mais on utilisera dt réel)
        self.timer = self.create_timer(self.Ts, self.control_loop)
        self.get_logger().info(
            f'WMR controller started: ctrl={self.ctrl_type}, traj={self.traj_type}')

    # ====== Odom callback : récupérer x,y,theta ======
    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y

        q = pose.orientation
        xq, yq, zq, wq = q.x, q.y, q.z, q.w

        # calcul du yaw à partir du quaternion
        sin_yaw = 2.0 * (wq * zq + xq * yq)
        cos_yaw = 1.0 - 2.0 * (yq * yq + zq * zq)
        yaw = math.atan2(sin_yaw, cos_yaw)

        # pas de filtrage pour éviter le déphasage
        self.th = yaw
        self.odom_received = True

        # On attend la première odom pour initialiser le temps,
        # le centre reste (0,0)
        if not self.center_initialized:
            self.xc = self.x
            self.yc = self.y
            self.t = 0.0
            self.center_initialized = True
            self.prev_thr = self.th
            self.prev_time = self.get_clock().now()
            self.get_logger().info(
                f'Centre de la trajectoire fixé à xc={self.xc:.2f}, yc={self.yc:.2f}')

    # ====== Fonctions utilitaires ======
    def sigmoid(self, s: float) -> float:
        s = max(-2.0, min(2.0, s))
        return math.tanh(0.5 * self.eps * s)

    def angle_wrap(self, a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def yaw_to_quat(self, yaw: float):
        half = 0.5 * yaw
        cz = math.cos(half)
        sz = math.sin(half)
        return (0.0, 0.0, sz, cz)  # (x,y,z,w)

    # ====== Référence cercle / 8 (centre (0,0) dans 'odom') ======
    def reference(self, t: float):
        if self.traj_type == 'circle':
            xr = self.xc + self.R * math.cos(self.Omg * t)
            yr = self.yc + self.R * math.sin(self.Omg * t)
            dx = -self.R * self.Omg * math.sin(self.Omg * t)
            dy = self.R * self.Omg * math.cos(self.Omg * t)
            thr = math.atan2(dy, dx)
            vr = math.hypot(dx, dy)
            omegar = self.Omg
        else:  # figure8 Gerono
            xr = self.xc + self.R * math.sin(self.Omg * t)
            yr = self.yc + self.R * math.sin(self.Omg * t) * math.cos(self.Omg * t)
            dx = self.R * self.Omg * math.cos(self.Omg * t)
            dy = self.R * self.Omg * math.cos(2.0 * self.Omg * t)
            thr = math.atan2(dy, dx)
            vr = math.hypot(dx, dy)
            omegar = float('nan')  # dérivée numérique
        return xr, yr, thr, vr, omegar

    # ====== NSMC ======
    def ctrl_nsmc(self, ex, ey, et, vr, omegar):
        s1 = et
        s2 = self.k1 * ex - self.k2 * omegar * ey

        sig1 = self.sigmoid(s1)
        sig2 = self.sigmoid(s2)

        omega = omegar \
            + self.a11 * (abs(s1) ** (self.p11 / self.q11)) * sig1 \
            + self.a12 * (abs(s1) ** (self.p12 / self.q12)) * sig1

        nu = ey * omegar + vr \
            + (1.0 / self.k1) * (
                self.k2 * (omegar ** 2) * ex
                + self.a21 * (abs(s2) ** (self.p21 / self.q21)) * sig2
                + self.a22 * (abs(s2) ** (self.p22 / self.q22)) * sig2
            )
        return nu, omega

    # ====== BSMC ======
    def ctrl_bsmc(self, ex, ey, et, vr, omegar):
        s1 = ex
        psi = math.atan(vr * ey)
        s2 = et + psi

        dpsi_dye = vr / (1.0 + (vr * ey) ** 2)
        dpsi_dvr = ey / (1.0 + (vr * ey) ** 2)
        dvr = 0.0  # simplification

        sig1 = self.sigmoid(s1)
        sig2 = self.sigmoid(s2)

        nu = ey * omegar + vr * math.cos(et) \
            + self.a11 * (abs(s1) ** (self.p11 / self.q11)) * sig1 \
            + self.a12 * (abs(s1) ** (self.p12 / self.q12)) * sig1

        omega = omegar \
            + dpsi_dye * vr * math.sin(et) + dpsi_dvr * dvr \
            + self.a21 * (abs(s2) ** (self.p21 / self.q21)) * sig2 \
            + self.a22 * (abs(s2) ** (self.p22 / self.q22)) * sig2
        return nu, omega

    # ====== Boucle de contrôle principale ======
    def control_loop(self):
        if not self.odom_received or not self.center_initialized:
            return   # on attend la première odom et l'init du centre

        # dt réel
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9

        if dt <= 0.0 or dt > 0.2:
            self.prev_time = now
            return

        self.prev_time = now
        self.t += dt

        # --- Référence ---
        xr, yr, thr_ref, vr, omegar_ref = self.reference(self.t)

        # omega_r
        if self.traj_type == 'circle':
            self.omega_r = omegar_ref
        else:
            dth = self.angle_wrap(thr_ref - self.prev_thr)
            self.omega_r = dth / dt
        self.prev_thr = thr_ref

        # --- Erreurs locales ---
        ct = math.cos(self.th)
        st = math.sin(self.th)
        ex = (xr - self.x) * ct + (yr - self.y) * st
        ey = -(xr - self.x) * st + (yr - self.y) * ct
        et = self.angle_wrap(thr_ref - self.th)

        # --- Choix du contrôleur ---
        if self.ctrl_type.upper() == 'NSMC':
            nu, omega = self.ctrl_nsmc(ex, ey, et, vr, self.omega_r)
        else:
            nu, omega = self.ctrl_bsmc(ex, ey, et, vr, self.omega_r)

        # --- Perturbations optionnelles ---
        if self.use_dist:
            # sinus en fonction du temps interne t
            d1 = self.dist_lin * math.sin(self.t)
            d2 = self.dist_ang * math.sin(self.t)
            nu += d1
            omega += d2

        # --- Saturations (sécurité Turtlebot) ---
        nu = max(-self.Vmax, min(self.Vmax, nu))
        omega = max(-self.Wmax, min(self.Wmax, omega))

        # --- Publication cmd_vel ---
        cmd = Twist()
        cmd.linear.x = float(nu)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

        # --- Mise à jour des chemins pour RViz ---
        now_msg = now.to_msg()

        # référence
        ref_pose = PoseStamped()
        ref_pose.header.stamp = now_msg
        ref_pose.header.frame_id = 'odom'
        ref_pose.pose.position.x = xr
        ref_pose.pose.position.y = yr
        qx, qy, qz, qw = self.yaw_to_quat(thr_ref)
        ref_pose.pose.orientation.x = qx
        ref_pose.pose.orientation.y = qy
        ref_pose.pose.orientation.z = qz
        ref_pose.pose.orientation.w = qw
        self.ref_path.header.stamp = now_msg
        self.ref_path.poses.append(ref_pose)

        # robot
        rob_pose = PoseStamped()
        rob_pose.header.stamp = now_msg
        rob_pose.header.frame_id = 'odom'
        rob_pose.pose.position.x = self.x
        rob_pose.pose.position.y = self.y
        qx, qy, qz, qw = self.yaw_to_quat(self.th)
        rob_pose.pose.orientation.x = qx
        rob_pose.pose.orientation.y = qy
        rob_pose.pose.orientation.z = qz
        rob_pose.pose.orientation.w = qw
        self.robot_path.header.stamp = now_msg
        self.robot_path.poses.append(rob_pose)

        self.ref_path_pub.publish(self.ref_path)
        self.robot_path_pub.publish(self.robot_path)


def main(args=None):
    rclpy.init(args=args)
    node = WMRController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
