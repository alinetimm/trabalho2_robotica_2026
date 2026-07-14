from collections import namedtuple
from datetime import datetime
import csv
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


LOG_DIR = '/ros_ws/logs'
CSV_HEADER = ['t', 'x', 'y', 'yaw', 'estado', 'waypoint', 'dist',
              'fm', 'fleft', 'fright', 'sleft', 'sright', 'v', 'w']


Waypoint = namedtuple('Waypoint', ['name', 'x', 'y', 'tol', 'is_target'])


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.laser_cb, qos_profile_sensor_data)

        self._load_params()
        self._init_csv_log()

        self.rx, self.ry, self.rt = self.x0, self.y0, self.theta0
        self.first_odom = True
        self.ox0, self.oy0, self.ot0 = 0.0, 0.0, 0.0

        self.widx = 0
        self.ml_sx, self.ml_sy = self.x0, self.y0

        self.ranges = []
        self.amin = 0.0
        self.ainc = 0.0
        self.lok = False

        self.state = 'GO_TO_GOAL'
        self.wdir = 1
        self.hit_dist = 999.0
        self.hit_x = 0.0
        self.hit_y = 0.0
        self.fticks = 0

        self.last_leave_tick = -1000
        self.last_wdir = 1

        self.n_contornos = 0
        self.wp_times = []
        self.wp_start_time = None
        self.mission_start_time = None
        self.wait_until = None

        self.timer = self.create_timer(0.1, self.control)
        self.tick = 0
        self.get_logger().info('=== Navigator (Bug2 + LoS + missao) iniciado ===')
        self.get_logger().info(
            f'Pose inicial: ({self.x0}, {self.y0}) theta={math.degrees(self.theta0):.0f}deg')
        self.get_logger().info('Missao: ' + ' -> '.join(w.name for w in self.mission))

    def _load_params(self):
        DBL = rclpy.Parameter.Type.DOUBLE
        STRARR = rclpy.Parameter.Type.STRING_ARRAY

        self.declare_parameter('pose0.x', DBL)
        self.declare_parameter('pose0.y', DBL)
        self.declare_parameter('pose0.theta_deg', DBL)
        self.x0 = self.get_parameter('pose0.x').value
        self.y0 = self.get_parameter('pose0.y').value
        self.theta0 = math.radians(self.get_parameter('pose0.theta_deg').value)

        self.declare_parameter('home.x', DBL)
        self.declare_parameter('home.y', DBL)
        self.declare_parameter('home.tol', DBL)
        home = Waypoint('home',
                         self.get_parameter('home.x').value,
                         self.get_parameter('home.y').value,
                         self.get_parameter('home.tol').value,
                         False)

        self.declare_parameter('target_order', STRARR)
        order = self.get_parameter('target_order').value

        self.mission = []
        for name in order:
            self.declare_parameter(f'targets.{name}.x', DBL)
            self.declare_parameter(f'targets.{name}.y', DBL)
            self.declare_parameter(f'targets.{name}.tol', DBL)
            self.mission.append(Waypoint(
                name,
                self.get_parameter(f'targets.{name}.x').value,
                self.get_parameter(f'targets.{name}.y').value,
                self.get_parameter(f'targets.{name}.tol').value,
                True))
            self.mission.append(home)

        self.declare_parameter('bug2.d_obs', DBL)
        self.declare_parameter('bug2.wall_dist', DBL)
        self.declare_parameter('bug2.mline_tol', DBL)
        self.declare_parameter('bug2.lin', DBL)
        self.declare_parameter('bug2.ang', DBL)
        self.D_OBS = self.get_parameter('bug2.d_obs').value
        self.WALL_DIST = self.get_parameter('bug2.wall_dist').value
        self.MLINE_TOL = self.get_parameter('bug2.mline_tol').value
        self.LIN = self.get_parameter('bug2.lin').value
        self.ANG = self.get_parameter('bug2.ang').value

        self.declare_parameter('approach.window_dist', DBL)
        self.declare_parameter('approach.max_speed', DBL)
        self.declare_parameter('approach.safety_fm', DBL)
        self.approach_window = self.get_parameter('approach.window_dist').value
        self.approach_max_speed = self.get_parameter('approach.max_speed').value
        self.safety_fm = self.get_parameter('approach.safety_fm').value

        self.declare_parameter('wait_time', DBL)
        self.wait_time = self.get_parameter('wait_time').value

        self.declare_parameter('log_csv', True)
        self.log_csv = self.get_parameter('log_csv').value

    def _init_csv_log(self):
        self.csv_file = None
        self.csv_writer = None
        if not self.log_csv:
            return
        os.makedirs(LOG_DIR, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(LOG_DIR, f'mission_{ts}.csv')
        self.csv_file = open(path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(CSV_HEADER)
        self.get_logger().info(f'Log CSV: {path}')

    def close_log(self):
        if self.csv_file:
            self.csv_file.close()

    def odom_cb(self, msg):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        ot = 2.0 * math.atan2(msg.pose.pose.orientation.z,
                              msg.pose.pose.orientation.w)
        if self.first_odom:
            self.ox0, self.oy0, self.ot0 = ox, oy, ot
            self.first_odom = False
            self.get_logger().info(f'---> ot0 (yaw inicial do odom): {self.ot0:.3f}')

        dx, dy = ox - self.ox0, oy - self.oy0
        a = self.theta0 - self.ot0
        self.rx = self.x0 + math.cos(a) * dx - math.sin(a) * dy
        self.ry = self.y0 + math.sin(a) * dx + math.cos(a) * dy
        self.rt = self._na(ot - self.ot0 + self.theta0)

    def laser_cb(self, msg):
        self.ranges = list(msg.ranges)
        self.amin = msg.angle_min
        self.ainc = msg.angle_increment
        self.lok = True

    @staticmethod
    def _na(a):
        while a > math.pi:  a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def _angle_to_idx(self, angle_rad):
        n = len(self.ranges)
        raw = (angle_rad - self.amin) / self.ainc
        return int(round(raw)) % n

    def _sector_min(self, deg_min, deg_max):
        if not self.ranges:
            return 999.0
        i1 = self._angle_to_idx(math.radians(deg_min))
        i2 = self._angle_to_idx(math.radians(deg_max))
        if i1 <= i2:
            sec = self.ranges[i1:i2 + 1]
        else:
            sec = self.ranges[i1:] + self.ranges[:i2 + 1]
        v = [r for r in sec if 0.05 < r < 30.0
             and not math.isinf(r) and not math.isnan(r)]
        return min(v) if v else 999.0

    def _goal_info(self):
        wp = self.mission[self.widx]
        dx, dy = wp.x - self.rx, wp.y - self.ry
        d = math.hypot(dx, dy)
        ga = math.atan2(dy, dx)
        gr = self._na(ga - self.rt)
        return d, ga, gr

    def _on_mline(self, wp):
        sx, sy = self.ml_sx, self.ml_sy
        ll = math.hypot(wp.x - sx, wp.y - sy)
        if ll < 0.01:
            return True
        d = abs((wp.y - sy) * (self.rx - sx) - (wp.x - sx) * (self.ry - sy)) / ll
        return d < self.MLINE_TOL

    def _line_of_sight_to_goal(self, ga_r, dist):
        center_deg = math.degrees(ga_r)
        if abs(center_deg) > 120:
            return False
        sec_min = self._sector_min(center_deg - 20, center_deg + 20)
        return sec_min > min(dist - 0.2, 3.0)

    def control(self):
        if not self.lok:
            return
        self.tick += 1

        if self.wp_start_time is None:
            self.wp_start_time = self.get_clock().now()
            self.mission_start_time = self.wp_start_time

        fm     = self._sector_min(-25,   25)
        fleft  = self._sector_min( 15,   60)
        fright = self._sector_min(-60,  -15)
        sleft  = self._sector_min( 60,  120)
        sright = self._sector_min(-120, -60)

        tw = Twist()
        wp_name = 'DONE'
        dist = 0.0

        if self.state == 'WAITING':
            wp = self.mission[self.widx]
            wp_name = wp.name
            dist, _, _ = self._goal_info()
            if self.get_clock().now() >= self.wait_until:
                self._advance_waypoint()

        elif self.state != 'DONE':
            wp = self.mission[self.widx]
            wp_name = wp.name
            dist, ga_w, ga_r = self._goal_info()

            if dist < wp.tol:
                self._arrive(wp, dist)
            else:
                in_window = wp.is_target and dist < self.approach_window
                if in_window:
                    tw = self._approach_final(wp, ga_r, fm, sleft, sright)
                else:
                    tw = self._bug2_step(wp, dist, ga_r, fm, fleft, fright, sleft, sright)

                if self.tick % 30 == 0:
                    ml = 'ML' if self._on_mline(wp) else '--'
                    self.get_logger().info(
                        f'[{self.state}] alvo={wp.name} ({self.rx:.1f},{self.ry:.1f}) '
                        f'd={dist:.1f}m {ml} fm={fm:.2f}')

        self.cmd_pub.publish(tw)
        self._log_row(wp_name, dist, fm, fleft, fright, sleft, sright, tw)

    def _log_row(self, wp_name, dist, fm, fleft, fright, sleft, sright, tw):
        if not self.log_csv:
            return
        t = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9
        self.csv_writer.writerow([
            f'{t:.3f}', f'{self.rx:.4f}', f'{self.ry:.4f}', f'{self.rt:.4f}',
            self.state, wp_name, f'{dist:.4f}',
            f'{fm:.3f}', f'{fleft:.3f}', f'{fright:.3f}', f'{sleft:.3f}', f'{sright:.3f}',
            f'{tw.linear.x:.4f}', f'{tw.angular.z:.4f}'])
        self.csv_file.flush()

    def _arrive(self, wp, dist):
        now = self.get_clock().now()
        elapsed = (now - self.wp_start_time).nanoseconds / 1e9
        self.get_logger().info(
            f'Alvo {wp.name} alcancado em ({self.rx:.2f}, {self.ry:.2f}), '
            f'erro={dist:.3f}m, t={elapsed:.1f}s')
        self.wp_times.append((wp.name, elapsed))
        self.state = 'WAITING'
        self.wait_until = now + Duration(seconds=self.wait_time)

    def _advance_waypoint(self):
        self.widx += 1
        if self.widx >= len(self.mission):
            self.state = 'DONE'
            self._log_summary()
            return
        self.state = 'GO_TO_GOAL'
        self.ml_sx, self.ml_sy = self.rx, self.ry
        self.fticks = 0
        self.wp_start_time = self.get_clock().now()

    def _log_summary(self):
        total = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9
        self.get_logger().info('=== Missao concluida ===')
        self.get_logger().info(f'Tempo total: {total:.1f}s')
        for name, t in self.wp_times:
            self.get_logger().info(f'  {name}: {t:.1f}s')
        self.get_logger().info(f'Contornos executados: {self.n_contornos}')

    def _approach_final(self, wp, ga_r, fm, sleft, sright):
        if self.state == 'WALL_FOLLOW':
            self.state = 'GO_TO_GOAL'
            self.last_leave_tick = self.tick
            self.last_wdir = self.wdir
            self.get_logger().info(
                f'Janela final ({self.approach_window:.2f}m) perto de {wp.name}: '
                f'saindo de WALL_FOLLOW')

        if fm < self.safety_fm:
            tw = Twist()
            tw.angular.z = self.ANG if sleft >= sright else -self.ANG
            self.get_logger().warn(
                f'SEGURANCA fm={fm:.2f}m < {self.safety_fm:.2f}m perto de {wp.name} '
                f'-- girando no lugar')
            return tw

        tw = self._drive_to_goal(ga_r, fm)
        tw.linear.x = min(tw.linear.x, self.approach_max_speed)
        return tw

    def _bug2_step(self, wp, dist, ga_r, fm, fleft, fright, sleft, sright):
        tw = Twist()

        if self.state == 'GO_TO_GOAL':
            if fm < self.D_OBS:
                self.hit_dist = dist
                self.hit_x = self.rx
                self.hit_y = self.ry
                self.fticks = 0
                self.n_contornos += 1

                if self.tick - self.last_leave_tick < 30:
                    self.wdir = self.last_wdir
                    razao = 'histerese'
                else:
                    left_space  = min(sleft,  fleft  * 1.5)
                    right_space = min(sright, fright * 1.5)
                    align_bonus = 1.5 * ga_r
                    left_score  = left_space  + align_bonus
                    right_score = right_space - align_bonus
                    self.wdir = 1 if left_score > right_score else -1
                    razao = f'L={left_score:.2f} R={right_score:.2f}'

                self.state = 'WALL_FOLLOW'
                side_lbl = 'ESQ' if self.wdir == 1 else 'DIR'
                self.get_logger().info(
                    f'HIT ({self.rx:.1f},{self.ry:.1f}) d={dist:.1f}m '
                    f'-> contorno pela {side_lbl} ({razao})')
            else:
                tw = self._drive_to_goal(ga_r, fm)

        elif self.state == 'WALL_FOLLOW':
            self.fticks += 1

            los = self._line_of_sight_to_goal(ga_r, dist)
            los_exit = (
                self.fticks > 15
                and los
                and dist < self.hit_dist - 0.2
            )

            mline_exit = (
                self.fticks > 25
                and self._on_mline(wp)
                and dist < self.hit_dist - 0.3
                and fm > self.D_OBS
            )

            stuck = (
                self.fticks > 250
                and dist > self.hit_dist + 1.0
            )

            back_to_hit = (
                self.fticks > 150
                and math.hypot(self.rx - self.hit_x, self.ry - self.hit_y) < 0.8
            )

            if los_exit:
                self.state = 'GO_TO_GOAL'
                self.ml_sx, self.ml_sy = self.rx, self.ry
                self.last_leave_tick = self.tick
                self.last_wdir = self.wdir
                self.get_logger().info(
                    f'LOS_EXIT ({self.rx:.1f},{self.ry:.1f}) d={dist:.1f}m')
            elif mline_exit:
                self.state = 'GO_TO_GOAL'
                self.ml_sx, self.ml_sy = self.rx, self.ry
                self.last_leave_tick = self.tick
                self.last_wdir = self.wdir
                self.get_logger().info(
                    f'LEAVE ({self.rx:.1f},{self.ry:.1f}) d={dist:.1f}m hit={self.hit_dist:.1f}m')
            elif stuck:
                self.wdir *= -1
                self.fticks = 0
                self.hit_dist = dist
                self.hit_x = self.rx
                self.hit_y = self.ry
                side_lbl = 'ESQ' if self.wdir == 1 else 'DIR'
                self.get_logger().warn(
                    f'STUCK ({self.rx:.1f},{self.ry:.1f}) d={dist:.1f}m '
                    f'-- invertendo para {side_lbl}')
                tw = self._follow_wall(fm, fleft, fright, sleft, sright)
            elif back_to_hit:
                self.wdir *= -1
                self.fticks = 0
                self.hit_dist = dist
                side_lbl = 'ESQ' if self.wdir == 1 else 'DIR'
                self.get_logger().warn(
                    f'LOOP fechado -- invertendo para {side_lbl}')
                tw = self._follow_wall(fm, fleft, fright, sleft, sright)
            else:
                tw = self._follow_wall(fm, fleft, fright, sleft, sright)

        return tw

    def _drive_to_goal(self, ga_r, fm):
        tw = Twist()
        speed_factor = max(0.3, min(1.0, fm / (2 * self.D_OBS)))

        if abs(ga_r) > math.radians(45):
            tw.linear.x = 0.0
            tw.angular.z = self.ANG * (1.0 if ga_r > 0 else -1.0)
        elif abs(ga_r) > math.radians(15):
            tw.linear.x = self.LIN * 0.4 * speed_factor
            tw.angular.z = ga_r * 2.0
        else:
            tw.linear.x = self.LIN * speed_factor
            tw.angular.z = ga_r * 1.5
        return tw

    def _follow_wall(self, fm, fleft, fright, sleft, sright):
        tw = Twist()
        turn = float(self.wdir)

        if self.wdir == 1:
            side_dist  = sright
            front_diag = fright
        else:
            side_dist  = sleft
            front_diag = fleft

        if fm < 0.30:
            tw.linear.x = 0.0
            tw.angular.z = self.ANG * turn
        elif fm < self.D_OBS or front_diag < self.D_OBS * 0.8:
            tw.linear.x = 0.08
            tw.angular.z = self.ANG * 0.65 * turn
        elif side_dist < 0.30:
            tw.linear.x = self.LIN * 0.3
            tw.angular.z = self.ANG * 0.5 * turn
        elif side_dist > self.WALL_DIST + 0.6:
            tw.linear.x = self.LIN * 0.4
            tw.angular.z = self.ANG * 0.4 * (-turn)
        else:
            err = self.WALL_DIST - side_dist
            tw.linear.x = self.LIN * 0.6
            tw.angular.z = err * 1.5 * turn
        return tw


def main(args=None):
    rclpy.init(args=args)
    nav = Navigator()
    try:
        rclpy.spin(nav)
    except KeyboardInterrupt:
        nav.get_logger().info('Encerrado pelo usuario.')
    finally:
        try:
            nav.cmd_pub.publish(Twist())
        except Exception:
            pass
        nav.close_log()
        nav.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
