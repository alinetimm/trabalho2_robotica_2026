from collections import deque, namedtuple
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
              'front_clear', 'path_len', 'v', 'w']

CONTROL_PERIOD = 0.1
NEI8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


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

        self.ranges = []
        self.amin = 0.0
        self.ainc = 0.0
        self.range_max = 3.5
        self.range_min = 0.12
        self.lok = False

        # Grade de ocupacao: comeca tudo livre/desconhecido (0). Marcada so por
        # impactos validos do laser -- acumulativa, nunca desmarca (suficiente
        # para uma arena estatica). A grade inflada e recalculada a cada replan.
        self.grid_n = int(round((self.WORLD_MAX - self.WORLD_MIN) / self.GRID_RES))
        self.occ = bytearray(self.grid_n * self.grid_n)
        self.inflated = bytearray(self.grid_n * self.grid_n)
        radius_cells = max(1, int(round(
            (self.ROBOT_RADIUS + self.SAFETY_MARGIN) / self.GRID_RES)))
        self._inflate_offsets = [
            (di, dj)
            for di in range(-radius_cells, radius_cells + 1)
            for dj in range(-radius_cells, radius_cells + 1)
            if di * di + dj * dj <= radius_cells * radius_cells
        ]

        self.state = 'PLAN'
        self.path = []
        self.n_replans = 0
        self.explore_ticks_left = 0
        self.explore_attempts = 0
        self.last_fail_reason = ''
        self.stuck_dumped = False

        self.wp_times = []
        self.wp_start_time = None
        self.mission_start_time = None
        self.wait_until = None

        self.timer = self.create_timer(CONTROL_PERIOD, self.control)
        self.tick = 0
        self.get_logger().info('=== Navigator (mapa de ocupacao + wavefront) iniciado ===')
        self.get_logger().info(
            f'Pose inicial: ({self.x0}, {self.y0}) theta={math.degrees(self.theta0):.0f}deg')
        self.get_logger().info('Missao: ' + ' -> '.join(w.name for w in self.mission))
        self.get_logger().info(
            f'Grade: {self.grid_n}x{self.grid_n} celulas de {self.GRID_RES}m, '
            f'raio de inflacao {radius_cells} celulas')

    def _load_params(self):
        DBL = rclpy.Parameter.Type.DOUBLE
        STRARR = rclpy.Parameter.Type.STRING_ARRAY
        INT = rclpy.Parameter.Type.INTEGER

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

        self.declare_parameter('plan.grid_res', DBL)
        self.declare_parameter('plan.world_min', DBL)
        self.declare_parameter('plan.world_max', DBL)
        self.declare_parameter('plan.robot_radius', DBL)
        self.declare_parameter('plan.safety_margin', DBL)
        self.declare_parameter('plan.replan_period_s', DBL)
        self.declare_parameter('plan.lookahead', DBL)
        self.declare_parameter('plan.k_ang', DBL)
        self.declare_parameter('plan.v_max', DBL)
        self.declare_parameter('plan.w_max', DBL)
        self.declare_parameter('plan.turn_in_place_angle', DBL)
        self.declare_parameter('plan.explore_time_s', DBL)
        self.declare_parameter('plan.max_explore_attempts', INT)
        self.GRID_RES = self.get_parameter('plan.grid_res').value
        self.WORLD_MIN = self.get_parameter('plan.world_min').value
        self.WORLD_MAX = self.get_parameter('plan.world_max').value
        self.ROBOT_RADIUS = self.get_parameter('plan.robot_radius').value
        self.SAFETY_MARGIN = self.get_parameter('plan.safety_margin').value
        self.REPLAN_TICKS = max(1, int(round(
            self.get_parameter('plan.replan_period_s').value / CONTROL_PERIOD)))
        self.LOOKAHEAD = self.get_parameter('plan.lookahead').value
        self.K_ANG = self.get_parameter('plan.k_ang').value
        self.V_MAX = self.get_parameter('plan.v_max').value
        self.W_MAX = self.get_parameter('plan.w_max').value
        self.TURN_IN_PLACE_ANGLE = self.get_parameter('plan.turn_in_place_angle').value
        self.EXPLORE_TIME = self.get_parameter('plan.explore_time_s').value
        self.MAX_EXPLORE_ATTEMPTS = self.get_parameter('plan.max_explore_attempts').value

        self.declare_parameter('safety.frontal_deg', DBL)
        self.declare_parameter('safety.min_dist', DBL)
        self.SAFETY_FRONTAL_DEG = self.get_parameter('safety.frontal_deg').value
        self.SAFETY_MIN_DIST = self.get_parameter('safety.min_dist').value

        self.declare_parameter('approach.window_dist', DBL)
        self.declare_parameter('approach.max_speed', DBL)
        self.declare_parameter('approach.safety_fm', DBL)
        self.approach_window = self.get_parameter('approach.window_dist').value
        self.approach_max_speed = self.get_parameter('approach.max_speed').value
        self.approach_safety_fm = self.get_parameter('approach.safety_fm').value

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
        self.range_max = msg.range_max
        self.range_min = msg.range_min
        self.lok = True
        self._update_occupancy()

    def _update_occupancy(self):
        # Para cada raio com retorno valido, projeta o ponto de impacto no mundo
        # (usando a pose atual do odom) e marca a celula como ocupada. inf/0/nan
        # (fora de alcance ou zona cega abaixo de range_min) sao descartados aqui
        # -- nao viram "livre" nem "ocupado", so nao contribuem com informacao.
        for k, r in enumerate(self.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if not (self.range_min < r < self.range_max):
                continue
            wang = self.rt + self.amin + k * self.ainc
            px = self.rx + r * math.cos(wang)
            py = self.ry + r * math.sin(wang)
            cell = self._world_to_cell(px, py)
            if cell is None:
                continue
            i, j = cell
            self.occ[i * self.grid_n + j] = 1

    @staticmethod
    def _na(a):
        while a > math.pi:  a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def _angle_to_idx(self, angle_rad):
        n = len(self.ranges)
        raw = (angle_rad - self.amin) / self.ainc
        return int(round(raw)) % n

    def _sector_clear(self, deg_min, deg_max):
        # Indexacao circular ja validada -- nao mexer. NaN/inf/fora de
        # range_min-range_max contam como BLOQUEADO (zona cega do laser), nunca
        # como livre -- e a ultima linha de defesa contra colisao na zona cega.
        if not self.ranges:
            return 0.0
        i1 = self._angle_to_idx(math.radians(deg_min))
        i2 = self._angle_to_idx(math.radians(deg_max))
        sec = self.ranges[i1:i2 + 1] if i1 <= i2 else self.ranges[i1:] + self.ranges[:i2 + 1]
        vals = []
        for r in sec:
            if math.isnan(r) or (not math.isinf(r) and r < self.range_min):
                # NaN ou leitura finita abaixo de range_min: zona cega, obstaculo
                # perto demais pra medir -- bloqueado.
                vals.append(0.0)
            elif math.isinf(r) or r > self.range_max:
                # Nada detectado dentro do alcance (ex: olhando por um corredor
                # aberto) -- livre, nao bloqueado.
                vals.append(self.range_max)
            else:
                vals.append(r)
        return min(vals) if vals else 0.0

    def _goal_info(self):
        wp = self.mission[self.widx]
        dx, dy = wp.x - self.rx, wp.y - self.ry
        d = math.hypot(dx, dy)
        ga = math.atan2(dy, dx)
        gr = self._na(ga - self.rt)
        return d, ga, gr

    # ------------------------------------------------------------------ grade

    def _world_to_cell(self, x, y):
        if not (self.WORLD_MIN <= x <= self.WORLD_MAX and self.WORLD_MIN <= y <= self.WORLD_MAX):
            return None
        i = min(max(int((x - self.WORLD_MIN) / self.GRID_RES), 0), self.grid_n - 1)
        j = min(max(int((y - self.WORLD_MIN) / self.GRID_RES), 0), self.grid_n - 1)
        return i, j

    def _cell_to_world(self, i, j):
        x = self.WORLD_MIN + (i + 0.5) * self.GRID_RES
        y = self.WORLD_MIN + (j + 0.5) * self.GRID_RES
        return x, y

    def _inflate_grid(self):
        n = self.grid_n
        inflated = bytearray(n * n)
        for idx in range(n * n):
            if not self.occ[idx]:
                continue
            i, j = idx // n, idx % n
            for di, dj in self._inflate_offsets:
                ni, nj = i + di, j + dj
                if 0 <= ni < n and 0 <= nj < n:
                    inflated[ni * n + nj] = 1
        self.inflated = inflated

    def _wavefront(self, goal_cell):
        # BFS a partir do objetivo sobre celulas livres (nao infladas), gerando
        # o campo de distancias (mundo classico do "wavefront planner").
        n = self.grid_n
        gi, gj = goal_cell
        if self.inflated[gi * n + gj]:
            return None
        dist = [-1] * (n * n)
        dist[gi * n + gj] = 0
        dq = deque([(gi, gj)])
        while dq:
            i, j = dq.popleft()
            d = dist[i * n + j]
            for di, dj in NEI8:
                ni, nj = i + di, j + dj
                if 0 <= ni < n and 0 <= nj < n:
                    idx = ni * n + nj
                    if dist[idx] == -1 and not self.inflated[idx]:
                        dist[idx] = d + 1
                        dq.append((ni, nj))
        return dist

    def _nearest_with_dist(self, cell, dist_field, max_radius=10):
        n = self.grid_n
        i0, j0 = cell
        if dist_field[i0 * n + j0] != -1:
            return cell
        for r in range(1, max_radius + 1):
            best, best_d = None, None
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    if max(abs(di), abs(dj)) != r:
                        continue
                    ni, nj = i0 + di, j0 + dj
                    if 0 <= ni < n and 0 <= nj < n:
                        dv = dist_field[ni * n + nj]
                        if dv != -1 and (best_d is None or dv < best_d):
                            best_d, best = dv, (ni, nj)
            if best is not None:
                return best
        return None

    def _extract_path(self, dist_field, start_cell):
        n = self.grid_n
        start_cell = self._nearest_with_dist(start_cell, dist_field)
        if start_cell is None:
            return None
        i, j = start_cell
        path = [(i, j)]
        cur_d = dist_field[i * n + j]
        guard = 0
        while cur_d > 0 and guard < n * n:
            guard += 1
            best, best_d = None, cur_d
            for di, dj in NEI8:
                ni, nj = i + di, j + dj
                if 0 <= ni < n and 0 <= nj < n:
                    dv = dist_field[ni * n + nj]
                    if dv != -1 and dv < best_d:
                        best_d, best = dv, (ni, nj)
            if best is None:
                break
            i, j = best
            path.append((i, j))
            cur_d = best_d
        return path

    def _find_approach_cell(self, tx, ty, tol):
        # O alvo e um cilindro fisico: a celula do seu centro fica dentro da
        # regiao inflada e e inalcancavel. Procura a celula livre mais proxima
        # do robo num anel a ~tol do centro, expandindo o raio se preciso.
        n_angles = 36
        for k in range(12):
            r = tol + k * self.GRID_RES
            candidates = []
            for a in range(n_angles):
                ang = 2.0 * math.pi * a / n_angles
                px = tx + r * math.cos(ang)
                py = ty + r * math.sin(ang)
                cell = self._world_to_cell(px, py)
                if cell is None:
                    continue
                i, j = cell
                if not self.inflated[i * self.grid_n + j]:
                    candidates.append((math.hypot(px - self.rx, py - self.ry), (i, j)))
            if candidates:
                candidates.sort(key=lambda c: c[0])
                return candidates[0][1]
        return None

    def _goal_cell(self, wp):
        if wp.is_target:
            return self._find_approach_cell(wp.x, wp.y, wp.tol)
        cell = self._world_to_cell(wp.x, wp.y)
        if cell is None:
            return None
        i, j = cell
        if self.inflated[i * self.grid_n + j]:
            return self._find_approach_cell(wp.x, wp.y, 0.05)
        return cell

    def _replan(self, wp):
        self._inflate_grid()
        goal_cell = self._goal_cell(wp)
        if goal_cell is None:
            self.path = []
            self.last_fail_reason = ('objetivo sem celula livre por perto '
                                      '(anel de aproximacao todo bloqueado/inflado)')
            return False
        dist_field = self._wavefront(goal_cell)
        if dist_field is None:
            self.path = []
            self.last_fail_reason = 'celula do objetivo caiu dentro da regiao inflada'
            return False
        start_cell = self._world_to_cell(self.rx, self.ry)
        if start_cell is None:
            self.path = []
            self.last_fail_reason = 'robo fora dos limites da grade'
            return False
        path_cells = self._extract_path(dist_field, start_cell)
        if not path_cells:
            self.path = []
            self.last_fail_reason = ('sem conexao livre entre o robo e o objetivo no mapa '
                                      'conhecido ate agora (bloqueio real ou falso-positivo)')
            return False
        self.path = [self._cell_to_world(i, j) for i, j in path_cells]
        self.n_replans += 1
        self.explore_attempts = 0
        return True

    # --------------------------------------------------------------- controle

    def control(self):
        if not self.lok:
            return
        self.tick += 1

        if self.wp_start_time is None:
            self.wp_start_time = self.get_clock().now()
            self.mission_start_time = self.wp_start_time

        front_clear = self._sector_clear(-self.SAFETY_FRONTAL_DEG, self.SAFETY_FRONTAL_DEG)

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
                    if self.state != 'APPROACH':
                        self.state = 'APPROACH'
                        self.get_logger().info(
                            f'Janela final ({self.approach_window:.2f}m) perto de {wp.name}: '
                            f'aproximacao direta')
                    tw = self._approach_step(ga_r)
                else:
                    if self.state == 'APPROACH':
                        self.state = 'PLAN'
                        self.path = []
                    if self.explore_ticks_left > 0:
                        tw = self._explore_step()
                    else:
                        need_replan = self.state != 'FOLLOW' or self.tick % self.REPLAN_TICKS == 0
                        if need_replan:
                            if self._replan(wp):
                                self.state = 'FOLLOW'
                                tw = self._follow_path(front_clear)
                            else:
                                self.state = 'PLAN'
                                self._start_explore()
                                tw = self._explore_step()
                        else:
                            tw = self._follow_path(front_clear)

                if self.tick % 30 == 0:
                    self.get_logger().info(
                        f'[{self.state}] alvo={wp.name} ({self.rx:.1f},{self.ry:.1f}) '
                        f'd={dist:.1f}m front={front_clear:.2f}m path={len(self.path)}')

        self.cmd_pub.publish(tw)
        self._log_row(wp_name, dist, front_clear, len(self.path), tw)

    def _log_row(self, wp_name, dist, front_clear, path_len, tw):
        if not self.log_csv:
            return
        t = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9
        self.csv_writer.writerow([
            f'{t:.3f}', f'{self.rx:.4f}', f'{self.ry:.4f}', f'{self.rt:.4f}',
            self.state, wp_name, f'{dist:.4f}',
            f'{front_clear:.3f}', f'{path_len}',
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
        self.state = 'PLAN'
        self.path = []
        self.explore_ticks_left = 0
        self.explore_attempts = 0
        self.stuck_dumped = False
        self.wp_start_time = self.get_clock().now()

    def _log_summary(self):
        total = (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9
        self.get_logger().info('=== Missao concluida ===')
        self.get_logger().info(f'Tempo total: {total:.1f}s')
        for name, t in self.wp_times:
            self.get_logger().info(f'  {name}: {t:.1f}s')
        self.get_logger().info(f'Replanejamentos executados: {self.n_replans}')
        self._dump_occ_grid()

    def _dump_occ_grid(self, suffix=''):
        if not self.log_csv or self.csv_file is None:
            return
        self._inflate_grid()
        n = self.grid_n
        path = self.csv_file.name.rsplit('.', 1)[0] + suffix + '_occ.txt'
        with open(path, 'w') as f:
            f.write(f'{self.GRID_RES},{self.WORLD_MIN},{self.WORLD_MIN},'
                    f'{self.WORLD_MAX},{self.WORLD_MAX},{n}\n')
            for i in range(n):
                f.write(''.join('1' if self.inflated[i * n + j] else '0'
                                 for j in range(n)) + '\n')
        self.get_logger().info(f'Grade de ocupacao salva em {path}')

    # ---------------------------------------------------------- comportamentos

    def _approach_step(self, ga_r):
        # Dentro da janela final: desvio desligado, so aproximacao direta e lenta,
        # com parada de seguranca se algo aparecer perto demais.
        tw = Twist()
        fm = self._sector_clear(-25, 25)
        if fm < self.approach_safety_fm:
            sleft = self._sector_clear(60, 120)
            sright = self._sector_clear(-120, -60)
            tw.angular.z = self.W_MAX if sleft >= sright else -self.W_MAX
            self.get_logger().warn(
                f'SEGURANCA fm={fm:.2f}m < {self.approach_safety_fm:.2f}m na aproximacao '
                f'final -- girando no lugar')
            return tw

        if abs(ga_r) > math.radians(45):
            tw.angular.z = self.W_MAX * (1.0 if ga_r > 0 else -1.0)
        elif abs(ga_r) > math.radians(15):
            tw.linear.x = self.approach_max_speed * 0.5
            tw.angular.z = ga_r * 2.0
        else:
            tw.linear.x = self.approach_max_speed
            tw.angular.z = ga_r * 1.5
        return tw

    def _lookahead_point(self):
        if not self.path:
            return None
        for px, py in self.path:
            if math.hypot(px - self.rx, py - self.ry) >= self.LOOKAHEAD:
                return px, py
        return self.path[-1]

    def _follow_path(self, front_clear):
        # Pure-pursuit simples sobre o caminho do wavefront.
        tw = Twist()
        target = self._lookahead_point()
        if target is None:
            return tw
        tx, ty = target
        bearing = self._na(math.atan2(ty - self.ry, tx - self.rx) - self.rt)
        tw.angular.z = max(-self.W_MAX, min(self.W_MAX, self.K_ANG * bearing))
        tw.linear.x = self.V_MAX
        if abs(bearing) > self.TURN_IN_PLACE_ANGLE:
            tw.linear.x = 0.0

        # Trava dura, independente do plano: se a frente (+-frontal_deg) esta
        # bloqueada (incluindo zona cega abaixo de range_min), nao avanca de
        # jeito nenhum -- ultima linha de defesa contra colisao.
        if front_clear < self.SAFETY_MIN_DIST:
            tw.linear.x = 0.0
        return tw

    def _start_explore(self):
        self.explore_attempts += 1
        self.explore_ticks_left = max(1, int(round(self.EXPLORE_TIME / CONTROL_PERIOD)))
        msg = (f'PLAN ({self.rx:.1f},{self.ry:.1f}) -- {self.last_fail_reason} '
               f'(tentativa {self.explore_attempts}), girando pra explorar')
        if self.explore_attempts > self.MAX_EXPLORE_ATTEMPTS:
            self.get_logger().error(msg + ' [acima do limite de tentativas]')
            if not self.stuck_dumped:
                self.stuck_dumped = True
                self._dump_occ_grid(suffix='_stuck')
        else:
            self.get_logger().warn(msg)

    def _explore_step(self):
        # Deterministico: sempre gira no mesmo sentido, sem manobra aleatoria.
        self.explore_ticks_left -= 1
        tw = Twist()
        tw.angular.z = self.W_MAX
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
