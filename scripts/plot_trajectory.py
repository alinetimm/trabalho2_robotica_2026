#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import yaml
from matplotlib.lines import Line2D

ARENA_HALF = 2.425
TARGET_RADIUS = 0.12
TARGET_COLOR = {'verde': 'green', 'vermelho': 'red', 'azul': 'blue', 'laranja': 'orange'}
STATE_COLOR = {'GO_TO_GOAL': 'tab:blue', 'WALL_FOLLOW': 'tab:red'}
OTHER_COLOR = 'lightgray'


def load_mission(path):
    with open(path) as f:
        cfg = yaml.safe_load(f)['navigator']['ros__parameters']
    home = (cfg['home']['x'], cfg['home']['y'])
    targets = {name: (t['x'], t['y']) for name, t in cfg['targets'].items()}
    return home, targets


def load_log(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def segments(rows):
    segs = []
    cur_state, cur_x, cur_y = None, [], []
    for r in rows:
        st = r['estado']
        x, y = float(r['x']), float(r['y'])
        if st != cur_state:
            if cur_x:
                segs.append((cur_state, cur_x, cur_y))
            cur_state, cur_x, cur_y = st, [x], [y]
        else:
            cur_x.append(x)
            cur_y.append(y)
    if cur_x:
        segs.append((cur_state, cur_x, cur_y))
    return segs


def hit_and_leave_points(rows):
    hit_x, hit_y, leave_x, leave_y = [], [], [], []
    prev_state = None
    for r in rows:
        st = r['estado']
        x, y = float(r['x']), float(r['y'])
        if prev_state == 'GO_TO_GOAL' and st == 'WALL_FOLLOW':
            hit_x.append(x)
            hit_y.append(y)
        elif prev_state == 'WALL_FOLLOW' and st != 'WALL_FOLLOW':
            leave_x.append(x)
            leave_y.append(y)
        prev_state = st
    return hit_x, hit_y, leave_x, leave_y


def plot(rows, home, targets, out_path):
    fig, ax = plt.subplots(figsize=(8, 8))

    wall = [-ARENA_HALF, ARENA_HALF, ARENA_HALF, -ARENA_HALF, -ARENA_HALF]
    wall_y = [-ARENA_HALF, -ARENA_HALF, ARENA_HALF, ARENA_HALF, -ARENA_HALF]
    ax.plot(wall, wall_y, color='black', linewidth=1.5)

    for st, xs, ys in segments(rows):
        color = STATE_COLOR.get(st, OTHER_COLOR)
        lw = 2.0 if st in STATE_COLOR else 1.0
        ax.plot(xs, ys, color=color, linewidth=lw)

    hit_x, hit_y, leave_x, leave_y = hit_and_leave_points(rows)
    ax.scatter(hit_x, hit_y, marker='x', color='black', s=70, zorder=5)
    ax.scatter(leave_x, leave_y, marker='+', color='purple', s=90, zorder=5)

    for name, (tx, ty) in targets.items():
        circ = plt.Circle((tx, ty), TARGET_RADIUS, color=TARGET_COLOR.get(name, 'gray'),
                           alpha=0.6, zorder=3)
        ax.add_patch(circ)
        ax.annotate(name, (tx, ty), textcoords='offset points', xytext=(0, 10),
                    ha='center', fontsize=9)

    hx, hy = home
    ax.scatter([hx], [hy], marker='X', s=150, color='black', zorder=4)

    legend_elems = [
        Line2D([0], [0], color=STATE_COLOR['GO_TO_GOAL'], lw=2, label='GO_TO_GOAL'),
        Line2D([0], [0], color=STATE_COLOR['WALL_FOLLOW'], lw=2, label='WALL_FOLLOW'),
        Line2D([0], [0], color=OTHER_COLOR, lw=1, label='WAITING / DONE'),
        Line2D([0], [0], marker='x', color='black', lw=0, markersize=8, label='HIT'),
        Line2D([0], [0], marker='+', color='purple', lw=0, markersize=10, label='LEAVE/LOS_EXIT'),
        Line2D([0], [0], marker='X', color='black', lw=0, markersize=10, label='home'),
    ]
    ax.legend(handles=legend_elems, loc='upper left', bbox_to_anchor=(1.02, 1.0),
              fontsize=8, borderaxespad=0.0)

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Trajetoria - Bug2 + missao')
    ax.set_aspect('equal')
    ax.set_xlim(-ARENA_HALF - 0.3, ARENA_HALF + 0.3)
    ax.set_ylim(-ARENA_HALF - 0.3, ARENA_HALF + 0.3)
    ax.grid(alpha=0.3)

    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f'Figura salva em {out_path}')


def main():
    ap = argparse.ArgumentParser(
        description='Plota a trajetoria de uma missao a partir do CSV de log do navigator.')
    ap.add_argument('csv_path', help='Caminho do CSV gerado pelo navigator (log_csv)')
    ap.add_argument('-o', '--output', default=None, help='Caminho do PNG de saida')
    ap.add_argument('--mission-yaml', default=None,
                     help='Caminho do mission.yaml (default: config/mission.yaml do pacote)')
    args = ap.parse_args()

    script_dir = Path(__file__).resolve().parent
    mission_yaml = args.mission_yaml or (
        script_dir.parent / 'ros_ws' / 'src' / 'robot_navigation' / 'config' / 'mission.yaml')

    home, targets = load_mission(mission_yaml)
    rows = load_log(args.csv_path)
    out_path = args.output or str(Path(args.csv_path).with_suffix('.png'))
    plot(rows, home, targets, out_path)


if __name__ == '__main__':
    main()
