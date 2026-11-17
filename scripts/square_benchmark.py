#!/usr/bin/env python3
"""
Record /odom + /cmd_vel during the square test and make the checkpoint 1.4 plots:
  1. estimated (x, y) pose over all loops vs the ideal square
  2. linear + angular velocity (measured and commanded) for one loop

Record (resets odometry first, auto-stops 3 s after the robot stops moving, or Ctrl-C):
  python3 square_benchmark.py --tag slow
Re-plot an existing CSV:
  python3 square_benchmark.py --plot <file.csv>
"""
import argparse, csv, math, os, threading, time
from datetime import datetime

OUT_DIRS = {'square': os.path.expanduser('~/mbot_firmware_ros/Checkpoint 1/1.4/atman_square_data'),
            'maze': os.path.expanduser('~/mbot_firmware_ros/Checkpoint 1/submission/atman_maze_data')}
C = 0.61
MAZE = [(0, 0), (C, 0), (C, -C), (2*C, -C), (2*C, C), (3*C, C), (3*C, -C), (4*C, -C), (4*C, 0), (5*C, 0)]
FAST_C = '#eb6834'
ODOM_C, CMD_C = '#2a78d6', '#eb6834'
INK, MUTED, GRID = '#1f1f1e', '#6b6a64', '#e4e3df'


def record(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_srvs.srv import Trigger

    rclpy.init()
    node = Node('square_recorder')
    lock = threading.Lock()
    rows, cmd = [], [0.0, 0.0]

    def on_cmd(m):
        with lock:
            cmd[0], cmd[1] = m.linear.x, m.angular.z

    def on_odom(m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        with lock:
            rows.append(dict(t=time.time(), x=m.pose.pose.position.x, y=m.pose.pose.position.y,
                             theta=yaw, vx=m.twist.twist.linear.x, wz=m.twist.twist.angular.z,
                             cmd_vx=cmd[0], cmd_wz=cmd[1]))

    node.create_subscription(Twist, '/cmd_vel', on_cmd, 10)
    node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data)
    cli = node.create_client(Trigger, '/reset_odometry')
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    if cli.wait_for_service(timeout_sec=3.0):
        fut = cli.call_async(Trigger.Request())
        while not fut.done():
            time.sleep(0.05)
        print('Odometry reset.')
    time.sleep(0.5)
    with lock:
        rows.clear()

    print('Recording. Now start square_publisher. Auto-stops 4 s after the robot stops (Ctrl-C to stop early).')
    moved, still_since = False, None
    try:
        while True:
            time.sleep(0.1)
            with lock:
                if not rows:
                    continue
                last = rows[-1]
            # 'moved' only once the robot has really left the start (encoder noise can't trigger it)
            if not moved and math.hypot(last['x'], last['y']) > 0.10:
                moved = True
                print('Robot moving - recording run...')
            busy = abs(last['vx']) > 0.02 or abs(last['wz']) > 0.2 or last['cmd_vx'] != 0 or last['cmd_wz'] != 0
            if busy or not moved:
                still_since = None
            else:
                still_since = still_since or time.time()
                if time.time() - still_since > 4.0:
                    break
    except KeyboardInterrupt:
        pass

    rclpy.try_shutdown()
    out_dir = OUT_DIRS[args.path]
    os.makedirs(out_dir, exist_ok=True)
    tag = f'_{args.tag}' if args.tag else ''
    path = os.path.join(out_dir, f'{args.path}{tag}_{datetime.now():%Y%m%d_%H%M%S}.csv')
    with lock:
        data = list(rows)
    t0 = data[0]['t']
    for r in data:
        r['t'] -= t0
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(data[0].keys()))
        w.writeheader(); w.writerows(data)
    print(f'Saved {path} ({len(data)} samples)')
    return path


def style(ax, title):
    ax.set_title(title, loc='left', color=INK, fontsize=11, pad=8)
    for s in ('top', 'right'): ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'): ax.spines[s].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(True, color=GRID, lw=0.8); ax.set_axisbelow(True)


def plot(path, side, shape='square'):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    data = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(path))]
    base = os.path.splitext(path)[0]
    t = [r['t'] for r in data]

    # Loop boundaries: back within 10 cm of origin after travelling ~3.5 sides since last boundary
    bounds, dist, start = [0], 0.0, 0
    for i in range(1, len(data)):
        dist += math.hypot(data[i]['x'] - data[i-1]['x'], data[i]['y'] - data[i-1]['y'])
        if dist > 3.5 * side and math.hypot(data[i]['x'], data[i]['y']) < 0.10:
            # advance to the point closest to origin in this approach
            j = i
            while j + 1 < len(data) and math.hypot(data[j+1]['x'], data[j+1]['y']) <= math.hypot(data[j]['x'], data[j]['y']):
                j += 1
            bounds.append(j); dist = 0.0
    n_loops = len(bounds) - 1 if shape == 'square' else 0

    # 1. XY pose
    fig, ax = plt.subplots(figsize=(6, 6) if shape == 'square' else (10, 5))
    if shape == 'square':
        style(ax, f'Estimated pose (odometry), {n_loops} loop(s) of a {side:g} m square')
        ideal = [(0, 0), (side, 0), (side, side), (0, side), (0, 0)]
    else:
        style(ax, f'Estimated pose (odometry), checkpoint path - {os.path.basename(base)}')
        ideal = MAZE
    ax.plot(*zip(*ideal), color=MUTED, lw=1, ls='--', label='ideal path')
    ax.plot([r['x'] for r in data], [r['y'] for r in data], color=ODOM_C, lw=2, label='odom (x, y)')
    end = data[-1]
    ax.plot(end['x'], end['y'], 'o', ms=8, color=ODOM_C, mec='white', mew=2)
    ax.annotate(f'end ({end["x"]:+.3f}, {end["y"]:+.3f}) m', (end['x'], end['y']), xytext=(8, -14),
                textcoords='offset points', fontsize=9, color=INK)
    ax.set_aspect('equal')
    xs_, ys_ = [p[0] for p in ideal], [p[1] for p in ideal]
    ax.set_xlim(min(xs_) - 0.2, max(xs_) + 0.2); ax.set_ylim(min(ys_) - 0.2, max(ys_) + 0.2)
    ax.set_xlabel('x (m)', color=MUTED); ax.set_ylabel('y (m)', color=MUTED)
    ax.legend(frameon=False, fontsize=9, labelcolor=INK, loc='center' if shape == 'square' else 'lower right')
    fig.tight_layout(); fig.savefig(base + '_pose.png', dpi=150); plt.close(fig)

    # 2. Velocity for first loop
    i0, i1 = (bounds[0], bounds[1]) if n_loops >= 1 else (0, len(data) - 1)
    seg_name = 'loop 1' if n_loops >= 1 else 'full run'
    # trim idle time before the first command
    first = next((i for i, r in enumerate(data) if r['cmd_vx'] != 0 or r['cmd_wz'] != 0), 0)
    i0 = max(i0, first - 10)
    seg = data[i0:i1 + 1]; ts = [r['t'] - seg[0]['t'] for r in seg]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    style(a1, f'Linear velocity, {seg_name}')
    a1.plot(ts, [r['cmd_vx'] for r in seg], color=CMD_C, lw=2, label='commanded')
    a1.plot(ts, [r['vx'] for r in seg], color=ODOM_C, lw=2, label='measured (odom)')
    a1.set_ylabel('vx (m/s)', color=MUTED)
    style(a2, f'Angular velocity, {seg_name}')
    a2.plot(ts, [r['cmd_wz'] for r in seg], color=CMD_C, lw=2, label='commanded')
    a2.plot(ts, [r['wz'] for r in seg], color=ODOM_C, lw=2, label='measured (odom)')
    a2.set_ylabel('wz (rad/s)', color=MUTED); a2.set_xlabel('time (s)', color=MUTED)
    h, l = a1.get_legend_handles_labels()
    fig.legend(h, l, frameon=False, fontsize=9, labelcolor=INK, loc='lower center', ncol=2)
    fig.tight_layout(rect=(0, 0.05, 1, 1)); fig.savefig(base + '_velocity.png', dpi=150); plt.close(fig)

    print(f'Loops detected: {n_loops}. Final odom pose: x={end["x"]:+.3f} y={end["y"]:+.3f} '
          f'theta={math.degrees(end["theta"]):+.1f} deg')
    print(f'Plots: {base}_pose.png, {base}_velocity.png')


def compare(slow_csv, fast_csv):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 5))
    style(ax, 'Checkpoint path - odometry (x, y) at slow and fast speed')
    ax.plot(*zip(*MAZE), color=MUTED, lw=1, ls='--', label='ideal path')
    for f, c, name in ((slow_csv, ODOM_C, 'slow (0.2 m/s, π/4 rad/s)'), (fast_csv, FAST_C, 'fast (0.8 m/s, π rad/s)')):
        d = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(f))]
        ax.plot([r['x'] for r in d], [r['y'] for r in d], color=c, lw=2, label=name)
        ax.plot(d[-1]['x'], d[-1]['y'], 'o', ms=8, color=c, mec='white', mew=2)
    ax.set_aspect('equal'); ax.set_xlim(-0.2, 5 * C + 0.2); ax.set_ylim(-C - 0.2, C + 0.2)
    ax.set_xlabel('x (m)', color=MUTED); ax.set_ylabel('y (m)', color=MUTED)
    ax.legend(frameon=False, fontsize=9, labelcolor=INK, loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=3)
    out = os.path.join(os.path.dirname(os.path.abspath(slow_csv)), 'maze_slow_vs_fast.png')
    fig.tight_layout(); fig.savefig(out, dpi=150); plt.close(fig)
    print(f'Saved {out}')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--plot', help='re-plot an existing CSV instead of recording')
    ap.add_argument('--tag', default='', help='label added to file names, e.g. slow / fast')
    ap.add_argument('--side', type=float, default=1.0)
    ap.add_argument('--path', choices=['square', 'maze'], default='square')
    ap.add_argument('--compare', nargs=2, metavar=('SLOW_CSV', 'FAST_CSV'), help='overlay slow and fast maze runs')
    a = ap.parse_args()
    if a.compare:
        compare(*a.compare)
    else:
        plot(a.plot if a.plot else record(a), a.side, a.path)
