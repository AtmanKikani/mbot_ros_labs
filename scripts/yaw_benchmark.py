#!/usr/bin/env python3
"""
Yaw benchmark: /odom vs /gyrodom against a floor yaw poster.

Rotates the robot in place to a sequence of target yaws, stops, and asks you to
type the poster reading (ground truth). Logs odom and gyrodom yaw at each stop
and saves a CSV + prints an error summary.

Positive yaw = counter-clockwise viewed from above (ROS convention).

Usage:
  python3 yaw_benchmark.py                 # feedback on gyrodom, 3 full turns
  python3 yaw_benchmark.py --feedback odom --turns 5
"""
import argparse
import csv
import os
import math
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Unwrapper:
    """Turns a [-pi, pi] yaw stream into a continuous angle (for multi-turn spins)."""
    def __init__(self):
        self.last = None
        self.total = 0.0

    def update(self, yaw):
        if self.last is not None:
            d = yaw - self.last
            d = (d + math.pi) % (2 * math.pi) - math.pi
            self.total += d
        else:
            self.total = yaw
        self.last = yaw
        return self.total


class YawBench(Node):
    def __init__(self):
        super().__init__('yaw_benchmark')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lock = threading.Lock()
        self.yaw = {'odom': None, 'gyrodom': None}
        self.unwrap = {'odom': Unwrapper(), 'gyrodom': Unwrapper()}
        self.create_subscription(Odometry, '/odom', lambda m: self._cb('odom', m), 10)
        self.create_subscription(Odometry, '/gyrodom', lambda m: self._cb('gyrodom', m), 10)
        self.reset_cli = self.create_client(Trigger, '/reset_odometry')

    def _cb(self, name, msg):
        with self.lock:
            self.yaw[name] = self.unwrap[name].update(quat_to_yaw(msg.pose.pose.orientation))

    def get(self, name):
        with self.lock:
            return self.yaw[name]

    def reset(self):
        for k in self.unwrap:
            self.unwrap[k] = Unwrapper()
            self.yaw[k] = None

    def send(self, wz):
        t = Twist()
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)


def rotate_to(node, src, target, max_w, min_w, kp, tol, timeout):
    """P-controller on the unwrapped yaw of `src` until within tol (rad)."""
    t0 = time.time()
    rate = 1.0 / 50.0
    while time.time() - t0 < timeout:
        err = target - node.get(src)
        if abs(err) < tol:
            break
        w = max(min(abs(kp * err), max_w), min_w)
        node.send(math.copysign(w, err))
        time.sleep(rate)
    else:
        print(f'  [warn] timed out reaching {math.degrees(target):.1f} deg')
    for _ in range(10):
        node.send(0.0)
        time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--feedback', choices=['gyrodom', 'odom'], default='gyrodom',
                    help='which estimate drives the rotation controller')
    ap.add_argument('--turns', type=int, default=3, help='full turns for the spin test')
    ap.add_argument('--slow', type=float, default=0.3, help='max rad/s for small steps')
    ap.add_argument('--fast', type=float, default=1.0, help='max rad/s for full turns')
    ap.add_argument('--min-w', type=float, default=0.12, help='min rad/s (overcome deadband)')
    ap.add_argument('--out-dir', default='.', help='folder to save the CSV in')
    ap.add_argument('--settle', type=float, default=1.5, help='seconds to settle before reading')
    args = ap.parse_args()

    rclpy.init()
    node = YawBench()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Reset firmware odometry so both estimates start at 0
    if node.reset_cli.wait_for_service(timeout_sec=3.0):
        fut = node.reset_cli.call_async(Trigger.Request())
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 3.0:
            time.sleep(0.05)
        print('Odometry reset:', fut.result().message if fut.done() else 'no response')
    else:
        print('[warn] /reset_odometry not available, using relative yaw')
    time.sleep(0.5)
    node.reset()

    print('Waiting for /odom and /gyrodom...')
    while node.get('odom') is None or node.get('gyrodom') is None:
        time.sleep(0.05)
    zero = {k: node.get(k) for k in ('odom', 'gyrodom')}

    # (label, target_deg, speed)
    steps = []
    for d in [15, 30, 45, 0, -15, -30, -45, 0]:
        steps.append((f'step {d:+d}', d, args.slow))
    n = args.turns
    steps.append((f'spin +{n} turns (CCW)', 360 * n, args.fast))
    steps.append((f'spin back -{n} turns (CW)', 0, args.fast))

    # Controller works relative to the starting zero of the feedback source
    rows = []
    os.makedirs(os.path.expanduser(args.out_dir), exist_ok=True)
    fname = os.path.join(os.path.expanduser(args.out_dir), f'yaw_benchmark_{datetime.now():%Y%m%d_%H%M%S}.csv')
    print(f'\nFeedback source: {args.feedback}.  Positive = CCW from above.')
    input('Robot at poster 0 mark? Press Enter to start (Ctrl-C to abort)...')

    try:
        for label, tgt_deg, w in steps:
            print(f'\n>> {label}: rotating to {tgt_deg:+d} deg')
            tgt = zero[args.feedback] + math.radians(tgt_deg)
            rotate_to(node, args.feedback, tgt, max_w=w, min_w=args.min_w, kp=1.5,
                      tol=math.radians(0.5), timeout=60 + 15 * abs(tgt_deg) / 90)
            time.sleep(args.settle)
            od = math.degrees(node.get('odom') - zero['odom'])
            gy = math.degrees(node.get('gyrodom') - zero['gyrodom'])
            print(f'   odom = {od:+8.2f} deg   gyrodom = {gy:+8.2f} deg')
            s = input('   RECORD GROUND TRUTH NOW - poster reading in deg (Enter = skip): ').strip()
            if s:
                reading = float(s)
                # Poster only shows +/-45; add back whole turns implied by the target
                gt = reading + 360.0 * round((tgt_deg - reading) / 360.0)
            else:
                gt = float('nan')
            rows.append(dict(step=label, target_deg=tgt_deg, gt_deg=gt, odom_deg=od,
                             gyrodom_deg=gy, odom_err=od - gt, gyrodom_err=gy - gt))
    except KeyboardInterrupt:
        print('\nAborted.')
    finally:
        for _ in range(10):
            node.send(0.0)
            time.sleep(0.02)

    if rows:
        with open(fname, 'w', newline='') as f:
            wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            wr.writeheader()
            wr.writerows(rows)
        print(f'\nSaved {fname}\n')
        print(f'{"step":<26}{"GT":>9}{"odom":>10}{"gyrodom":>10}{"odom err":>10}{"gyro err":>10}')
        for r in rows:
            print(f'{r["step"]:<26}{r["gt_deg"]:>9.2f}{r["odom_deg"]:>10.2f}{r["gyrodom_deg"]:>10.2f}'
                  f'{r["odom_err"]:>10.2f}{r["gyrodom_err"]:>10.2f}')
        valid = [r for r in rows if not math.isnan(r['gt_deg'])]
        if valid:
            rms = lambda k: math.sqrt(sum(r[k] ** 2 for r in valid) / len(valid))
            print(f'\nRMS error  odom: {rms("odom_err"):.2f} deg   gyrodom: {rms("gyrodom_err"):.2f} deg')

    rclpy.try_shutdown()
    spin_thread.join(timeout=2.0)
    node.destroy_node()


if __name__ == '__main__':
    main()
