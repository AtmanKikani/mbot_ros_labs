#!/usr/bin/env python3
"""
Calibrate DIFF_WHEEL_RADIUS and DIFF_BASE_RADIUS from two measured runs.

  1. Straight: drives --dist metres by odometry. You tape-measure the real distance.
       wheel radius scale s = actual / odom
  2. Spin: turns --turns full turns CCW by odometry at --spin-w rad/s (use the speed
     your controller actually turns at). You read the yaw poster.
       rotation scale k = actual / odom
  Rotation ∝ r / B, so:  r_new = r_old * s,   B_new = B_old * s / k

Usage: python3 odom_calibration.py --dist 2.0 --turns 4

The current radii are read from mbot_classic_config.h, so the robot must be running
firmware built from that file (do not edit the header between flashing and calibrating).
"""
import argparse, math, os, re, threading, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

CONFIG_H = os.path.expanduser('~/mbot_firmware_ros/include/config/mbot_classic_config.h')


def read_config_radii(path=CONFIG_H):
    """Current DIFF_WHEEL_RADIUS / DIFF_BASE_RADIUS from the firmware config (the values now flashed)."""
    vals = {}
    for line in open(path):
        m = re.match(r'\s*#define\s+(DIFF_WHEEL_RADIUS|DIFF_BASE_RADIUS)\s+([0-9.]+)', line)
        if m:
            vals[m.group(1)] = float(m.group(2))
    return vals['DIFF_WHEEL_RADIUS'], vals['DIFF_BASE_RADIUS']


class Cal(Node):
    def __init__(self):
        super().__init__('odom_calibration')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lock = threading.Lock()
        self.x = self.y = self.yaw = None
        self.unwrapped = 0.0
        self.create_subscription(Odometry, '/odom', self.cb, qos_profile_sensor_data)
        self.reset_cli = self.create_client(Trigger, '/reset_odometry')

    def cb(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        with self.lock:
            if self.yaw is not None:
                self.unwrapped += (yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
            self.x, self.y, self.yaw = m.pose.pose.position.x, m.pose.pose.position.y, yaw

    def state(self):
        with self.lock:
            return self.x, self.y, self.yaw, self.unwrapped

    def send(self, v, w):
        t = Twist(); t.linear.x = float(v); t.angular.z = float(w); self.pub.publish(t)

    def stop(self):
        for _ in range(10):
            self.send(0, 0); time.sleep(0.02)

    def reset(self):
        if self.reset_cli.wait_for_service(timeout_sec=3.0):
            f = self.reset_cli.call_async(Trigger.Request())
            while not f.done(): time.sleep(0.05)
        time.sleep(0.5)
        with self.lock:
            self.yaw = None; self.unwrapped = 0.0
        while self.state()[2] is None: time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dist', type=float, default=2.0)
    ap.add_argument('--turns', type=int, default=4)
    ap.add_argument('--v', type=float, default=0.25)
    ap.add_argument('--spin-w', type=float, default=1.0)
    a = ap.parse_args()

    rclpy.init(); n = Cal()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    try:
        # ---- straight ----
        input(f'\n[1/2] STRAIGHT {a.dist} m. Mark the robot start (e.g. front axle on tape). Enter to go...')
        n.reset()
        while True:
            x, y, yaw, _ = n.state()
            togo = a.dist - x
            if togo < 0.002: break
            v = max(min(a.v, 1.0 * togo), 0.03)
            n.send(v, -2.0 * y - 1.5 * yaw)  # hold the x-axis
            time.sleep(0.02)
        n.stop(); time.sleep(1.0)
        x, y, *_ = n.state(); odom_d = math.hypot(x, y)
        print(f'   odom distance = {odom_d:.4f} m')
        actual_d = float(input('   RECORD NOW - measured straight-line distance travelled (m): '))
        s = actual_d / odom_d

        # ---- spin ----
        input(f'\n[2/2] SPIN {a.turns} turns CCW. Place robot on poster 0 mark. Enter to go...')
        n.reset()
        target = 2 * math.pi * a.turns
        while True:
            err = target - n.state()[3]
            if err < math.radians(0.3): break
            n.send(0, max(min(a.spin_w, 2.0 * err), 0.12))
            time.sleep(0.02)
        n.stop(); time.sleep(1.5)
        odom_rot = math.degrees(n.state()[3])
        print(f'   odom rotation = {odom_rot:.2f} deg')
        reading = float(input('   RECORD NOW - poster reading (deg, CCW positive, e.g. -28): '))
        actual_rot = reading + 360.0 * round((odom_rot - reading) / 360.0)
        k = actual_rot / odom_rot
    except KeyboardInterrupt:
        n.stop(); print('\nAborted.'); rclpy.try_shutdown(); return
    n.stop(); rclpy.try_shutdown()

    # Corrections scale the values that were flashed during this test -> must match your firmware build
    R_OLD, B_OLD = read_config_radii()
    print(f'\nCurrent config ({CONFIG_H}): DIFF_WHEEL_RADIUS={R_OLD}, DIFF_BASE_RADIUS={B_OLD}')
    r_new = R_OLD * s
    b_new = B_OLD * s / k
    print(f'\nDistance scale s = {s:.4f}   (actual {actual_d:.3f} / odom {odom_d:.3f})')
    print(f'Rotation scale k = {k:.4f}   (actual {actual_rot:.1f} / odom {odom_rot:.1f})')
    print('\nNew values for ~/mbot_firmware_ros/include/config/mbot_classic_config.h:')
    print(f'#define DIFF_WHEEL_RADIUS            {r_new:.5f}   // was {R_OLD}')
    print(f'#define DIFF_BASE_RADIUS             {b_new:.5f}   // was {B_OLD}')


if __name__ == '__main__':
    main()
