#!/usr/bin/env python3
"""Record PX4 telemetry (via px4_ros_com / FastRTPS bridge) to CSV for MATLAB.

Run in a second terminal BEFORE starting the drone app:

    cd ~/px4_drone_simulation_ws && . ./install/setup.bash
    python3 /path/to/matlab/tools/telemetry_logger.py --out ~/drone_logs

Files written (one row per received message):
    gps.csv       VehicleGpsPosition_PubSubTopic  (lat/lon in 1e-7 deg, alt in mm)
    attitude.csv  VehicleAttitude_PubSubTopic     (quaternion + body rates)
    odometry.csv  VehicleOdometry_PubSubTopic     (local NED position/velocity)
    battery.csv   BatteryStatus_PubSubTopic
    status.csv    VehicleStatus_PubSubTopic       (arming / nav state)
Stop with Ctrl+C.
"""
import argparse
import csv
import os
import time

import rclpy
from rclpy.node import Node

from px4_msgs.msg import (BatteryStatus, VehicleAttitude, VehicleGpsPosition,
                          VehicleOdometry, VehicleStatus)


def g(msg, name, default=float('nan')):
    """Defensive getattr: field names differ slightly between px4_msgs versions."""
    return getattr(msg, name, default)


class TelemetryLogger(Node):
    def __init__(self, out_dir):
        super().__init__('telemetry_logger')
        os.makedirs(out_dir, exist_ok=True)
        self.t0 = time.time()
        self.files, self.writers = [], {}

        def open_csv(name, header):
            f = open(os.path.join(out_dir, name + '.csv'), 'w', newline='')
            w = csv.writer(f)
            w.writerow(header)
            self.files.append(f)
            self.writers[name] = w

        open_csv('gps', ['t_wall', 'px4_timestamp_us', 'lat_e7', 'lon_e7', 'alt_mm',
                         'vel_n', 'vel_e', 'vel_d', 'vel_m_s', 'fix_type', 'sats'])
        open_csv('attitude', ['t_wall', 'px4_timestamp_us', 'qw', 'qx', 'qy', 'qz',
                              'rollspeed', 'pitchspeed', 'yawspeed'])
        open_csv('odometry', ['t_wall', 'px4_timestamp_us', 'x', 'y', 'z',
                              'vx', 'vy', 'vz'])
        open_csv('battery', ['t_wall', 'px4_timestamp_us', 'voltage_v', 'current_a',
                             'remaining'])
        open_csv('status', ['t_wall', 'px4_timestamp_us', 'arming_state',
                            'nav_state'])

        self.create_subscription(VehicleGpsPosition, 'VehicleGpsPosition_PubSubTopic',
                                 self.on_gps, 10)
        self.create_subscription(VehicleAttitude, 'VehicleAttitude_PubSubTopic',
                                 self.on_att, 10)
        self.create_subscription(VehicleOdometry, 'VehicleOdometry_PubSubTopic',
                                 self.on_odom, 10)
        self.create_subscription(BatteryStatus, 'BatteryStatus_PubSubTopic',
                                 self.on_batt, 10)
        self.create_subscription(VehicleStatus, 'VehicleStatus_PubSubTopic',
                                 self.on_status, 10)
        self.get_logger().info('Logging to %s (Ctrl+C to stop)' % out_dir)

    def now(self):
        return time.time() - self.t0

    def on_gps(self, m):
        self.writers['gps'].writerow([
            self.now(), m.timestamp, m.lat, m.lon, m.alt,
            g(m, 'vel_n_m_s'), g(m, 'vel_e_m_s'), g(m, 'vel_d_m_s'),
            g(m, 'vel_m_s'), g(m, 'fix_type'), g(m, 'satellites_used')])

    def on_att(self, m):
        q = list(m.q)
        self.writers['attitude'].writerow([
            self.now(), m.timestamp, q[0], q[1], q[2], q[3],
            g(m, 'rollspeed'), g(m, 'pitchspeed'), g(m, 'yawspeed')])

    def on_odom(self, m):
        self.writers['odometry'].writerow([
            self.now(), m.timestamp, g(m, 'x'), g(m, 'y'), g(m, 'z'),
            g(m, 'vx'), g(m, 'vy'), g(m, 'vz')])

    def on_batt(self, m):
        self.writers['battery'].writerow([
            self.now(), m.timestamp, g(m, 'voltage_v'), g(m, 'current_a'),
            g(m, 'remaining')])

    def on_status(self, m):
        self.writers['status'].writerow([
            self.now(), m.timestamp, g(m, 'arming_state'), g(m, 'nav_state')])

    def close(self):
        for f in self.files:
            f.close()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out', default='drone_logs', help='output directory')
    args = p.parse_args()
    rclpy.init()
    node = TelemetryLogger(os.path.expanduser(args.out))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
