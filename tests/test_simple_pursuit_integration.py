#!/usr/bin/env python3
# -*- coding: utf-8 -*-



import math
import time
import unittest
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from megoldas_sim24.simple_pursuit import SimplePursuit

#source /opt/ros/foxy/setup.bash
#PYTHONPATH=".:install/megoldas_sim24/lib/python3.8/site-packages:$PYTHONPATH" pytest-3 tests/test_simple_pursuit_integration.py -v -ra --tb=short

MSG_TIMEOUT = 10.0

class SimplePursuitIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_simple_pursuit_node')
        
        self.cmd_vel_received = False
        self.received_cmd_vel_msg = None

        self.cmd_vel_subscriber = self.node.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.scan_publisher = self.node.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        self.target_node = SimplePursuit()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.target_node)
        
        self.exec_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.exec_thread.start()

        time.sleep(1.0)

    def tearDown(self):
        self.target_node.is_running = False
        self.executor.remove_node(self.target_node)
        self.executor.remove_node(self.node)
        self.target_node.destroy_node()
        self.node.destroy_node()
        
        self.executor.shutdown()
        self.exec_thread.join(timeout=1.0)

    def cmd_vel_callback(self, msg):
        self.received_cmd_vel_msg = msg
        self.cmd_vel_received = True

    def wait_for_connections(self):
        timeout = time.time() + MSG_TIMEOUT
        while time.time() < timeout:
            if self.scan_publisher.get_subscription_count() > 0:
                return True
            time.sleep(0.1)
        return False

    def wait_for_cmd_vel(self):
        timeout = time.time() + MSG_TIMEOUT
        while time.time() < timeout:
            if self.cmd_vel_received:
                return True
            time.sleep(0.1)
        return False

    def publish_scan_and_wait(self, scan, retries=3):
        for _ in range(retries):
            self.cmd_vel_received = False
            self.received_cmd_vel_msg = None
            self.scan_publisher.publish(scan)
            if self.wait_for_cmd_vel():
                return True
        return False

    def create_mock_scan(self, default_distance=2.0):
        scan = LaserScan()
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'

        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi
        scan.angle_increment = (scan.angle_max - scan.angle_min) / 359.0
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.02
        scan.range_max = 10.0

        scan.ranges = [float(default_distance)] * 360
        scan.intensities = []
        return scan

    def set_range_at_degree(self, scan, deg, value):
        rad = math.radians(deg)
        idx = int((rad - scan.angle_min) / scan.angle_increment)
        idx = max(0, min(len(scan.ranges) - 1, idx))
        scan.ranges[idx] = float(value)

    def test_1_node_initialization(self):
        topic_names_and_types = self.node.get_topic_names_and_types()
        topic_names = [name for name, types in topic_names_and_types]

        self.assertIn('/cmd_vel', topic_names, 'A simple_pursuit.py node nem publikal a /cmd_vel topicra!')
        self.assertTrue(self.wait_for_connections(), 'A /scan publisher nem csatlakozott idoben a node-hoz!')

    def test_2_publish_scan_produces_cmd_vel(self):
        self.assertTrue(self.wait_for_connections(), 'Nincs kapcsolat a /scan topicon!')

        scan = self.create_mock_scan(default_distance=2.0)
        success = self.publish_scan_and_wait(scan, retries=3)

        self.assertTrue(success, 'Nem erkezett /cmd_vel uzenet scan publish utan!')
        self.assertIsNotNone(self.received_cmd_vel_msg)
        self.assertTrue(math.isfinite(self.received_cmd_vel_msg.linear.x), 'A linear.x nem lehet nan/inf!')
        self.assertTrue(math.isfinite(self.received_cmd_vel_msg.angular.z), 'Az angular.z nem lehet nan/inf!')

    def test_3_publish_scan_obstacle_ahead(self):
        self.assertTrue(self.wait_for_connections(), 'Nincs kapcsolat a /scan topicon!')

        scan = self.create_mock_scan(default_distance=0.1)
        success = self.publish_scan_and_wait(scan, retries=3)

        self.assertTrue(success, 'Nem erkezett /cmd_vel uzenet scan publish utan!')
        self.assertIsNotNone(self.received_cmd_vel_msg)
        self.assertTrue(math.isfinite(self.received_cmd_vel_msg.linear.x), 'A linear.x nem lehet nan/inf!')
        self.assertTrue(math.isfinite(self.received_cmd_vel_msg.angular.z), 'Az angular.z nem lehet nan/inf!')

if __name__ == '__main__':
    unittest.main()

    def test_3_reverse_zone_close_obstacle_slows_down_significantly(self):
        self.assertTrue(self.wait_for_connections(), 'Nincs kapcsolat a /scan topicon!')

        baseline_scan = self.create_mock_scan(default_distance=2.0)
        baseline_ok = self.publish_scan_and_wait(baseline_scan, retries=3)
        self.assertTrue(baseline_ok, 'Nem erkezett baseline /cmd_vel uzenet!')
        baseline_speed = self.received_cmd_vel_msg.linear.x

        scan = self.create_mock_scan(default_distance=2.0)
        for deg in (170, 175, 185, 190):
            self.set_range_at_degree(scan, deg, 0.1)

        success = self.publish_scan_and_wait(scan, retries=3)

        self.assertTrue(success, 'Nem erkezett /cmd_vel uzenet reverse-zone tesztnel!')
        reduced_speed = self.received_cmd_vel_msg.linear.x

        self.assertLessEqual(
            reduced_speed,
            baseline_speed,
            msg='Kozeli hatso akadaly eseten a sebessegnek csokkennie vagy maradnia kell!',
        )
        self.assertLess(
            reduced_speed,
            0.3,
            msg='Kozeli hatso akadaly eseten alacsony sebesseg varhato (<0.3 m/s)!',
        )

    def test_4_repeated_scans_keep_node_responsive(self):
        self.assertTrue(self.wait_for_connections(), 'Nincs kapcsolat a /scan topicon!')

        first_scan = self.create_mock_scan(default_distance=2.0)
        second_scan = self.create_mock_scan(default_distance=1.2)

        success_first = self.publish_scan_and_wait(first_scan, retries=2)
        self.assertTrue(success_first, 'Az elso scan utan nem erkezett /cmd_vel!')
        first_cmd = self.received_cmd_vel_msg.linear.x

        success_second = self.publish_scan_and_wait(second_scan, retries=2)
        self.assertTrue(success_second, 'A masodik scan utan nem erkezett /cmd_vel!')
        second_cmd = self.received_cmd_vel_msg.linear.x

        self.assertTrue(math.isfinite(first_cmd))
        self.assertTrue(math.isfinite(second_cmd))

if __name__ == '__main__':
    unittest.main()
