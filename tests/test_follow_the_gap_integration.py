import math
import time
import unittest
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from megoldas_sim24.follow_the_gap import FollowTheGapNode

#source /opt/ros/foxy/setup.bash
#PYTHONPATH=".:install/megoldas_sim24/lib/python3.8/site-packages:$PYTHONPATH" pytest-3 tests/test_follow_the_gap_integration.py -v -ra --tb=short

MSG_TIMEOUT = 10.0

class FollowTheGapIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_follow_the_gap_node')
        
        self.cmd_vel_received = False
        self.received_cmd_vel_msg = None

        self.cmd_vel_subscriber = self.node.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.scan_publisher = self.node.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        self.target_node = FollowTheGapNode()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.target_node)
        
        self.exec_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.exec_thread.start()

        time.sleep(1.0)

    def tearDown(self):
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

    def create_mock_scan(self, default_distance=3.0):
        scan = LaserScan()
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (scan.angle_max - scan.angle_min) / 359.0
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.02
        scan.range_max = 10.0
        scan.ranges = [float(default_distance)] * 360
        scan.intensities = []
        return scan

    def test_publish_scan_produces_cmd_vel(self):
        self.assertTrue(self.wait_for_connections(), 'Nincs kapcsolat a /scan topicon!')

        scan = self.create_mock_scan(default_distance=3.0)
        success = self.publish_scan_and_wait(scan, retries=3)

        self.assertTrue(success, 'Nem erkezett cmd_vel uzenet scan publish utan!')
        self.assertIsNotNone(self.received_cmd_vel_msg)

    def test_publish_scan_obstacle_side(self):
        self.assertTrue(self.wait_for_connections(), 'Nincs kapcsolat a /scan topicon!')

        scan = self.create_mock_scan(default_distance=3.0)
        for i in range(225, 315):
            scan.ranges[i] = 0.5
            
        success = self.publish_scan_and_wait(scan, retries=3)

        self.assertTrue(success, 'Nem erkezett cmd_vel uzenet scan publish utan!')
        self.assertIsNotNone(self.received_cmd_vel_msg)
        self.assertLess(self.received_cmd_vel_msg.angular.z, 0.0, 'A robotnak jobbra kell kanyarodnia bal oldali akadalynal!')

if __name__ == '__main__':
    unittest.main()
