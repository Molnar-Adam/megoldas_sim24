import time
import unittest
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from megoldas_sim24.joystick_teleop import JoystickTeleopNode

#source /opt/ros/foxy/setup.bash
#PYTHONPATH=".:install/megoldas_sim24/lib/python3.8/site-packages:$PYTHONPATH" pytest-3 tests/test_joystick_teleop_integration.py -v -ra --tb=short

MSG_TIMEOUT = 10.0

class JoystickTeleopIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_joystick_teleop_node')
        
        self.cmd_vel_received = False
        self.received_cmd_vel_msg = None

        self.cmd_vel_subscriber = self.node.create_subscription(
            Twist,
            'roboworks/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.joy_publisher = self.node.create_publisher(
            Joy,
            'joy',
            10
        )

        self.target_node = JoystickTeleopNode()
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
            if self.joy_publisher.get_subscription_count() > 0:
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

    def publish_joy_and_wait(self, joy, retries=3):
        for _ in range(retries):
            self.cmd_vel_received = False
            self.received_cmd_vel_msg = None
            self.joy_publisher.publish(joy)
            if self.wait_for_cmd_vel():
                return True
        return False

    def create_mock_joy(self, linear=0.5, angular=-0.5):
        joy = Joy()
        joy.header.stamp = self.node.get_clock().now().to_msg()
        joy.header.frame_id = 'joy'
        joy.axes = [angular, linear, 0.0, 0.0]
        joy.buttons = [0] * 10
        return joy

    def test_publish_joy_produces_cmd_vel(self):
        self.assertTrue(self.wait_for_connections(), 'Nincs kapcsolat a joy topicon!')

        joy = self.create_mock_joy(linear=0.5, angular=-0.5)
        success = self.publish_joy_and_wait(joy, retries=3)

        self.assertTrue(success, 'Nem erkezett roboworks/cmd_vel uzenet joy publish utan!')
        self.assertIsNotNone(self.received_cmd_vel_msg)
        self.assertAlmostEqual(self.received_cmd_vel_msg.linear.x, 0.5)
        self.assertAlmostEqual(self.received_cmd_vel_msg.angular.z, -0.5)

    def test_publish_joy_produces_zero_cmd_vel(self):
        self.assertTrue(self.wait_for_connections(), 'Nincs kapcsolat a joy topicon!')

        joy = self.create_mock_joy(linear=0.0, angular=0.0)
        success = self.publish_joy_and_wait(joy, retries=3)

        self.assertTrue(success, 'Nem erkezett roboworks/cmd_vel uzenet joy publish utan!')
        self.assertIsNotNone(self.received_cmd_vel_msg)
        self.assertAlmostEqual(self.received_cmd_vel_msg.linear.x, 0.0)
        self.assertAlmostEqual(self.received_cmd_vel_msg.angular.z, 0.0)

if __name__ == '__main__':
    unittest.main()
