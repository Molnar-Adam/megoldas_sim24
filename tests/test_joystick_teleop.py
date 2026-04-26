import tests.mock_ros  
import pytest
from unittest.mock import MagicMock

from megoldas_sim24.joystick_teleop import JoystickTeleopNode

#python -m pytest tests/test_joystick_teleop.py -v

class TestJoystickTeleopNode:
    
    @pytest.fixture(autouse=True)
    def setup_node(self):
        self.node = JoystickTeleopNode()
    

    def test_joy_callback(self):
        """Ellenőrizzük, hogy a joystick jelre megfelelő sebesség üzenetek generálódnak-e."""
        mock_msg = MagicMock()
        mock_msg.axes = [0.5, 0.8]  

        self.node.joy_callback(mock_msg)
        assert self.node.publisher_.publish.called
        
        published_twist = self.node.publisher_.publish.call_args[0][0]
        assert published_twist.linear.x == 0.8 * self.node.linear_scale
        assert published_twist.angular.z == 0.5 * self.node.angular_scale

    def test_zero_axes_movement(self):
        """Ellenőrizzük, hogy ha a joystick el van engedve, akkor 0 sebességet ad-e ki."""
        mock_msg = MagicMock()
        mock_msg.axes = [0.0, 0.0, 0.0]  

        self.node.joy_callback(mock_msg)
        
        published_twist = self.node.publisher_.publish.call_args[0][0]
        assert published_twist.linear.x == 0.0
        assert published_twist.angular.z == 0.0

    def test_joy_callback_negative_values(self):
        """Ellenőrizzük a joystick negatív kimeneteit (pl. hátra menet, jobbra kanyar)."""
        mock_msg = MagicMock()
        mock_msg.axes = [-0.5, -0.8]  

        self.node.joy_callback(mock_msg)
        
        published_twist = self.node.publisher_.publish.call_args[0][0]
        assert published_twist.linear.x == -0.8 * self.node.linear_scale, "Backward (negative) linear speed failed."
        assert published_twist.angular.z == -0.5 * self.node.angular_scale, "Right (negative) angular speed failed."

    def test_joy_callback_state_publisher(self):
        """Megvizsgálja, hogy a kimeneti debug sztrinkbe belekerülnek-e a numerikus változók."""
        mock_msg = MagicMock()
        mock_msg.axes = [1.0, 1.0]

        self.node.joy_callback(mock_msg)
        
        assert self.node.state_publisher_.publish.called, "Control state String message should be published."
        published_str = self.node.state_publisher_.publish.call_args[0][0].data
        
        assert "Control State" in published_str, "Message header missing."
        assert "velocity:" in published_str, "Velocity debug field missing."
        assert "target_angle:" in published_str, "Target angle debug field missing."

    def test_custom_scales(self):
        """Ellenőrzi a dinamikus skálázási faktor alkalmazását az axes paraméterekkel szemben."""
        self.node.linear_scale = 2.0
        self.node.angular_scale = 0.5
        
        mock_msg = MagicMock()
        mock_msg.axes = [1.0, 1.0]  

        self.node.joy_callback(mock_msg)
        
        published_twist = self.node.publisher_.publish.call_args[0][0]
        assert published_twist.linear.x == 2.0, "Scale for linear x is not multiplying correctly."
        assert published_twist.angular.z == 0.5, "Scale for angular z is not multiplying correctly."
