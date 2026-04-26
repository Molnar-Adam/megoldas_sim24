import tests.mock_ros  # Ez elvégzi az összes ROS függőség kigúnyolását (mockolását)
import pytest

from megoldas_sim24.follow_the_gap import FollowTheGapNode

#python -m pytest tests/test_follow_the_gap.py -v

class TestFollowTheGapNodeMock:
    
    @pytest.fixture(autouse=True)
    def setup_node(self, monkeypatch):
        class DummyString:
            def __init__(self, data=""):
                self.data = data
        import megoldas_sim24.follow_the_gap as fg_module
        monkeypatch.setattr(fg_module, 'String', DummyString)

        self.node = FollowTheGapNode()
        
        self.node.kozepiskola_neve = "Ismeretlen kozepiskola"
        self.node.kozepiskola_azon = "A99"
        self.node.debug = False
        self.node.safety_radius = 2.0
        self.node.max_throttle = 0.5
        self.node.steering_sensitivity = 0.7
        self.node.max_steering_angle = 0.52
        self.node.is_running = True

    def test_find_best_gap_straight(self):
        """Teszteli a rést kereső algoritmust ideális egyenes folyosó (akadálymentes) esetben."""
        import numpy as np
        # 100 elemű lidar szken, min = -π/2, max = +π/2
        ranges = np.ones(100) * 10.0 
        angle_min = -np.pi / 2
        angle_increment = np.pi / 100
        
        best_angle = self.node.find_best_gap(ranges, angle_min, angle_increment)
        
        assert abs(best_angle) < 0.1, f"Expected near 0.0 angle, got {best_angle}"

    def test_find_best_gap_obstacle(self):
        """Teszteli, mit reagál az algoritmus egy jobb oldali aszimmetrikus akadály esetén."""
        import numpy as np
        ranges = np.ones(100) * 10.0 
        
        ranges[0:50] = 1.0 

        angle_min = -np.pi / 2
        angle_increment = np.pi / 100
        
        best_angle = self.node.find_best_gap(ranges, angle_min, angle_increment)
        
        assert best_angle > 0.0, "The best gap should lean to the left (> 0.0 rad) when obstacle is on the right."

    def test_find_best_gap_no_safe_path(self):
        """Teszteli az algoritmust, amikor sehol sincs biztonságos útvonal (minden távolság < safety_radius)."""
        import numpy as np
        ranges = np.ones(100) * 1.0 
        angle_min = -np.pi / 2
        angle_increment = np.pi / 100
        
        best_angle = self.node.find_best_gap(ranges, angle_min, angle_increment)
        
        assert best_angle == 0.0, "The node should return 0.0 angle when blocked everywhere."

    def test_find_best_gap_ignores_behind(self):
        """Teszteli, hogy a robot háta mögötti tartományokat ([-pi, -p/2] és [pi/2, pi]) kizárja-e."""
        import numpy as np
        ranges = np.ones(360) * 1.0  
        ranges[0:30] = 10.0         
        ranges[330:360] = 10.0  

        angle_min = -np.pi          
        angle_increment = (2 * np.pi) / 360
        
        best_angle = self.node.find_best_gap(ranges, angle_min, angle_increment)
        
        assert best_angle == 0.0, "The node must ignore safe gaps that are physically behind the robot."

    def test_publish_drive_command_math(self):
        """Ellenőrzi a kimeneti sebesség és kormányzási parancs numerikus helyességét."""
        self.node.max_throttle = 1.0
        self.node.steering_sensitivity = 0.5
        
        self.node.publish_drive_command(best_angle=1.0)
        
        assert self.node.cmd_pub.publish.called, "The Twist message was not published."
        twist_msg = self.node.cmd_pub.publish.call_args[0][0]
        
        assert twist_msg.linear.x == 1.0, "Linear velocity should be equal to max_throttle."
        assert twist_msg.angular.z == 0.5, "Angular velocity should be best_angle * steering_sensitivity."

    def test_publish_drive_command_is_not_running(self):
        """Ellenőrzi, hogy ha a node le van állítva (is_running=False), nem küld mozgásparancsot."""
        self.node.is_running = False
        
        self.node.publish_drive_command(best_angle=1.0)
        
        assert not self.node.cmd_pub.publish.called, "Should not publish cmd_vel when is_running is False."
        assert self.node.pubst2.publish.called, "Should still publish school name even if not running."

    def test_publish_drive_command_school_string(self):
        """Ellenőrzi a középiskola nevének és azonosítójának helyes formázását a pubst2 kimeneten."""
        self.node.kozepiskola_neve = "Teszt Suli"
        self.node.kozepiskola_azon = "B12"
        
        self.node.publish_drive_command(best_angle=0.0)
        
        assert self.node.pubst2.publish.called, "School string was not published."
        school_msg = self.node.pubst2.publish.call_args[0][0]
        
        assert school_msg.data == "Teszt Suli (B12)", "The school string format is incorrect."

    def test_publish_drive_command_negative_angle(self):
        """Ellenőrzi a negatív kormányzási szög helyes skálázását és kimenetét (jobbra kanyarodás)."""
        self.node.max_throttle = 0.8
        self.node.steering_sensitivity = 0.5
        
        self.node.publish_drive_command(best_angle=-2.0)
        
        twist_msg = self.node.cmd_pub.publish.call_args[0][0]
        assert twist_msg.linear.x == 0.8, "Linear velocity should remain max_throttle."
        assert twist_msg.angular.z == -1.0, "Negative angular velocity scaling is incorrect (-2.0 * 0.5 = -1.0)."