import tests.mock_ros  
import pytest
import math
import numpy as np
from types import SimpleNamespace
from unittest.mock import MagicMock

from megoldas_sim24.simple_pursuit import SimplePursuit

class MockLaserScan:
    def __init__(self, ranges, angle_min=-math.pi, angle_max=math.pi):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = (angle_max - angle_min) / len(ranges)

class TestSimplePursuitNode:
    
    @pytest.fixture(autouse=True)
    def setup_node(self):
        self.node = SimplePursuit()
        import megoldas_sim24.simple_pursuit as sp_module
        sp_module.WHEELBASE = 0.3187
        yield
        

    # calcPointPos tests
    def test_calcPointPos_zero_degree(self):
        """Teszt: 0 fok (jobbra), range = 1.0"""
        x, y = self.node.calcPointPos(1.0, 0.0)
        assert x == pytest.approx(1.0, abs=1e-9)
        assert y == pytest.approx(0.0, abs=1e-9)
    
    def test_calcPointPos_90_degree(self):
        """Teszt: 90 fok (előre), range = 2.0"""
        x, y = self.node.calcPointPos(2.0, math.radians(90))
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(2.0, abs=1e-9)
    
    def test_calcPointPos_180_degree(self):
        """Teszt: 180 fok (balra), range = 1.5"""
        x, y = self.node.calcPointPos(1.5, math.radians(180))
        assert x == pytest.approx(-1.5, abs=1e-9)
        assert y == pytest.approx(0.0, abs=1e-9)
    
    def test_calcPointPos_zero_range(self):
        """Teszt: range = 0, bármilyen szög"""
        x, y = self.node.calcPointPos(0.0, math.radians(45))
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(0.0, abs=1e-9)

    def test_calcPointPos_negative_angle(self):
        """Teszt: negatív szög (-90 fok), range = 3.0"""
        x, y = self.node.calcPointPos(3.0, math.radians(-90))
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(-3.0, abs=1e-9)

    # calcPursuitAngle tests
    def test_calcPursuitAngle_straight_ahead(self):
        """Teszt: cél egyenesen előre (x=1.0, y=0.0)"""
        angle = self.node.calcPursuitAngle(1.0, 0.0)
        assert angle == pytest.approx(0.0)

    def test_calcPursuitAngle_sideway(self):
        """Teszt: cél oldalra (x=0.0, y=1.0)"""
        angle = self.node.calcPursuitAngle(0.0, 1.0)
        WHEELBASE = 0.3187
        alpha = math.atan2(1.0, 0.0)
        ld = math.sqrt(0.0**2 + 1.0**2)
        expected = math.atan2(2.0 * WHEELBASE * math.sin(alpha) / ld, 1)
        assert angle == pytest.approx(expected, abs=1e-4)  

    def test_calcPursuitAngle_diagonal(self):
        """Teszt: cél Átlósan (x=1.0, y=1.0) - 45 fok"""
        angle = self.node.calcPursuitAngle(1.0, 1.0)
        WHEELBASE = 0.3187
        alpha = math.atan2(1.0, 1.0)
        ld = math.sqrt(1.0**2 + 1.0**2)
        expected = math.atan2(2.0 * WHEELBASE * math.sin(alpha) / ld, 1)
        assert angle == pytest.approx(expected, abs=1e-4) 

    def test_calcPursuitAngle_close_distance(self):
        """Teszt: cél rövid távon, bármilyen irányba (x=0.0, y=0.1)"""
        angle = self.node.calcPursuitAngle(0.0, 0.1)
        WHEELBASE = 0.3187
        expected = math.atan2(2.0 * WHEELBASE * math.sin(math.atan2(0.1, 0.0)) / 0.1, 1)
        assert angle == pytest.approx(expected, abs=1e-4) 

    def test_calcPursuitAngle_far_distance(self):
        """Teszt: cél hosszú távon, bármilyen irányba (x=100.0, y=10.0)"""
        angle = self.node.calcPursuitAngle(100.0, 10.0)
        WHEELBASE = 0.3187
        expected = math.atan2(2.0 * WHEELBASE * math.sin(math.atan2(10.0, 100.0)) / math.sqrt(100**2 + 10**2), 1)
        assert angle == pytest.approx(expected, abs=1e-4) 

    # Core logic tests
    def test_get_distance_short_scan_returns_default(self):
        ranges = [1.0] * 10
        angles = np.linspace(-math.pi, math.pi, len(ranges), endpoint=False)
        distance = self.node.getDistance(ranges, angles)
        assert distance == pytest.approx(0.4)

    def test_get_angle_symmetric_environment(self):
        ranges = [2.0] * 360
        angles = np.linspace(0, 2 * math.pi, len(ranges), endpoint=True)
        angle, left_d, right_d = self.node.getAngle(ranges, angles)
        
        assert angle == pytest.approx(0.357, abs=0.05)
        assert left_d is not None
        assert right_d is not None

    def test_follow_simple_applies_transform_and_smoothing(self, monkeypatch):
        mock_scan = MockLaserScan([1.0] * 360)

        monkeypatch.setattr(self.node, 'getDistance', lambda ranges, angles: -1.0)
        monkeypatch.setattr(self.node, 'getAngle', lambda ranges, angles: (0.2, 1.0, -1.0))

        import megoldas_sim24.simple_pursuit as sp_module
        def fake_transform(point_stamped, _):
            return SimpleNamespace(point=SimpleNamespace(x=2.0, y=1.0))
        monkeypatch.setattr(sp_module.tf2_geometry_msgs, 'do_transform_point', fake_transform)

        def fake_pursuit_angle(goal_x, goal_y):
            assert goal_x == pytest.approx(2.0)
            assert goal_y == pytest.approx(1.0)
            return 0.6
        monkeypatch.setattr(self.node, 'calcPursuitAngle', fake_pursuit_angle)

        self.node.prev_steering_err = 0.0
        self.node.prev_velocity = 0.0
        steering_err, velocity = self.node.followSimple(mock_scan)

        assert steering_err == pytest.approx(0.3)
        assert velocity == pytest.approx(1.0)

        assert self.node.prev_steering_err == pytest.approx(0.3)
        assert self.node.prev_velocity == pytest.approx(1.0)

    def test_callback_laser_publishes_scaled_cmd_vel(self, monkeypatch):
        class DummyTwist:
            def __init__(self):
                self.linear = SimpleNamespace(x=0.0)
                self.angular = SimpleNamespace(z=0.0)

        monkeypatch.setattr(self.node, 'followSimple', lambda data: (0.4, 2.0))
        import megoldas_sim24.simple_pursuit as sp_module
        monkeypatch.setattr(sp_module, 'Twist', DummyTwist)
        
        self.node.pub = MagicMock()
        self.node.is_running = True

        self.node.callbackLaser(SimpleNamespace())

        assert self.node.pub.publish.called
        published_msg = self.node.pub.publish.call_args[0][0]
        assert published_msg.linear.x == pytest.approx(1.0)
        assert published_msg.angular.z == pytest.approx(0.4)

