import sys
from unittest.mock import MagicMock

def setup_mocks():
    mock_rclpy = MagicMock()
    sys.modules['rclpy'] = mock_rclpy
    
    # Kamu Node osztály amiből a többi Node örökölhet
    class DummyNode:
        def __init__(self, node_name):
            self.node_name = node_name
            self.publishers = []
            self.subscriptions = []
            
        def get_name(self):
            return self.node_name
            
        def get_logger(self):
            return MagicMock()
            
        def declare_parameter(self, *args, **kwargs):
            pass
            
        def get_parameter(self, name):
            mock_param = MagicMock()
            if 'neve' in name or 'azon' in name:
                mock_param.get_parameter_value().string_value = "A00"
            elif 'debug' in name or 'is_running' in name:
                mock_param.get_parameter_value().bool_value = True
            else:
                mock_param.get_parameter_value().double_value = 0.5
            return mock_param
            
        def add_on_set_parameters_callback(self, *args):
            pass
            
        def set_parameters(self, *args):
            mock_res = MagicMock()
            mock_res.successful = True
            return [mock_res, mock_res]
            
        def create_subscription(self, msg_type, topic, callback, qos_profile, **kwargs):
            sub = MagicMock()
            sub.topic_name = topic
            self.subscriptions.append(sub)
            
            if not hasattr(self, 'create_subscription_mock'):
                self.create_subscription_mock = MagicMock()
            self.create_subscription_mock(msg_type, topic, callback, qos_profile, **kwargs)
            return sub
            
        def create_publisher(self, msg_type, topic, qos_profile, **kwargs):
            pub = MagicMock()
            pub.topic_name = topic
            self.publishers.append(pub)
            
            if not hasattr(self, 'create_publisher_mock'):
                self.create_publisher_mock = MagicMock()
            self.create_publisher_mock(msg_type, topic, qos_profile, **kwargs)
            return pub
            
        def create_timer(self, timer_period_sec, callback):
            timer = MagicMock()
            if not hasattr(self, 'create_timer_mock'):
                self.create_timer_mock = MagicMock()
            self.create_timer_mock(timer_period_sec, callback)
            return timer
            
        def destroy_node(self):
            pass

    mock_node_module = MagicMock()
    mock_node_module.Node = DummyNode
    sys.modules['rclpy.node'] = mock_node_module
    
    sys.modules['rclpy.time'] = MagicMock()
    sys.modules['rclpy.parameter'] = MagicMock()
    
    sys.modules['rcl_interfaces'] = MagicMock()
    sys.modules['rcl_interfaces.msg'] = MagicMock()
    
    sys.modules['sensor_msgs'] = MagicMock()
    sys.modules['sensor_msgs.msg'] = MagicMock()
    
    sys.modules['std_msgs'] = MagicMock()
    sys.modules['std_msgs.msg'] = MagicMock()
    
    sys.modules['geometry_msgs'] = MagicMock()
    sys.modules['geometry_msgs.msg'] = MagicMock()
    
    sys.modules['visualization_msgs'] = MagicMock()
    sys.modules['visualization_msgs.msg'] = MagicMock()
    
    sys.modules['tf2_ros'] = MagicMock()
    sys.modules['tf2_geometry_msgs'] = MagicMock()

setup_mocks()