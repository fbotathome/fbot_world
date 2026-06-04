import rclpy
from rclpy.node import Node
import redis
from ament_index_python.packages import get_package_share_directory


class WorldPlugin(Node):
    '''
    @brief A ROS 2 node plugin that connects to a Redis database and provides world-related parameters.
    This class initializes a ROS 2 node that connects to a Redis instance using parameters defined
    in the ROS parameter server. It retrieves Redis connection parameters and a fixed frame setting,
    and sets up the connection accordingly.
    '''
     
    def __init__(self, packageName='fbot_world', nodeName='world_plugin'):
        super().__init__(nodeName)
        self.world_plugin_declareParameters()
        self.world_plugin_readParameters()
        self.pkgPath = get_package_share_directory(packageName)
        redis_param = {
        'host': self.redis_param_host,
        'port': self.redis_param_port,
        'db': self.redis_param_db
        }
        self.r = redis.Redis(**redis_param)

  
    def world_plugin_declareParameters(self):
        self.declare_parameter('redis.host', 'localhost')
        self.declare_parameter('redis.port', 6379)
        self.declare_parameter('redis.db', 0)
        self.declare_parameter('fixed_frame', 'map')

    def world_plugin_readParameters(self):
        self.redis_param_host = self.get_parameter('redis.host').value
        self.redis_param_port = self.get_parameter('redis.port').value
        self.redis_param_db = self.get_parameter('redis.db').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
