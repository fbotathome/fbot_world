#!/usr/bin/env python3

import rclpy
from rclpy import time


import rclpy.logging
import tf2_ros

from scripts.world_plugin import WorldPlugin
from fbot_world_msgs.srv import GetKey, GetPose

from geometry_msgs.msg import Pose, PoseStamped, Vector3

import yaml

from math import sqrt





class PosePlugin(WorldPlugin):
  """
    @class PosePlugin
    @brief A plugin for managing and serving target poses and sizes within a robot world model.

    This class provides services to retrieve the position and orientation of targets,
    based on configuration parameters or Redis database entries.
  """

  def __init__(self, node_name: str = 'pose'):
    """
    @brief Constructor for PosePlugin.
    @param node_name The name of the ROS2 node.
    """  
    super().__init__(nodeName=node_name)

    self.declareParameters_targets()
    self.locations = self.readParameters_targets()
    self.declareParameters(self.locations)
    self.readParameters(self.locations)
    self.setStaticPose()
    self.pose_server = self.create_service(GetPose, '/fbot_world/get_pose', self.getPose)
    
    # self.closest_key_server = self.create_service(GetKey, '/fbot_world/get_closest_key', self.getClosestKey)

    self.get_logger().info(f"Node started!!!")

  def readPose(self, key: str):
    """
    @brief Reads pose data for a given key from the Redis database.
    @param key The key identifying the target.
    @return Pose object populated with position and orientation.
    """

    pose = Pose()
    db_pose = self.r.hgetall('target/'+key+'/'+'pose')
    self.get_logger().info(f"Reading pose from r: {self.r}")
    self.get_logger().info(f"Reading pose from db_pose: {db_pose}")
    pose.position.x = float(db_pose[b'px'])
    pose.position.y = float(db_pose[b'py'])
    pose.position.z = float(db_pose[b'pz'])
    pose.orientation.x = float(db_pose[b'ox'])
    pose.orientation.y = float(db_pose[b'oy'])
    pose.orientation.z = float(db_pose[b'oz'])
    pose.orientation.w = float(db_pose[b'ow'])  
    
    return pose

  def readSize(self, key: str):
    '''
    @brief Reads size (scale) data for a given key from the Redis database.
    @param key The key identifying the target.
    @return Vector3 object with x, y, and z sizes.
    '''
    size = Vector3()
    db_size = self.r.hgetall(key)
    try:
      size.x = float(db_size['sx'])
      size.y = float(db_size['sy'])
      size.z = float(db_size['sz'])
    except Exception:
      pass

    return size

  def setStaticPose(self):
    '''
    @brief Stores all target poses as static entries in the Redis database.
    '''
    poses = self.targets
    with self.r.pipeline() as pipe:
      for p_id, pose in poses.items():
        key = 'target/' + p_id + '/' + 'pose'
        pipe.hmset(key, pose)
      pipe.execute()
  
  # def getClosestKey(self, req : GetKey.Request, res : GetKey.Response):
  #   query = req.query 
  #   self.get_logger().info(f"Requisição: {query}")
  #   keys = self.r.keys(query)
  #   rclpy.logging.get_logger('pose_plugin').info("Tipo de cada key: " + str([type(k) for k in keys]))
  #   #rospy.loginfo(keys)
  #   rclpy.logging.get_logger('pose_plugin').info("Keys: " + str(keys))
  #   keys = [k.decode('utf-8') if isinstance(k, bytes) else k for k in keys]
  #   rclpy.logging.get_logger('pose_plugin').info("Tipo de cada key: " + str([type(k) for k in keys]))

  #   rclpy.logging.get_logger('pose_plugin').info("Keys decode: " + str(keys))
  #   keys = list(filter(lambda x: '/pose' in x and 'target' not in x, keys))
  #   #rospy.loginfo(keys)
  #   rclpy.logging.get_logger('pose_plugin').info("Filtered keys: " + str(keys))
  #   min_distance = float('inf')
  #   min_key = ''
  #   for key in keys:
  #     pose = PoseStamped()
  #     pose.header.frame_id = self.fixed_frame
  #     pose.header.stamp = time.Time.now()

  #     pose.pose = self.readPose(key)

  #     t = tf2_ros.TransformerROS()
  #     p = t.transformPose('/map', pose)

  #     distance = sqrt(p.pose.position.x**2 + p.pose.position.y**2 + p.pose.position.z**2)
  #     if distance < min_distance:
  #       min_distance = distance
  #       min_key = key
    
  #   min_key = min_key.replace('/pose', '')
  #   #rospy.loginfo("Key: " + min_key)
  #   rclpy.logging.get_logger('pose_plugin').info("Closest key: " + min_key)
  #   res = GetKey.Response()
  #   res.key = min_key
  #   res.success = True
  #   return res

  def getPose(self, req : GetPose.Request, res : GetPose.Response):
    '''
    @brief Service callback to return the pose and size for a requested target key.
    @param req The service request containing the target key.
    @param res The service response to populate with pose and size.
    @return A filled GetPose.Response object.
    '''
    key = req.key
    res = GetPose.Response()
    pose = self.readPose(key)
    res.pose = pose
    rclpy.logging.get_logger('pose_plugin').info("Pose: " + str(pose))
    size = self.readSize(key)
    res.size = size
    rclpy.logging.get_logger('pose_plugin').info("Size: " + str(size))
    return res

  def declareParameters_targets(self):
    """
    @brief Declares the initial list of target keys as parameters.
    """
    self.declare_parameter('target', rclpy.Parameter.Type.STRING_ARRAY)
  
  def readParameters_targets(self):
    """
    @brief Reads the list of target keys from declared parameters.
    @return A list of target names.
    """
    return self.get_parameter('target').value
  

  def declareParameters(self, locations):
    """
    @brief Declares position and orientation parameters for each target location.
    @param locations A list of target names.
    """
    self.locations = locations
    self.get_logger().info(f"Declaring parameters {self.locations}")
    for target in range(0,len(self.locations)):
      self.declare_parameter('targets.'+ self.locations[target] +'.px', 1.0)
      self.declare_parameter('targets.'+ self.locations[target] +'.py', 1.0)
      self.declare_parameter('targets.'+ self.locations[target] +'.pz', 1.0)
      self.declare_parameter('targets.'+ self.locations[target] +'.ox', 1.0)
      self.declare_parameter('targets.'+ self.locations[target] +'.oy', 1.0)
      self.declare_parameter('targets.'+ self.locations[target] +'.oz', 1.0)
      self.declare_parameter('targets.'+ self.locations[target] +'.ow', 1.0)

  def readParameters(self, locations):
    """
    @brief Reads position and orientation values from parameters for each target.
    @param locations A list of target names.
    """
    self.locations = locations
    self.targets = {}
    for target in  range(len(self.locations)):
      self.targets.update({ self.locations[target]: {
        'px': self.get_parameter('targets.'+ str(self.locations[target]) +'.px').value,
        'py': self.get_parameter('targets.'+ str(self.locations[target]) +'.py').value,
        'pz': self.get_parameter('targets.'+ str(self.locations[target]) +'.pz').value,
        'ox': self.get_parameter('targets.'+ str(self.locations[target]) +'.ox').value,
        'oy': self.get_parameter('targets.'+ str(self.locations[target]) +'.oy').value,
        'oz': self.get_parameter('targets.'+ str(self.locations[target]) +'.oz').value,
        'ow': self.get_parameter('targets.'+ str(self.locations[target]) +'.ow').value,
      }})

  


def main(args=None) -> None: 
    rclpy.init(args=args)
    node = PosePlugin('pose')
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()