#!/usr/bin/env python3

import rclpy
import rclpy.logging
import yaml
import os
from scripts.world_plugin import WorldPlugin
from fbot_world_msgs.srv import GetPose
from geometry_msgs.msg import Pose, Vector3
from ament_index_python.packages import get_package_share_directory

def readYamlFile(file_path: str = None):
  """
  @brief Reads a YAML file and returns the 'targets' section as a dictionary.
  @param file_path: The path to the YAML file.
  @return A dictionary containing the 'targets' section of the YAML file.
  """
  with open(file_path, 'r') as file:
    yaml_data = yaml.safe_load(file)
  return yaml_data

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
    @param node_name: The name of the ROS2 node.
    """  
    super().__init__(nodeName=node_name)
    self.declareParameters()
    self.readParameters()
    ws_dir = os.path.abspath(os.path.join(get_package_share_directory('fbot_world_bringup'), '../../../..'))
    self.file_path = os.path.join(ws_dir, "src", "fbot_world","fbot_world_bringup", "config", self.config_file_name + '.yaml')
    self.targets = readYamlFile(self.file_path)
    self.get_logger().info(f"File name: {self.config_file_name}")

    self.setStaticPose()
    self.pose_server = self.create_service(GetPose, '/fbot_world/get_pose', self.getTargets)
    self.sitable_server = self.create_service(GetPose, '/fbot_world/get_sitable_pose', self.getSitablePose)
    self.get_logger().info(f"Pose node started!!!")

  def readPose(self, dict: str, key: str):
    """
    @brief Reads pose data for a given key from the Redis database.
    @param key: The key identifying the target.
    @return Pose object populated with position and orientation.
    """

    pose = Pose()
    db_pose = self.r.hgetall(dict+'/'+key+'/'+'pose')
    pose.position.x = float(db_pose[b'px'])
    pose.position.y = float(db_pose[b'py'])
    pose.position.z = float(db_pose[b'pz'])
    pose.orientation.x = float(db_pose[b'ox'])
    pose.orientation.y = float(db_pose[b'oy'])
    pose.orientation.z = float(db_pose[b'oz'])
    pose.orientation.w = float(db_pose[b'ow'])  
    
    return pose

  def declareParameters(self):
    """
    @brief Declares parameters for the PosePlugin node.
    """
    self.declare_parameter('config_file_name', 'pose')
  
  def readParameters(self):
    """
    @brief Reads parameters for the PosePlugin node.
    """
    self.config_file_name = self.get_parameter('config_file_name').get_parameter_value().string_value


  def readSize(self, dict: str, key: str):
    '''
    @brief Reads size (scale) data for a given key from the Redis database.
    @param key: The key identifying the target.
    @return Vector3: object with x, y, and z sizes.
    '''
    size = Vector3()
    db_size = self.r.hgetall(dict+'/'+key)
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
    with self.r.pipeline() as pipe:
      for target in self.targets.keys():
        for p_id, pose in self.targets[target].items():
          key = str(target)+'/' + p_id + '/' + 'pose'
          pipe.hmset(key, pose)
      pipe.execute()

  def getSitablePose(self, req : GetPose.Request, res : GetPose.Response):
    '''
    @brief Service callback to return the list of target keys.
    The function retrieves all keys from the Redis database and returns them in the response.
    @param req: The service request (not used in this case).
    @param res: The service response to populate with target keys.
    @return A filled GetPose.Response object containing the list of target keys.
    '''
    res = GetPose.Response()
    self.get_logger().info("Targets request")
    res = self.getPose(dict = 'sitable_positions', key = req.key)
    return res


  def getTargets(self, req : GetPose.Request, res : GetPose.Response):
    '''
    @brief Service callback to return the list of target keys.
    The function retrieves all keys from the Redis database and returns them in the response.
    @param req: The service request (not used in this case).
    @param res: The service response to populate with target keys.
    @return A filled GetPose.Response object containing the list of target keys.
    '''
    res = GetPose.Response()
    self.get_logger().info("Targets request")
    res = self.getPose(dict = 'targets', key = req.key)
    return res
  
  def getPose(self, dict: str, key: str):
    '''
    @brief Service callback to return the pose and size for a requested target key. 
    The function checks if the key is valid and retrieves the pose and size from Redis.
    If the key is not found or empty, it returns an error code.
    Error codes: 
      - 0: Success
      - 1: Key is empty
      - 2: Key not found in locations
    @param req: The service request containing the target key.
    @param res: The service response to populate with pose and size.
    @return A filled GetPose.Response object.
    '''

    res = GetPose.Response()
    self.get_logger().info(f"Pose request: {key}")
    if key == 'None':
      rclpy.logging.get_logger('pose_plugin').error("Key is empty: " + str(key))
      res.error = 1
      return res
    if key not in self.targets[dict].keys():
      rclpy.logging.get_logger('pose_plugin').error("Key not found in "+dict+": " + str(key))
      res.error = 2
      return res
    res.error = 0
    key = key
    pose = self.readPose(dict, key)
    res.pose = pose
    rclpy.logging.get_logger('pose_plugin').info("Pose: " + str(pose))
    size = self.readSize(dict, key)
    res.size = size
    rclpy.logging.get_logger('pose_plugin').info("Size: " + str(size))
    return res
  


def main(args=None) -> None: 
    rclpy.init(args=args)
    node = PosePlugin('pose')
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()