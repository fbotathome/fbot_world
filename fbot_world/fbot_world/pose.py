#!/usr/bin/env python3

import rclpy
import yaml
import os
from scripts.world_plugin import WorldPlugin
from fbot_world_msgs.msg import FBOTPoses
from fbot_world_msgs.srv import GetPose, GetPoseFromSet, GetSets
from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg  import Empty
from ament_index_python.packages import get_package_share_directory

def readYamlFile(file_path: str = None):
  """
  @brief: Reads a YAML file and returns the 'targets' section as a dictionary.
  @param: file_path: The path to the YAML file.
  @return: A dictionary containing the 'targets' section of the YAML file.
  """
  with open(file_path, 'r') as file:
    yaml_data = yaml.safe_load(file)
  return yaml_data

class PosePlugin(WorldPlugin):
  """
    @class: PosePlugin
    @brief: A plugin for managing and serving target poses and sizes within a robot world model.

    This class provides services to retrieve the position and orientation of targets,
    based on configuration parameters or Redis database entries.
  """

  def __init__(self, node_name: str = 'pose'):
    """
    @brief: Constructor for PosePlugin.
    @param: node_name: The name of the ROS2 node.
    """  
    super().__init__(nodeName=node_name)
    self.declareParameters()
    self.readParameters()
    ws_dir = os.path.abspath(os.path.join(get_package_share_directory('fbot_world'), '../../../..'))
    self.file_path = os.path.join(ws_dir, "src", "fbot_world","fbot_world", "config", self.config_file_name + '.yaml')
    self.targets = readYamlFile(self.file_path)
    self.get_logger().info(f"File name: {self.config_file_name}")

    self.setStaticPose()
    self.pose_server = self.create_service(GetPose, '/fbot_world/get_pose', self.getPose)
    self.set_server = self.create_service(GetPoseFromSet, '/fbot_world/get_set', self.getPoseFromSet)
    self.sets_names = self.create_service(GetSets, '/fbot_world/get_groups_names', self.getGroupNames)
    self.get_logger().info(f"Pose node started!!!")

  def readPose(self, group_set: str, key: str):
    """
    @brief: Reads pose data for a given key from the Redis database.
    @param: group_set: The group set of the target.
    @param: key: The key identifying the target.
    @return: Pose object populated with position and orientation.
    """

    pose = Pose()
    db_pose = self.r.hgetall(group_set+'/'+key+'/'+'pose')
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
    @brief: Declares parameters for the PosePlugin node.
    """
    self.declare_parameter('config_file_name', 'pose')
  
  def readParameters(self):
    """
    @brief: Reads parameters for the PosePlugin node.
    """
    self.config_file_name = self.get_parameter('config_file_name').get_parameter_value().string_value


  def readSize(self, group_set: str, key: str):
    '''
    @brief: Reads size (scale) data for a given key from the Redis database.
    @param: group_set: The group set of the target.
    @param: key: The key identifying the target.
    @return: Vector3: object with x, y, and z sizes.
    '''
    size = Vector3()
    db_size = self.r.hgetall(group_set+'/'+key)
    try:
      size.x = float(db_size['sx'])
      size.y = float(db_size['sy'])
      size.z = float(db_size['sz'])
    except Exception:
      pass
    return size

  def setStaticPose(self):
    '''
    @brief: Stores all target poses as static entries in the Redis database.
    '''
    with self.r.pipeline() as pipe:
      for target in self.targets.keys():
        for p_id, pose in self.targets[target].items():
          key = str(target)+'/' + p_id + '/' + 'pose'
          pipe.hmset(key, pose)
      pipe.execute()

  
  def setResponseError(self):
    '''
    @brief: A function that set a error pose
    @return: return a pose and size filled with NaN values 
    '''
    pose = Pose()
    size = Vector3()
    pose.position.x = float('NaN')
    pose.position.y = float('NaN')
    pose.position.z = float('NaN')
    pose.orientation.x = float('NaN')
    pose.orientation.y = float('NaN')
    pose.orientation.z = float('NaN')
    pose.orientation.w = float('NaN')
    size.x = float('NaN')
    size.y = float('NaN')
    size.z = float('NaN')
    return pose, size

  def getPoseFromSet(self, req: GetPoseFromSet.Request, res: GetPoseFromSet.Response):
    '''
    @brief: Service callback to return all poses for a requested group name key.
    The function checks if the group set is valid and retrieves the poses and sizes from Redis.
    If the group set is not found or empty, it returns a 'NaN.
    Error codes: 
      - 0: Success
      - 1: Group set is empty
      - 2: Group set not found in targets
    @param req: The service request containing the group set.
    @param res: The service response with a array populate with poses and sizes.
    @return: A filled GetPoseFromSet.Response object.
    ''' 
    
    if req.group_set in self.targets.keys():
      for key in self.targets[req.group_set]:
        poses = FBOTPoses()
        poses.key = key
        poses.pose = self.readPose(req.group_set, key)
        poses.size = self.readSize(req.group_set, key)
        res.error = 0
        res.pose_array.append(poses)

    elif req.group_set == '' or req.group_set == 'None':
      poses = FBOTPoses()
      poses.key = 'NaN'
      poses.pose, poses.size = self.setResponseError()
      res.pose_array.append(poses)
      res.error = 1

    else:
      poses = FBOTPoses()
      poses.key = 'NaN'
      poses.pose, poses.size = self.setResponseError()
      res.pose_array.append(poses)
      res.error = 2
      
    return res
  
  def getPose(self, req: GetPose.Request, res: GetPose.Response):
    '''
    @brief: Service callback to return the pose and size for a requested target key. 
    The function checks if the key is valid and retrieves the pose and size from Redis.
    If the key is not found or empty, it returns an error code.
    Error codes: 
      - 0: Success
      - 1: Key is empty
      - 2: Group set not found in targets
      - 3: Key not found in the specified group
    @param req: The service request containing the target key.
    @param res: The service response to populate with pose and size.
    @return: A filled GetPose.Response object.
    '''

    if req.key == '' or req.key == 'None':
      self.get_logger().error("The key is empty.")
      res.error = 1
      res.pose, res.size = self.setResponseError()
      return res
    
    else:
      if req.group_set == '' or req.group_set == 'None':
        req.group_set = 'targets'
        self.get_logger().warning("Class is not specified, using 'targets' as default")

      if req.group_set not in self.targets.keys():
        self.get_logger().error("Group Set not found in targets: " + str(self.targets.keys()))
        res.error = 2
        res.pose, res.size = self.setResponseError()
        return res 
        
      if req.key not in self.targets[req.group_set].keys():
        self.get_logger().error("Key not found in "+req.group_set+": " + str(req.key))
        res.error = 3
        res.pose, res.size = self.setResponseError()
        return res
      res.error = 0
      pose = self.readPose(req.group_set, req.key)
      res.pose = pose
      size = self.readSize(req.group_set, req.key)
      res.size = size
      return res

  def getGroupNames(self, req: GetSets.Request, res: GetSets.Response):
    '''
    @brief: A service that returns all groups names in yaml file
    @param req: The service request
    @param res: The service response
    @return: A array with all group names
    '''
    res.response = self.targets.keys()
    return res
  


def main(args=None) -> None: 
    rclpy.init(args=args)
    node = PosePlugin('pose')
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
