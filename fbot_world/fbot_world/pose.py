#!/usr/bin/env python3

import rclpy
import yaml
import os
import numpy as np
from scripts.world_plugin import WorldPlugin
from fbot_world_msgs.msg import FBOTPoses, FBOTRooms, FBOTVertices
from fbot_world_msgs.srv import GetPose, GetPoseFromSet, GetSets, GetRoom
from geometry_msgs.msg import Pose, Vector3, Point
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
    self.room_server = self.create_service(GetRoom, '/fbot_world/get_room', self.getRoom)
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
      self.get_logger().info(f"Setting static poses in Redis for targets: {self.targets['poses'].keys()}")
      for target in self.targets['poses'].keys():
        for p_id, pose in self.targets['poses'][target].items():
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
      - 1: Group set is empty, return all poses in 'targets'
      - 2: Group set not found in targets
      - 3: Exception occurred
    @param req: The service request containing the group set.
    @param res: The service response with a array populate with poses and sizes.
    @return: A filled GetPoseFromSet.Response object.
    ''' 
    try:
      if req.group_set in self.targets['poses'].keys():
        for key in self.targets['poses'][req.group_set]:
          poses = FBOTPoses()
          poses.key = key
          poses.pose = self.readPose(req.group_set, key)
          poses.size = self.readSize(req.group_set, key)
          res.error = 0
          res.pose_array.append(poses)

      elif req.group_set == '' or req.group_set == 'None':
        res.error = 1
        for key in self.targets['poses']['targets'].keys():
          poses = FBOTPoses()
          poses.key = key
          poses.pose = self.readPose('targets', key)
          poses.size = self.readSize('targets', key)
          res.pose_array.append(poses)

      else:
        poses = FBOTPoses()
        poses.key = 'NaN'
        poses.pose, poses.size = self.setResponseError()
        res.pose_array.append(poses)
        res.error = 2
    except Exception as e:
      self.get_logger().error(f"Error in getPoseFromSet: {e}")
      poses = FBOTPoses()
      poses.key = 'NaN'
      poses.pose, poses.size = self.setResponseError()
      res.pose_array.append(poses)
      res.error = 3
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
    @brief: A service that returns all poses and rooms names with postions and places in yaml file
    @param req: The service request
    @param res: The service response
    @return: A array with all poses names and rooms names with postions and places in yaml file
    '''
    
    for target in self.targets['poses'].keys():
      for key in self.targets['poses'][target].keys():
        pose = FBOTPoses()
        pose.key = key
        pose.pose = self.readPose(target, key)
        res.poses.append(pose)
    for room in self.targets['rooms'].keys():
      room_ = FBOTRooms()
      room_.room.key = room
      for vertice in self.targets['rooms'][room]['vertices']:
        point = Point()
        point.x = vertice[0]
        point.y = vertice[1]
        point.z = 0.0
        room_.room.points.append(point)
      for key in self.targets['rooms'][room]['places'].keys():
        place = FBOTVertices()
        place.key = key
        for vertice in self.targets['rooms'][room]['vertices']:
          point = Point()
          point.x = vertice[0]
          point.y = vertice[1]
          point.z = 0.0
          place.points.append(point)
        room_.objects.append(place)
      res.rooms.append(room_)
    return res
  
  def getRoom(self, req: GetRoom.Request, res: GetRoom.Response):
    """
    Executes the state by compare if the point is inside a polygon, and saves the points inside blackboard['inside_polygon'].
    @return Execution outcome (SUCCEED, ABORT).
    """
    pose = Pose()
    pose = req.pose
    for room in self.targets['rooms'].items():
      polygon = np.array(room[1]['vertices'],dtype= np.float32)
      self.itens_points = pose.position
      if self.is_point_in_area(polygon, [self.itens_points.x, self.itens_points.y]):
          for place in room[1]['places'].items():
            subpolygon = np.array(place[1],dtype= np.float32)
            if self.is_point_in_area(subpolygon, [self.itens_points.x, self.itens_points.y]):
              res.response = [room[0], place[0]]
              return res
          res.response = [room[0], 'None']
          return res
    res.response = ['None', 'None']
    return res
      
  def is_point_in_area(self, polygon, point) -> bool:
        """
        @brief A function thats verify if a point are inside a area.
        @param point: Point to verify.
        @return a bool param, if the intersections are a pair value return true, else, return false.
        """
        x, y = point[0], point[1]
        inside = False
        
        n = len(polygon)
        j = n - 1  # Inicia com o último vértice para testar a aresta que fecha o polígono
        
        for i in range(n):
            xi, yi = polygon[i][0], polygon[i][1]
            xj, yj = polygon[j][0], polygon[j][1]
            
            # Condição 1: O ponto Y do robô está entre os Ys da parede?
            # Condição 2: A parede cruza o raio à direita da posição X do robô?
            intersect = ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi)
            
            if intersect:
                # Cada vez que cruza uma parede válida, inverte o status (dentro/fora)
                inside = not inside
                
            j = i  # Avança para a próxima aresta
            
        return inside
       
  def intersect(self, A,B,C,D):
      return self.ccw(A,C,D) != self.ccw(B,C,D) and self.ccw(A,B,C) != self.ccw(A,B,D)
  
  def ccw(self,A,B,C):
      return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])









def main(args=None) -> None: 
    rclpy.init(args=args)
    node = PosePlugin('pose')
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
