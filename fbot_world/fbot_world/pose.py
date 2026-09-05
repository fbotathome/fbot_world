#!/usr/bin/env python3

import rclpy
import yaml
import os
import numpy as np
from pathlib import Path
from fbot_world import marker_utils
from world_scripts.world_plugin import WorldPlugin
from fbot_world_msgs.msg import FBOTPoses, FBOTRooms, DBPose
from fbot_world_msgs.srv import GetPose, GetPoseFromSet, GetSets, GetRoom
from geometry_msgs.msg import Pose, Vector3, Point
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
  

qos = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

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

  # Predefined colors for rooms (RGBA, 0.0–1.0)
  ROOM_COLORS = [
    (0.2, 0.6, 1.0, 0.35),   # blue
    (0.2, 1.0, 0.4, 0.35),   # green
    (1.0, 0.6, 0.1, 0.35),   # orange
    (0.9, 0.2, 0.9, 0.35),   # magenta
    (0.1, 0.9, 0.9, 0.35),   # cyan
    (1.0, 1.0, 0.1, 0.35),   # yellow
  ]

  OBJECT_COLORS = [
    (1.0, 0.2, 0.2, 0.45),   # red
    (1.0, 0.8, 0.2, 0.45),   # amber
    (0.6, 0.2, 1.0, 0.45),   # purple
    (0.2, 1.0, 0.8, 0.45),   # teal
  ]

  def __init__(self, node_name: str = 'pose'):
    """
    @brief: Constructor for PosePlugin.
    @param: node_name: The name of the ROS2 node.
    """  
    super().__init__(nodeName=node_name)
    self.declareParameters()
    self.readParameters()
    ws_dir = os.path.abspath(os.path.join(get_package_share_directory('fbot_world'), '../../../..'))
    self.file_path = Path(os.path.join(ws_dir, "src", "fbot_world","fbot_world", "config", self.config_file_name + '.yaml')).resolve()
    self.get_logger().info(f"File name: {self.config_file_name}")
    self.last_modification = 0.0
    self.data_config = {}
    self.verifyYamlFile()


    self.setStaticPose()

    # --- Marker publisher ---
    self.marker_pub = self.create_publisher(MarkerArray, '/fbot_world/debug_markers', qos)
    self.pose_server = self.create_service(GetPose, '/fbot_world/get_pose', self.getPose)
    self.set_server = self.create_service(GetPoseFromSet, '/fbot_world/get_set', self.getPoseFromSet)
    self.sets_names = self.create_service(GetSets, '/fbot_world/get_groups_names', self.getGroupNames)
    self.room_server = self.create_service(GetRoom, '/fbot_world/get_room', self.getRoom)

    # Publish once after a short delay so RViz has time to subscribe
    self.create_timer(1.0, self._publish_debug_markers_once)

    self.timer_checagem = self.create_timer(5.0, self.timerReadYamlFile)
    self.get_logger().info(f"Pose node started!!!")

  # ------------------------------------------------------------------
  # Debug marker helpers
  # ------------------------------------------------------------------

  def _make_color(self, rgba: tuple) -> ColorRGBA:
    return marker_utils.make_color(rgba)

  def _make_lifetime_forever(self) -> Duration:
    """Duration(sec=0, nanosec=0) means the marker lives forever."""
    return marker_utils.lifetime_forever()

  def _polygon_to_line_strip_marker(
    self,
    polygon: list,
    marker_id: int,
    ns: str,
    color: tuple,
    z: float = 0.0,
    line_width: float = 0.05,
  ) -> Marker:
    """
    @brief: Creates a LINE_STRIP marker that traces a closed polygon.
    @param polygon: List of [x, y] vertices.
    @param marker_id: Unique integer ID within the namespace.
    @param ns: Marker namespace string.
    @param color: (r, g, b, a) tuple.
    @param z: Height at which to draw the polygon.
    @param line_width: Width of the lines in meters.
    @return: Populated Marker message.
    """
    return marker_utils.polygon_line_strip(polygon, marker_id, ns, color, z=z, line_width=line_width)

  def _polygon_to_fill_marker(
    self,
    polygon: list,
    marker_id: int,
    ns: str,
    color: tuple,
    z: float = 0.0,
  ) -> Marker:
    """
    @brief: Creates a TRIANGLE_LIST marker that fills a convex polygon.
    Uses a fan triangulation from the centroid — works well for convex/mildly concave shapes.
    @param polygon: List of [x, y] vertices.
    @param marker_id: Unique integer ID within the namespace.
    @param ns: Marker namespace string.
    @param color: (r, g, b, a) tuple.
    @param z: Height at which to draw the fill.
    @return: Populated Marker message.
    """
    return marker_utils.polygon_fill(polygon, marker_id, ns, color, z=z)

  def _text_marker(
    self,
    text: str,
    position: tuple,
    marker_id: int,
    ns: str,
    color: tuple,
    z: float = 0.3,
    text_size: float = 0.25,
  ) -> Marker:
    """
    @brief: Creates a TEXT_VIEW_FACING marker at a given position.
    @param text: Label string to display.
    @param position: (x, y) centroid for the text.
    @param marker_id: Unique integer ID within the namespace.
    @param ns: Marker namespace string.
    @param color: (r, g, b, a) tuple.
    @param z: Height offset for the text.
    @param text_size: Size of the text in meters.
    @return: Populated Marker message.
    """
    return marker_utils.text_marker(text, position, marker_id, ns, z=z, text_size=text_size)

  def _centroid(self, polygon: list) -> tuple:
    return marker_utils.centroid(polygon)

  def _publish_debug_markers_once(self):
    """
    @brief: Builds and publishes a MarkerArray covering all rooms and their
            sub-areas (objects) loaded from the YAML file. Called once via timer.
    """
    marker_array = MarkerArray()
    marker_id = 0

    rooms = self.targets.get('rooms', {})
    for room_idx, (room_name, room_data) in enumerate(rooms.items()):
      room_color = self.ROOM_COLORS[room_idx % len(self.ROOM_COLORS)]
      room_vertices = room_data.get('vertices', [])

      if room_vertices:
        # Filled polygon (z slightly below outline so it doesn't z-fight)
        marker_array.markers.append(
          self._polygon_to_fill_marker(room_vertices, marker_id, 'rooms', room_color, z=0.01)
        )
        marker_id += 1

        # Outline
        marker_array.markers.append(
          self._polygon_to_line_strip_marker(room_vertices, marker_id, 'rooms', room_color, z=0.02, line_width=0.06)
        )
        marker_id += 1

        # Room label at centroid
        cx, cy = self._centroid(room_vertices)
        marker_array.markers.append(
          self._text_marker(room_name, (cx, cy), marker_id, 'rooms', room_color, z=0.5, text_size=0.3)
        )
        marker_id += 1

      # Sub-areas (objects) inside the room
      objects = room_data.get('objects', {})
      for obj_idx, (obj_name, obj_vertices) in enumerate(objects.items()):
        if not obj_vertices:
          continue
        obj_color = self.OBJECT_COLORS[obj_idx % len(self.OBJECT_COLORS)]

        marker_array.markers.append(
          self._polygon_to_fill_marker(obj_vertices, marker_id, f'room_{room_name}_objects', obj_color, z=0.03)
        )
        marker_id += 1

        marker_array.markers.append(
          self._polygon_to_line_strip_marker(obj_vertices, marker_id, f'room_{room_name}_objects', obj_color, z=0.04, line_width=0.04)
        )
        marker_id += 1

        # Object label
        cx, cy = self._centroid(obj_vertices)
        marker_array.markers.append(
          self._text_marker(f"{room_name}\n{obj_name}", (cx, cy), marker_id, f'room_{room_name}_objects', obj_color, z=0.25, text_size=0.18)
        )
        marker_id += 1

    self.marker_pub.publish(marker_array)
    self.get_logger().info(f"Published {len(marker_array.markers)} debug markers ({len(rooms)} rooms).")

    # Cancel timer so we only publish once (markers persist forever)
    self._debug_timer.cancel()

  # Store timer ref so we can cancel it inside the callback
  def create_timer(self, period, callback):
    timer = super().create_timer(period, callback)
    if callback == self._publish_debug_markers_once:
      self._debug_timer = timer
    return timer

  # ------------------------------------------------------------------
  # Existing methods (unchanged)
  # ------------------------------------------------------------------

  def loadTargets(self):
    content = readYamlFile(self.file_path)
    if 'poses' not in content:
      self.targets = {
        'poses': content,
        'rooms': {}
      }
    else:
      self.targets = content

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
      if req.group_set not in self.targets['poses'].keys():
        self.get_logger().error("Group Set not found in targets: " + str(self.targets['poses'].keys()))
        res.error = 2
        res.pose, res.size = self.setResponseError()
        return res 
      if req.key not in self.targets['poses'][req.group_set].keys():
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
    @brief: A service that returns all poses and rooms names with postions and objects in yaml file
    @param req: The service request
    @param res: The service response
    @return: A array with all poses names and rooms names with postions and objects in yaml file
    '''
    for target in self.targets['poses'].keys():
      pose = DBPose()
      pose.type = target
      for key in self.targets['poses'][target].keys():
        pose.poses.append(key)
      res.poses.append(pose)
    for room in self.targets['rooms'].keys():
      room_ = FBOTRooms()
      room_.room = room
      if 'objects' in self.targets['rooms'][room].keys():
        for object in self.targets['rooms'][room]['objects'].keys():
          room_.objects.append(object)
      if 'poses' in self.targets['rooms'][room].keys():
        for pose in self.targets['rooms'][room]['poses']:
          room_.poses.append(pose)
      res.rooms.append(room_)
    return res
  
  def getRoom(self, req: GetRoom.Request, res: GetRoom.Response):
    """
    @brief: A service that returns the room and object sub-area containing the pose provided in the request.
    The function checks whether `req.pose.position` lies within any room polygon and then any object sub-area polygon.
    @param req: The service request containing the pose to query.
    @param res: The service response to populate with the room and object sub-area names.
    """
    pose = Pose()
    pose = req.pose
    for room in self.targets['rooms'].items():
      polygon = np.array(room[1]['vertices'],dtype= np.float32)
      self.itens_points = pose.position
      if self.is_point_in_area(polygon, [self.itens_points.x, self.itens_points.y]):
          for place in room[1]['objects'].items():
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
        j = n - 1
        for i in range(n):
            xi, yi = polygon[i][0], polygon[i][1]
            xj, yj = polygon[j][0], polygon[j][1]
            intersect = ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi)
            if intersect:
                inside = not inside
            j = i
        return inside
       
  def intersect(self, A,B,C,D):
      return self.ccw(A,C,D) != self.ccw(B,C,D) and self.ccw(A,B,C) != self.ccw(A,B,D)
  
  def ccw(self,A,B,C):
      return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
  
  def verifyYamlFile(self):
    """
    @brief: Verifies if the YAML file has been modified and loads new configurations if it has.
      This method checks the last modification time of the YAML file against a stored timestamp.
      If the file has been modified since the last check, it reads the new data and updates the internal state.
    """
    if not self.file_path.exists():
        self.get_logger().error(f'File not found: {self.file_path}')
        return

    try:
        actual_modification_time = self.file_path.stat().st_mtime
        if actual_modification_time != self.last_modification:
            with open(self.file_path, 'r', encoding='utf-8') as f:
                new_data = yaml.safe_load(f)
            
            self.data_config = new_data
            self.last_modification = actual_modification_time
            self.get_logger().info('Yaml modified! New configurations loaded.')
            self.applyNewConfigurations()

    except Exception as e:
        self.get_logger().error(f'Error occurred while reading or processing the YAML file: {e}')

  def timerReadYamlFile(self):
    """
    @brief: Timer callback to periodically check for modifications in the YAML file.
      This method is called at regular intervals (e.g., every 5 seconds.
    """
    self.verifyYamlFile()

  def applyNewConfigurations(self):
    """
    @brief: Clear the redis database and applies new configurations loaded from the YAML file.
    """
    self.r.flushdb(asynchronous=True)
    self.targets = readYamlFile(self.file_path)
    self.setStaticPose()



def main(args=None) -> None: 
    rclpy.init(args=args)
    node = PosePlugin('pose')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()