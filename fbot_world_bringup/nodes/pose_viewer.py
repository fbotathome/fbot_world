#!/usr/bin/env python3

import rclpy
import rclpy.clock
import rclpy.duration
import tf2_ros as tf
import numpy as np  
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3

from visualization_msgs.msg import Marker, MarkerArray
from rclpy.exceptions import ROSInterruptException
from scripts.world_plugin import WorldPlugin
from builtin_interfaces.msg import Duration
from rclpy.time import Time
import time
import redis

class ViewerReaderPlugin(WorldPlugin): 
    def __init__(self, nodeName='pose_viewer'):
        super().__init__(nodeName=nodeName)
        self.markerPublisher = self.create_publisher(Marker,'/fbot_vision/fr/markers',qos_profile=10)   
        self.markersPublisher = self.create_publisher(MarkerArray,'/fbot_vision/fr/markers2',qos_profile=10)    
        self.nodeName= nodeName
        self.pose_key = []
        # rate = self.create_rate(30)
        pose_keys = self.r.keys(b'*/pose')
        for key in pose_keys:
            pose = Pose()
            self.get_logger().info(f"Reading pose from key: {key}")
            link = key.replace(b'/pose', b'')
            db_pose = self.r.hgetall(key)
            pose.position.x = float(db_pose[b'px'])
            pose.position.y = float(db_pose[b'py'])
            pose.position.z = float(db_pose[b'pz'])
            pose.orientation.x = float(db_pose[b'ox'])
            pose.orientation.y = float(db_pose[b'oy'])
            pose.orientation.z = float(db_pose[b'oz'])
            pose.orientation.w = float(db_pose[b'ow'])
            self.pose_key.append(pose)
        self.publishMarkers(self.pose_key)
        
        # while rclpy.ok():
        #     rclpy.spin_once(self)
        #     pose_keys = self.r.keys(b'*/pose')
        #     pose = Pose()
        #     for key in pose_keys:
        #         self.get_logger().info(f"Reading pose from key: {key}")
        #         link = key.replace(b'/pose', b'')
        #         db_pose = self.r.hgetall(key)
        #         pose.position.x = float(db_pose[b'px'])
        #         pose.position.y = float(db_pose[b'py'])
        #         pose.position.z = float(db_pose[b'pz'])
        #         pose.orientation.x = float(db_pose[b'ox'])
        #         pose.orientation.y = float(db_pose[b'oy'])
        #         pose.orientation.z = float(db_pose[b'oz'])
        #         pose.orientation.w = float(db_pose[b'ow'])
                # self.pose_key.append(pose)
                # p = pose.position
                # o = pose.orientation

                # transform_stamped = TransformStamped()
                # transform_stamped.header.stamp = self.get_clock().now().to_msg()
                # transform_stamped.header.frame_id = self.fixed_frame
                # transform_stamped.child_frame_id = self.nodeName
                # transform_stamped.transform.translation.x = p.x
                # transform_stamped.transform.translation.y = p.y
                # transform_stamped.transform.translation.z = p.z
                # transform_stamped.transform.rotation.x = o.x
                # transform_stamped.transform.rotation.y = o.y
                # transform_stamped.transform.rotation.z = o.z
                # transform_stamped.transform.rotation.w = o.w
                # #self.get_logger().info(f"Broadcasting {key}: {transform_stamped}")
                # lista_1.append(transform_stamped)

            # br.sendTransform(lista_1)


                # br.sendTransform((p.x, p.y, p.z), (o.x, o.y, o.z, o.w), self.get_clock().now(), link.decode('utf-8'), self.fixed_frame)
            # rate.sleep()
    def publishMarkers(self, pose_keys : list[Pose] , color=[255,0,0]):
        # markers = MarkerArray()
        # for i, det in enumerate(pose_keys):
        #     marker =  Marker()
        #     marker.header.frame_id = "map"
        #     marker.header.stamp = self.get_clock().now().to_msg()

        #     marker.ns = "basic_shapes"
        #     marker.id = i

        #     marker.type = Marker.CUBE

        #     marker.action = Marker.ADD
        #     marker.pose = det

        #     # marker.pose.position.x = 0.0 + float(i)
        #     # marker.pose.position.y = 0.0
        #     marker.pose.position.z = 0.5
        #     # marker.pose.orientation.x = 0.0
        #     # marker.pose.orientation.y = 0.0
        #     # marker.pose.orientation.z = 0.0
        #     # marker.pose.orientation.w = 1.0

        #     marker.scale.x = 0.25
        #     marker.scale.y = 0.25
        #     marker.scale.z = 1.0

        #     marker.color.r = 0.0
        #     marker.color.g = 1.0
        #     marker.color.b = 0.0
        #     marker.color.a = 1.0

        #     # // only if using a MESH_RESOURCE marker type:
        #     # marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

        #     duration = Duration()
        #     duration.sec = 20
        #     duration.nanosec = 0
        #     marker.lifetime = duration
        #     markers.markers.append(marker)

        #     # self.markerPublisher.publish(marker)
        #     # time.sleep(0.25)
        # for i in range(5):
        #     self.markersPublisher.publish(markers)
        
        
        
        
        
        markers = MarkerArray()
        duration = Duration()
        duration.sec = 5
        duration.nanosec = 0
        color = np.asarray(color)/255.0
        for i, det in enumerate(pose_keys):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.action = Marker.ADD
            marker.pose = det
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.4
            marker.ns = "bboxes"
            marker.id = i
            marker.type = Marker.CUBE
            scale = Vector3()
            scale.x = 0.125
            scale.y = 0.125
            scale.z = 0.6
            marker.pose.position.z = marker.pose.position.z + (scale.z/2)
            marker.scale = scale
            marker.lifetime = duration
            markers.markers.append(marker)

            marker_arrow = Marker()
            marker_arrow = marker
            marker_arrow.ns = "arrow"
            marker_arrow.pose.position.x = 
            marker_arrow.pose.position.y
            marker.header.stamp = self.get_clock().now().to_msg()
            marker_arrow.type = Marker.ARROW
            marker_arrow.id = i + len(pose_keys)

            markers.markers.append(marker_arrow)

        for i in range(5):
            self.markersPublisher.publish(markers)

def main(args=None) -> None:
    try:
        rclpy.init(args=args)
        saver = ViewerReaderPlugin()
        rclpy.spin(saver)
        saver.destroy_node()
        rclpy.try_shutdown()
    except ROSInterruptException:
        pass

if __name__ == '__main__':
    main()


#  def publishMarkers(self, pose_keys : Pose , color=[255,0,0]):
#         markers = MarkerArray()
#         duration = Duration()
#         duration.sec = 10
#         color = np.asarray(color)/255.0
#         for i, det in enumerate(pose_keys):
#             # det.header.frame_id = "map"
#             name = det.label

#             # cube marker
#             marker = Marker()
#             marker.header = det.header
#             marker.action = Marker.ADD
#             marker.pose = det.position
#             marker.color.r = color[0]
#             marker.color.g = color[1]
#             marker.color.b = color[2]
#             marker.color.a = 0.4
#             marker.ns = "bboxes"
#             marker.id = i
#             marker.type = Marker.CUBE
#             marker.scale = [3.0, 3.0, 3.0]
#             marker.lifetime = duration
#             markers.markers.append(marker)
#             self.markerPublisher.publish(markers)
#             # # text marker
#             # marker = Marker()
#             # marker.header = det.header
#             # marker.action = Marker.ADD
#             # marker.pose = det.bbox3d.center
#             # marker.color.r = color[0]
#             # marker.color.g = color[1]
#             # marker.color.b = color[2]
#             # marker.color.a = 1.0
#             # marker.id = i
#             # marker.ns = "texts"
#             # marker.type = Marker.TEXT_VIEW_FACING
#             # marker.scale.x = 0.05
#             # marker.scale.y = 0.05
#             # marker.scale.z = 0.05
#             # marker.text = '{} ({:.2f})'.format(name, det.score)
#             # marker.lifetime = duration
#             # markers.markers.append(marker)

#             # for idx, kpt3D in enumerate(det.pose):
#             #     # print("oi")
#             #     if kpt3D.score > 0:
#             #         marker = Marker()
#             #         marker.header = det.header
#             #         marker.type = Marker.SPHERE
#             #         marker.id = idx
#             #         marker.color.r = color[1]
#             #         marker.color.g = color[2]
#             #         marker.color.b = color[0]
#             #         marker.color.a = 1.0
#             #         marker.scale.x = 0.05
#             #         marker.scale.y = 0.05
#             #         marker.scale.z = 0.05
#             #         marker.pose.position.x = kpt3D.x
#             #         marker.pose.position.y = kpt3D.y
#             #         marker.pose.position.z = kpt3D.z
#             #         marker.pose.orientation.x = 0.0
#             #         marker.pose.orientation.y = 0.0
#             #         marker.pose.orientation.z = 0.0
#             #         marker.pose.orientation.w = 1.0
#             #         marker.lifetime = duration
#             #         markers.markers.append(marker)
        
#         self.markerPublisher.publish(markers)