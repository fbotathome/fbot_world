#!/usr/bin/env python3

import rclpy
import tf2_ros as tf
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped

from rclpy.exceptions import ROSInterruptException
from scripts.world_plugin import WorldPlugin

class ViewerReaderPlugin(WorldPlugin): 
    def __init__(self, nodeName='pose_viewer'):
        super().__init__(nodeName=nodeName)
        self.nodeName= nodeName
        lista_1 = []
        br = tf.TransformBroadcaster(self)
        rate = self.create_rate(30)
        while rclpy.ok():
            rclpy.spin_once(self)
            pose_keys = self.r.keys(b'*/pose')
            pose = Pose()
            for key in pose_keys:
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

                p = pose.position
                o = pose.orientation

                transform_stamped = TransformStamped()
                transform_stamped.header.stamp = self.get_clock().now().to_msg()
                transform_stamped.header.frame_id = self.fixed_frame
                transform_stamped.child_frame_id = self.nodeName
                transform_stamped.transform.translation.x = p.x
                transform_stamped.transform.translation.y = p.y
                transform_stamped.transform.translation.z = p.z
                transform_stamped.transform.rotation.x = o.x
                transform_stamped.transform.rotation.y = o.y
                transform_stamped.transform.rotation.z = o.z
                transform_stamped.transform.rotation.w = o.w
                self.get_logger().info(f"Broadcasting {key}: {transform_stamped}")
                lista_1.append(transform_stamped)

            br.sendTransform(lista_1)


                # br.sendTransform((p.x, p.y, p.z), (o.x, o.y, o.z, o.w), self.get_clock().now(), link.decode('utf-8'), self.fixed_frame)
            rate.sleep()

def main(args=None) -> None:
    try:
        rclpy.init(args=args)
        saver = ViewerReaderPlugin()
        saver.destroy_node()
        rclpy.try_shutdown()
    except ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
