#!/usr/bin/env python3

import rclpy
import os
import yaml

from rclpy.exceptions import ROSInterruptException
from rclpy.wait_for_message import wait_for_message
from geometry_msgs.msg import PoseWithCovarianceStamped
from collections import OrderedDict
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from fbot_world import yaml_io

'''
Save the current pose in the specified topic to a yaml file.
Created by Gabriel Dorneles on 2024-10-06.
Port to ROS2 by Vitor Anello on 2025-05-7.
'''


class PoseWriter (Node):
    '''
    @class PoseWriter
    @brief A ROS 2 node that saves the current pose of a robot to a YAML file.
    This node subscribes to a specified pose topic and allows the user to save the current pose
    by entering a name. The saved poses are stored in a YAML file for later use.
    '''
    
    def __init__(self):
        '''
        @brief Constructor for the PoseWriter node.
        '''

        super().__init__(node_name='pose_writer')
    
        self.poses = {'targets': {}}

        ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../.."))
        self.config_path = os.path.join(ws_dir, "src", "fbot_world", "fbot_world", "config")

        while True:
            self.yaml_file = input("Enter the name of the file to save the poses (e.g., 'pose_inspection.yaml'): ")
            if self.yaml_file.endswith('.yaml'):
                break
            else:
                self.get_logger().warning("Invalid input. The file name must end with '.yaml'. Please try again.")
        self.yaml_path = self.config_path + '/' + self.yaml_file
        self.declare_parameter('~pose_topic', '/amcl_pose')
        self.pose_topic = self.get_parameter('~pose_topic').get_parameter_value().string_value
        self.pose_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   reliability=ReliabilityPolicy.RELIABLE)

    def save_pose(self) -> None:
        '''
        @brief Save the current pose in the specified topic to a yaml file.
        '''

        while rclpy.ok():
            self.get_logger().info("Ready to save a new pose.")
            pose_name = input("Move the robot to the desired pose and enter its name (e.g., 'garbage_1', 'exit'): ")

            if not pose_name:
                self.get_logger().warning("No name provided, skipping pose.")
                continue

            # Read the pose after the operator confirms, so it is the pose at the named spot.
            # AMCL publishes only when the robot moves; transient_local returns its last (latched) pose.
            success, message = wait_for_message(msg_type=PoseWithCovarianceStamped, node=self, topic=self.pose_topic,
                                                qos_profile=self.pose_qos, time_to_wait=2.0)
            if not success:
                self.get_logger().warning(f"No pose received on {self.pose_topic}. Pose '{pose_name}' not saved.")
                continue
            current_pose = message.pose.pose

            self.poses['targets'][pose_name] = OrderedDict([
            ('px', current_pose.position.x),
            ('py', current_pose.position.y),
            ('pz', current_pose.position.z),
            ('ox', current_pose.orientation.x),
            ('oy', current_pose.orientation.y),
            ('oz', current_pose.orientation.z),
            ('ow', current_pose.orientation.w)
        ])

            self.get_logger().info(f"Pose '{pose_name}' saved.")

            while True:
                save_now = input("Do you want to add more poses? (y/n): ").lower()
                if save_now == 'n':
                    if self.write_to_yaml():
                        self.get_logger().info(f"Poses saved to {self.yaml_file}. Shutting down node.")
                    return

                elif save_now == 'y':
                    break
                else:
                    self.get_logger().warning("Invalid input. Please enter 'y' or 'n'.")

    def write_to_yaml(self):
        '''
        @brief Merge the collected poses into poses/targets of the YAML file,
        keeping its rooms and other groups.
        @return False if the existing file could not be parsed (nothing written).
        '''
        if os.path.exists(self.yaml_path):
            self.get_logger().info(f"{self.yaml_file} already exists. The new poses will be appended to the existing data.")
        else:
            self.get_logger().info(f"{self.yaml_file} does not exist. Creating a new file.")

        try:
            existing_data = yaml_io.load_ordered(self.yaml_path)
        except yaml.YAMLError as e:
            self.get_logger().error(f"Could not parse {self.yaml_path}: {e}. File left untouched.")
            return False

        yaml_io.merge_poses(existing_data, 'targets', self.poses['targets'])
        yaml_io.dump_ordered(existing_data, self.yaml_path)
        return True


def main(args=None) -> None:
    try:
        rclpy.init(args=args)
        saver = PoseWriter()
        saver.save_pose()
        rclpy.spin_once(saver)
        saver.destroy_node()
        rclpy.try_shutdown()
    except ROSInterruptException:
        pass

if __name__ == '__main__':
    main()