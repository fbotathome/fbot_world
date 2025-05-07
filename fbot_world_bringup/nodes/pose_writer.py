#!/usr/bin/env python3

import rclpy
import yaml
import os

from rclpy.exceptions import ROSInterruptException
from geometry_msgs.msg import PoseWithCovarianceStamped
from collections import OrderedDict
from rclpy.node import Node

'''
Save the current pose in the specified topic to a yaml file.
Created by Gabriel Dorneles on 2024-10-06.
Port to ROS2 by Vitor Anello on 2025-05-7.
'''


class OrderedDumper(yaml.SafeDumper):
    '''
    @brief Custom YAML dumper to handle OrderedDict.
    '''
    def represent_ordereddict(self, data):
        return self.represent_dict(data.items())
    
class OrderedLoader(yaml.SafeLoader):
    pass

def construct_ordered_dict(loader, node):
    '''
    @brief Custom YAML loader to handle OrderedDict.
    '''
    return OrderedDict(loader.construct_pairs(node))

OrderedLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    construct_ordered_dict
)

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
    
        self.root_path = os.environ["HOME"]
        self.poses = {'pose': {'ros__parameters': {'targets': {}}}}

        self.config_path = self.root_path + '/fbot_ws/src/fbot_world/fbot_world_bringup/config'

        while True:
            self.yaml_file = input("Enter the name of the file to save the poses (e.g., 'pose_inspection.yaml'): ")
            if self.yaml_file.endswith('.yaml'):
                break
            else:
                self.get_logger().info("Invalid input. The file name must end with '.yaml'. Please try again.")
        self.yaml_path = self.config_path + '/' + self.yaml_file
        self.declare_parameter('~pose_topic', '/amcl_pose')
        self.pose_topic = self.get_parameter('~pose_topic').get_parameter_value().string_value

        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, 10)

    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        '''
        @brief Callback function for the pose subscriber.
        @param msg: The PoseWithCovarianceStamped message received from the topic.
        '''
        self.get_logger().info('Pose callback triggered')
        self.get_logger().info(f"Received msg: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z}")
        self.current_pose = msg.pose.pose

    def save_pose(self) -> None:
        '''
        @brief Save the current pose in the specified topic to a yaml file.
        '''

        while rclpy.ok():
            rclpy.spin_once(self)

            pose_name = input("Move the robot to the desired pose and enter its name (e.g., 'garbage_1', 'exit'): ")

            if not pose_name:
                self.get_logger().info("No name provided, skipping pose.")
                continue

            if not hasattr(self, 'current_pose'):
                self.get_logger().info("No pose received from topic yet.")
                continue

            self.poses['pose']['ros__parameters']['targets'][pose_name] = OrderedDict([
            ('px', self.current_pose.position.x),
            ('py', self.current_pose.position.y),
            ('pz', self.current_pose.position.z),
            ('ox', self.current_pose.orientation.x),
            ('oy', self.current_pose.orientation.y),
            ('oz', self.current_pose.orientation.z),
            ('ow', self.current_pose.orientation.w)
        ])

            self.get_logger().info(f"Pose '{pose_name}' saved.")

            while True:
                save_now = input("Do you want to add more poses? (y/n): ").lower()
                if save_now == 'n':
                    self.write_to_yaml()
                    self.get_logger().info(f"Poses saved to {self.yaml_file}. Shutting down node.")
                    return

                elif save_now == 'y':
                    break
                else:
                    self.get_logger().info("Invalid input. Please enter 'y' or 'n'.")

    def write_to_yaml(self):
        '''
        @brief Write the collected poses to a YAML file.
        '''
        OrderedDumper.add_representer(OrderedDict, OrderedDumper.represent_ordereddict)

        if os.path.exists(self.yaml_path):
            self.get_logger().info(f"{self.yaml_file} already exists. The new poses will be appended to the existing data.")

            with open(self.yaml_path, 'r') as yaml_file:
                try:
                    existing_data = yaml.load(yaml_file, Loader=OrderedLoader) or OrderedDict()
                except yaml.YAMLError as e:
                    self.get_logger().info(f"Error reading {self.yaml_file}: {e}")
                    existing_data = OrderedDict()
        else:
            self.get_logger().info(f"{self.yaml_file} does not exist. Creating a new file.")
            existing_data = OrderedDict()

        if 'pose' not in existing_data:
            existing_data['pose']= OrderedDict({'ros__parameters':{'targets': OrderedDict()}})

        existing_data['pose']['ros__parameters']['targets'].update(self.poses['pose']['ros__parameters']['targets'])
        with open(self.yaml_path, 'w') as yaml_file:
             yaml.dump(existing_data, yaml_file, default_flow_style=False, Dumper=OrderedDumper)
             
        return


def main(args=None) -> None:
    try:
        rclpy.init(args=args)
        saver = PoseWriter()
        saver.save_pose()
        saver.destroy_node()
        rclpy.try_shutdown()
    except ROSInterruptException:
        pass

if __name__ == '__main__':
    main()