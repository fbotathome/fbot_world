#!/usr/bin/env python3

import rclpy
import rclpy.logging
import rclpy.exceptions
#import rospy
from rclpy.exceptions import ROSInterruptException
import yaml
import os
from geometry_msgs.msg import PoseWithCovarianceStamped
from collections import OrderedDict

'''
Save the current pose in the specified topic to a yaml file.
Created by Gabriel Dorneles on 2024-10-06.
'''


class OrderedDumper(yaml.SafeDumper):
    def represent_ordereddict(self, data):
        return self.represent_dict(data.items())
    
class OrderedLoader(yaml.SafeLoader):
    pass

def construct_ordered_dict(loader, node):
    return OrderedDict(loader.construct_pairs(node))

OrderedLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    construct_ordered_dict
)

class PoseWriter:
    def __init__(self):
        rclpy.init(args=None)
        

        self.root_path = os.environ["HOME"]
        self.poses = {'pose': {'targets': {}}}

        self.config_path = self.root_path + '/fbot_ws/src/butia_world/config'

        while True:
            self.yaml_file = input("Enter the name of the file to save the poses (e.g., 'pose_inspection.yaml'): ")
            if self.yaml_file.endswith('.yaml'):
                break
            else:
                rclpy.logging.get_logger().info("Invalid input. The file name must end with '.yaml'. Please try again.")
        self.yaml_path = self.config_path + '/' + self.yaml_file

        rclpy.Node.declare_parameter('~pose_topic', 'amcl_pose')
        self.pose_topic = rclpy.Node.get_parameter('~pose_topic', 'amcl_pose').value

        #self.pose_sub = rospy.Subscriber(self.pose_topic, PoseWithCovarianceStamped, self.pose_callback)

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def save_pose(self):
        '''
        Save the current pose in the specified topic to a yaml file.
        '''

        #while not rospy.is_shutdown():
        while True:
            pose_name = input("Move the robot to the desired pose and enter its name (e.g., 'garbage_1', 'exit'): ")

            if not pose_name:
                rclpy.logging.get_logger().info("No name provided, skipping pose.")
                continue

            if not hasattr(self, 'current_pose'):
                rclpy.logging.get_logger().info("No pose received from topic yet.")
                continue

            self.poses['pose']['targets'][pose_name] = OrderedDict([
            ('px', self.current_pose.position.x),
            ('py', self.current_pose.position.y),
            ('pz', self.current_pose.position.z),
            ('ox', self.current_pose.orientation.x),
            ('oy', self.current_pose.orientation.y),
            ('oz', self.current_pose.orientation.z),
            ('ow', self.current_pose.orientation.w)
        ])

            rclpy.logging.get_logger().info(f"Pose '{pose_name}' saved.")

            while True:
                save_now = input("Do you want to add more poses? (y/n): ").lower()
                if save_now == 'n':
                    self.write_to_yaml()
                    rclpy.logging.get_logger().info(f"Poses saved to {self.yaml_file}. Shutting down node.")
                    return

                elif save_now == 'y':
                    break
                else:
                    rclpy.logging.get_logger().info("Invalid input. Please enter 'y' or 'n'.")

    def write_to_yaml(self):
        OrderedDumper.add_representer(OrderedDict, OrderedDumper.represent_ordereddict)

        if os.path.exists(self.yaml_path):
            rclpy.logging.get_logger().info(f"{self.yaml_file} already exists. The new poses will be appended to the existing data.")

            with open(self.yaml_path, 'r') as yaml_file:
                try:
                    existing_data = yaml.load(yaml_file, Loader=OrderedLoader) or OrderedDict()
                except yaml.YAMLError as e:
                    rclpy.logging.get_logger().info(f"Error reading {self.yaml_file}: {e}")
                    existing_data = OrderedDict()
        else:
            rclpy.logging.get_logger().info(f"{self.yaml_file} does not exist. Creating a new file.")
            existing_data = OrderedDict()

        if 'pose' not in existing_data:
            existing_data['pose'] = OrderedDict({'targets': OrderedDict()})

        existing_data['pose']['targets'].update(self.poses['pose']['targets'])

        with open(self.yaml_path, 'w') as yaml_file:
             yaml.dump(existing_data, yaml_file, default_flow_style=False, Dumper=OrderedDumper)
             
        return


def main(args=None):
    try:
        saver = PoseWriter()
        saver.save_pose()
    except rclpy.exceptions.ROSInterruptException:
        pass
