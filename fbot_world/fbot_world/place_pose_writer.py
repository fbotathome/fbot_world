#!/usr/bin/env python3

import os
import re
import threading

import rclpy

from collections import OrderedDict
from geometry_msgs.msg import Pose
from rclpy.exceptions import ROSInterruptException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros

from interbotix_xs_msgs.srv import TorqueEnable

from fbot_world import yaml_io

'''
Save the current end-effector pose (read from TF, expressed in the map frame) to a
yaml file, so fixed "place" locations such as shelves can be authored by moving the
arm to the desired drop pose and pressing Enter.

Companion to pose_writer.py (which saves the robot base pose from /amcl_pose). The
saved poses are served, unchanged, by the existing PosePlugin (/fbot_world/get_pose)
since it iterates arbitrary top-level groups; this node writes under the
'place_poses' group by default.

Created by Gabriel Dorneles on 2026-06-01.
'''


class PlacePoseWriter(Node):
    '''
    @class PlacePoseWriter
    @brief A ROS 2 node that saves the current end-effector pose (from TF, in the
    map frame) to a YAML file. The user moves the arm to the desired place pose and
    enters a name; poses are stored for later use as geometric place targets.
    '''

    def __init__(self):
        '''
        @brief Constructor for the PlacePoseWriter node.
        '''
        super().__init__(node_name='place_pose_writer')

        self.declare_parameter('reference_frame', 'map')
        self.declare_parameter('ee_frame', 'wx200/ee_gripper_link')
        self.declare_parameter('group_set', 'place_poses')
        # Torque is disabled on this joint group so the arm can be hand-guided to the
        # desired place pose, then re-enabled before exiting.
        self.declare_parameter('robot_name', 'wx200')
        self.declare_parameter('torque_group', 'arm')

        self.reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value
        self.ee_frame = self.get_parameter('ee_frame').get_parameter_value().string_value
        self.group_set = self.get_parameter('group_set').get_parameter_value().string_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.torque_group = self.get_parameter('torque_group').get_parameter_value().string_value

        self.poses = {self.group_set: {}}

        ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../.."))
        self.config_path = os.path.join(ws_dir, "src", "fbot_world", "fbot_world_bringup", "config")

        while True:
            self.yaml_file = input("Enter the name of the file to save the poses (e.g., 'place_poses.yaml'): ")
            if self.yaml_file.endswith('.yaml'):
                break
            else:
                self.get_logger().warning("Invalid input. The file name must end with '.yaml'. Please try again.")
        self.yaml_path = os.path.join(self.config_path, self.yaml_file)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.torque_service = f'/{self.robot_name}/torque_enable'
        self.torque_client = self.create_client(TorqueEnable, self.torque_service)

        # Spin in a background thread so the TF listener keeps filling the buffer and
        # service responses are processed while the main thread blocks on input().
        self.executor_thread = threading.Thread(target=self._spin, daemon=True)
        self.executor_thread.start()

    def _spin(self) -> None:
        '''
        @brief Background spin loop to keep the TF buffer up to date.
        '''
        try:
            rclpy.spin(self)
        except (ROSInterruptException, rclpy.executors.ExternalShutdownException):
            pass

    def lookupCurrentPose(self) -> Pose:
        '''
        @brief Look up the current end-effector pose in the reference frame via TF.
        @return geometry_msgs/Pose of ee_frame expressed in reference_frame, or None
        if the transform is unavailable.
        '''
        try:
            tf = self.tf_buffer.lookup_transform(
                self.reference_frame, self.ee_frame, Time(seconds=0),
                timeout=Duration(seconds=2.0)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException) as exc:
            self.get_logger().warning(
                f"Could not transform {self.ee_frame} -> {self.reference_frame}: {exc}"
            )
            return None

        pose = Pose()
        pose.position.x = tf.transform.translation.x
        pose.position.y = tf.transform.translation.y
        pose.position.z = tf.transform.translation.z
        pose.orientation = tf.transform.rotation
        return pose

    def setTorque(self, enable: bool) -> bool:
        '''
        @brief Enable or disable torque on the configured joint group via the
        Interbotix TorqueEnable service, so the arm can be hand-guided.
        @param enable: True to torque on, False to torque off.
        @return True if the service call succeeded, False otherwise.
        '''
        action = "Enabling" if enable else "Disabling"
        if not self.torque_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f"{action} torque failed: service '{self.torque_service}' unavailable."
            )
            return False

        request = TorqueEnable.Request()
        request.cmd_type = 'group'
        request.name = self.torque_group
        request.enable = enable

        future = self.torque_client.call_async(request)

        # The node spins in a background thread, so just wait for the future here.
        start = self.get_clock().now()
        while not future.done():
            if (self.get_clock().now() - start) > Duration(seconds=5.0):
                self.get_logger().error(f"{action} torque timed out.")
                return False

        if future.exception() is not None:
            self.get_logger().error(f"{action} torque raised: {future.exception()}")
            return False

        self.get_logger().info(
            f"{action.replace('ing', 'ed')} torque on group '{self.torque_group}'."
        )
        return True

    def save_pose(self) -> None:
        '''
        @brief Interactively capture named end-effector place poses and persist them.
        Torque is disabled while capturing (so the arm can be hand-guided) and always
        re-enabled before returning.
        '''
        # The wx200 has no gravity compensation: it will go limp and drop when
        # torque is cut, so make the user support it first.
        input(
            "WARNING: torque is about to be DISABLED so you can hand-guide the arm. "
            "Hold the arm to stop it from dropping, then press Enter..."
        )
        self.setTorque(False)
        try:
            self._capture_loop()
        finally:
            self.setTorque(True)

    def _capture_loop(self) -> None:
        '''
        @brief The interactive capture loop, run with torque disabled.
        '''
        while rclpy.ok():
            pose_name = input(
                "Move the arm to the desired place pose and enter its name (e.g., 'shelf_1', 'bin'): "
            ).strip()

            if not pose_name:
                self.get_logger().warning("No name provided, skipping pose.")
                continue

            if not re.fullmatch(r"[A-Za-z0-9_.-]+", pose_name):
                self.get_logger().warning(
                    "Invalid pose name. Use only letters, numbers, '-', '_' or '.'. Pose not saved."
                )
                continue

            current_pose = self.lookupCurrentPose()
            if current_pose is None:
                self.get_logger().warning("No transform available yet. Pose not saved.")
                continue

            self.poses[self.group_set][pose_name] = OrderedDict([
                ('px', current_pose.position.x),
                ('py', current_pose.position.y),
                ('pz', current_pose.position.z),
                ('ox', current_pose.orientation.x),
                ('oy', current_pose.orientation.y),
                ('oz', current_pose.orientation.z),
                ('ow', current_pose.orientation.w),
            ])

            self.get_logger().info(
                f"Pose '{pose_name}' saved "
                f"({current_pose.position.x:.3f}, {current_pose.position.y:.3f}, "
                f"{current_pose.position.z:.3f}) in '{self.reference_frame}'."
            )

            while True:
                save_now = input("Do you want to add more poses? (y/n): ").lower()
                if save_now == 'n':
                    self.write_to_yaml()
                    self.get_logger().info(f"Poses saved to {self.yaml_file}. Shutting down node.")
                    return
                elif save_now == 'y':
                    break
                else:
                    self.get_logger().warning("Invalid input. Please enter 'y' or 'n'.")

    def write_to_yaml(self):
        '''
        @brief Write the collected poses to a YAML file, appending to the group if it
        already exists.
        '''
        if os.path.exists(self.yaml_path):
            self.get_logger().info(f"{self.yaml_file} already exists. The new poses will be appended to the existing data.")
        else:
            self.get_logger().info(f"{self.yaml_file} does not exist. Creating a new file.")

        existing_data = yaml_io.load_ordered(self.yaml_path)

        if self.group_set not in existing_data:
            existing_data[self.group_set] = OrderedDict()

        existing_data[self.group_set].update(self.poses[self.group_set])
        yaml_io.dump_ordered(existing_data, self.yaml_path)

        return


def main(args=None) -> None:
    saver = None
    try:
        rclpy.init(args=args)
        saver = PlacePoseWriter()
        saver.save_pose()
    except (ROSInterruptException, KeyboardInterrupt):
        pass
    finally:
        if saver is not None:
            saver.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
