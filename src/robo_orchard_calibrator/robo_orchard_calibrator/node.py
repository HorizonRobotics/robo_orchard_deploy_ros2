# Project RoboOrchard
#
# Copyright (c) 2024-2025 Horizon Robotics. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied. See the License for the specific language governing
# permissions and limitations under the License.

import json
import os
import threading

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node, ParameterDescriptor
from sshkeyboard import listen_keyboard, stop_listening
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from robo_orchard_calibrator.config import (
    HandEyeCalibrationConfig,
)
from robo_orchard_calibrator.utils import (
    run_eye_in_hand_calibration,
    run_eye_to_hand_calibration,
)


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("handeye_calibration_node")
        self._tf_broadcaster = StaticTransformBroadcaster(self)

        self.declare_parameter(
            "config_file",
            "",
            descriptor=ParameterDescriptor(description="Config path"),
        )
        config_file: str = (
            self.get_parameter("config_file")
            .get_parameter_value()
            .string_value
        )
        if not os.path.exists(config_file):
            raise FileNotFoundError(
                f"config file does not exists!({config_file})"
            )
        with open(config_file, "r") as f:
            self.config: HandEyeCalibrationConfig = (
                HandEyeCalibrationConfig.model_validate_json(f.read())
            )  # noqa: E501

        # callback group
        self._aruco_pose_callback_group = ReentrantCallbackGroup()
        self._end_effector_callback_group = ReentrantCallbackGroup()

        # aruco marker pose subscriber
        self.create_subscription(
            msg_type=PoseStamped,
            topic=self.config.aruco_marker_pose_topic_name,
            callback=self._aruco_pose_callback,
            qos_profile=10,
            callback_group=self._aruco_pose_callback_group,
        )

        # end effector pose subscriber
        self.create_subscription(
            msg_type=PoseStamped,
            topic=self.config.end_effector_pose_topic_name,
            callback=self._end_effector_pose_callback,
            qos_profile=10,
            callback_group=self._end_effector_callback_group,
        )

        self.cur_aruco_pose = None
        self.cur_ee_pose = None
        self.aruco_poses_list = []
        self.ee_poses_list = []
        self.record_data_cnt = 0

        self._listener_thread = threading.Thread(
            target=self._keyboard_listener_loop, daemon=True
        )
        self._listener_thread.start()

        self.get_logger().info(
            "Calibration node initialized."
            "Press 'r' to record data, 'q' to quit and save result."
        )

    def _aruco_pose_callback(self, msg: PoseStamped):
        self.cur_aruco_pose = msg

    def _end_effector_pose_callback(self, msg: PoseStamped):
        self.cur_ee_pose = msg

    def _keyboard_listener_loop(self):
        listen_keyboard(
            on_press=self._on_key_press, delay_second_char=0.05, lower=False
        )

    def _on_key_press(self, key):
        if key == "r":
            if (
                self.cur_aruco_pose is not None
                and self.cur_ee_pose is not None
            ):
                aruco_pose_timestamp = self.cur_aruco_pose.header.stamp
                ee_pose_timestamp = self.cur_ee_pose.header.stamp
                aruco_pose_time = (
                    aruco_pose_timestamp.sec
                    + aruco_pose_timestamp.nanosec / 1e9
                )
                ee_pose_time = (
                    ee_pose_timestamp.sec + ee_pose_timestamp.nanosec / 1e9
                )
                if abs(aruco_pose_time - ee_pose_time) > 0.1:
                    self.get_logger().warning(
                        "Aruco pose and end effector pose timestamps differ "
                        "by more than 0.1 seconds, "
                        "check aruco marker whether in camera field."
                    )
                    return

                self.aruco_poses_list.append(self.cur_aruco_pose)
                self.ee_poses_list.append(self.cur_ee_pose)
                self.record_data_cnt += 1
                self.cur_aruco_pose = None
                self.cur_ee_pose = None
                self.get_logger().info(
                    f"Recorded {self.record_data_cnt} data."
                )
            else:
                self.get_logger().warning(
                    "Current aruco pose or end effector pose is None, "
                    "please check the topics."
                )
        elif key == "q":
            if len(self.aruco_poses_list) < 3 or len(self.ee_poses_list) < 3:
                self.get_logger().warning(
                    "Not enough data, please record at least 3 poses."
                )
                return
            self.get_logger().info(
                f"Recording finished, result will be saved to"
                f" {self.config.result_file}"
            )
            parent_frame = None
            child_frame = None
            if self.config.mode == "eye_in_hand":
                res_position, res_orientation = run_eye_in_hand_calibration(  # noqa: E501
                    self.aruco_poses_list,
                    self.ee_poses_list,
                    self.record_data_cnt,
                )
                parent_frame = self.config.robot_name + "_end_effector"
                child_frame = self.config.camera_name
            elif self.config.mode == "eye_to_hand":
                res_position, res_orientation = run_eye_to_hand_calibration(  # noqa: E501
                    self.aruco_poses_list,
                    self.ee_poses_list,
                    self.record_data_cnt,
                )
                parent_frame = self.config.robot_name + "_base"
                child_frame = self.config.camera_name
            else:
                self.get_logger().error(
                    "Invalid mode, please check the config file."
                )
            with open(self.config.result_file, "w") as f:
                json.dump(
                    {
                        "parent_frame": parent_frame,
                        "child_frame": child_frame,
                        "result": {
                            "position": res_position,
                            "orientation": res_orientation,
                        },
                    },
                    f,
                    indent=4,
                )
            # publish to ros2 tf_tree
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            t.transform.translation.x = res_position[0]
            t.transform.translation.y = res_position[1]
            t.transform.translation.z = res_position[2]
            t.transform.rotation.x = res_orientation[0]
            t.transform.rotation.y = res_orientation[1]
            t.transform.rotation.z = res_orientation[2]
            t.transform.rotation.w = res_orientation[3]
            self._tf_broadcaster.sendTransform(t)
            self.get_logger().info("Result saved and broadcasted to tf tree.")

    def destroy_node(self):
        stop_listening()
        if (
            hasattr(self, "_listener_thread")
            and self._listener_thread.is_alive()
        ):
            self._listener_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init()
    try:
        node = CalibrationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Ctrl-C pressed, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
