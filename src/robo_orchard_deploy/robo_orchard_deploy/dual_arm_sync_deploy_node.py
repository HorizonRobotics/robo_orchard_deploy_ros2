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
import time
from io import BytesIO

import cv_bridge
import numpy as np
import rclpy
import requests
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node, ParameterDescriptor
from sensor_msgs.msg import CameraInfo, Image, JointState

from robo_orchard_deploy.config import DeployConfig


class DeployNode(Node):
    def __init__(self):
        super().__init__("dual_arm_sync_deploy_node")
        self._initialize()

        # action publisher
        self.left_arm_publisher = self.create_publisher(
            JointState, self.config.left_arm_control_topic, 1
        )
        self.right_arm_publisher = self.create_publisher(
            JointState, self.config.right_arm_control_topic, 1
        )

        # subscribers
        self.observation_subs = []
        for obs in self.config.observation:
            if obs == "left_arm":
                self.left_arm_state_sub = Subscriber(
                    self, JointState, self.config.left_arm_state_topic
                )
                self.observation_subs.append(self.left_arm_state_sub)
            elif obs == "right_arm":
                self.right_arm_state_sub = Subscriber(
                    self, JointState, self.config.right_arm_state_topic
                )
                self.observation_subs.append(self.right_arm_state_sub)
            elif obs == "left_rgb":
                self.left_rgb_sub = Subscriber(
                    self, Image, self.config.left_rgb_topic
                )
                self.observation_subs.append(self.left_rgb_sub)
            elif obs == "left_depth":
                self.left_depth_sub = Subscriber(
                    self, Image, self.config.left_depth_topic
                )
                self.observation_subs.append(self.left_depth_sub)
            elif obs == "middle_rgb":
                self.middle_rgb_sub = Subscriber(
                    self, Image, self.config.middle_rgb_topic
                )
                self.observation_subs.append(self.middle_rgb_sub)
            elif obs == "middle_depth":
                self.middle_depth_sub = Subscriber(
                    self, Image, self.config.middle_depth_topic
                )
                self.observation_subs.append(self.middle_depth_sub)
            elif obs == "right_rgb":
                self.right_rgb_sub = Subscriber(
                    self, Image, self.config.right_rgb_topic
                )
                self.observation_subs.append(self.right_rgb_sub)
            elif obs == "right_depth":
                self.right_depth_sub = Subscriber(
                    self, Image, self.config.right_depth_topic
                )
                self.observation_subs.append(self.right_depth_sub)
            elif obs == "left_camera_intrinsic":
                self.left_camera_intrinsic_sub = Subscriber(
                    self, CameraInfo, self.config.left_camera_intrinsic_topic
                )
                self.observation_subs.append(self.left_camera_intrinsic_sub)
            elif obs == "middle_camera_intrinsic":
                self.middle_camera_intrinsic_sub = Subscriber(
                    self, CameraInfo, self.config.middle_camera_intrinsic_topic
                )
                self.observation_subs.append(self.middle_camera_intrinsic_sub)
            elif obs == "right_camera_intrinsic":
                self.right_camera_intrinsic_sub = Subscriber(
                    self, CameraInfo, self.config.right_camera_intrinsic_topic
                )
                self.observation_subs.append(self.right_camera_intrinsic_sub)
            else:
                raise ValueError(f"Unknown observation type: {obs}")
        self.sync = ApproximateTimeSynchronizer(
            self.observation_subs, queue_size=1, slop=0.05
        )
        self.sync.registerCallback(self._observe_callback)

        self.cv_bridge = cv_bridge.CvBridge()
        self.get_logger().info(
            f"Initialized DeployNode with model server "
            f"{self.config.server_url}"
        )

    def _initialize(self):
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
                "config file {} does not exists!".format(config_file)
            )
        with open(config_file, "r") as f:
            self.config: DeployConfig = DeployConfig.model_validate_json(
                f.read()
            )

    def _encode_np_array(self, data) -> BytesIO:
        """Encode numpy array into a binary stream for transmission.

        Args:
            data (np.ndarray): The numpy array to encode.

        Returns:
            BytesIO: A binary stream containing the encoded data.
        """
        buff = BytesIO()
        np.save(buff, data)
        buff.seek(0)
        return buff

    def _request_model_inference(self, request_files):
        """Send a request to the model infer server.

        Args:
            None
        Returns:
            model restult
        """
        response = requests.post(
            self.config.server_url, files=request_files, timeout=5
        )
        if response.status_code != requests.codes.ok:
            self.get_logger().error(
                f"Get an error when request {self.config.server_url}: "
                f"{response.content}"
            )
        predict_actions = json.loads(response.content)
        return predict_actions

    def _observe_callback(self, *args):
        self.get_logger().info("Executing new inference and sending actions")
        request_files = {}
        for obs, msg in zip(self.config.observation, args, strict=False):
            if obs in ["left_rgb", "middle_rgb", "right_rgb"]:
                brg_img = self.cv_bridge.imgmsg_to_cv2(
                    msg, desired_encoding="bgr8"
                )
                # rgb_img = cv2.cvtColor(brg_img, cv2.COLOR_BGR2RGB)
                request_files[obs] = (
                    f"{obs}.png",
                    self._encode_np_array(brg_img),
                    "image/png",
                )
            elif obs in ["left_depth", "middle_depth", "right_depth"]:
                depth_img = self.cv_bridge.imgmsg_to_cv2(
                    msg, desired_encoding="passthrough"
                )
                request_files[obs] = (
                    f"{obs}.png",
                    self._encode_np_array(depth_img),
                    "image/png",
                )
            elif obs in [
                "left_camera_intrinsic",
                "middle_camera_intrinsic",
                "right_camera_intrinsic",
            ]:
                request_files[obs] = (
                    f"{obs}.bin",
                    self._encode_np_array(np.array(msg.p).reshape(3, 4)),
                    "application/octet-stream",
                )
            elif obs == "left_arm":
                left_arm_state_msg = msg
                left_joint_state = np.array(left_arm_state_msg.position)
            elif obs == "right_arm":
                right_arm_state_msg = msg
                right_joint_state = np.array(right_arm_state_msg.position)
            else:
                raise ValueError(f"Unknown observation type: {obs}")
        joint_state = np.concatenate(
            (left_joint_state, right_joint_state), axis=0
        )
        request_files["joint_states"] = (
            "joint_state.bin",
            self._encode_np_array(joint_state),
            "application/octet-stream",
        )
        request_files["instruction"] = (
            "instruction.txt",
            BytesIO(self.config.instruction.encode("utf-8")),
            "text/plain",
        )
        predict_actions = self._request_model_inference(request_files)
        if len(predict_actions) == 0:
            self.get_logger().error("No actions received from server.")
            return
        for idx, action in enumerate(predict_actions):
            left_arm_target = action[0:7]
            right_arm_target = action[7:14]
            self._send_action(left_arm_target, right_arm_target)
            time.sleep(1 / self.config.control_frequency)
            self.get_logger().info(f"Action {idx} sent.")
        time.sleep(0.5)

    def _send_left_arm_action(self, joint_position):
        goal_msg = JointState()
        goal_msg.name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]
        goal_msg.position = joint_position
        goal_msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 80.0]
        goal_msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]
        self.left_arm_publisher.publish(goal_msg)

    def _send_right_arm_action(self, joint_position):
        goal_msg = JointState()
        goal_msg.name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]
        goal_msg.position = joint_position
        goal_msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 80.0]
        goal_msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]
        self.right_arm_publisher.publish(goal_msg)

    def _send_action(self, left_arm_target, right_arm_target):
        self._send_left_arm_action(left_arm_target)
        self._send_right_arm_action(right_arm_target)


def main(args=None):
    rclpy.init(args=args)
    excutor = rclpy.executors.MultiThreadedExecutor()
    excutor.add_node(DeployNode())

    try:
        excutor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        excutor.shutdown()
