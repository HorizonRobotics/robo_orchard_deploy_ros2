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


from typing import List

from pydantic import BaseModel, Field


class DeployConfig(BaseModel):
    observation: List[str] = Field(
        default=[
            "left_rgb",
            "left_depth",
            "middle_rgb",
            "middle_depth",
            "right_rgb",
            "right_depth",
            "left_arm",
            "right_arm",
        ],
        description="List of observation.",
    )
    left_arm_control_topic: str = Field(
        default="/master/joint_left", description="Topic for left arm control."
    )
    right_arm_control_topic: str = Field(
        default="/master/joint_right",
        description="Topic for right arm control.",
    )
    left_arm_state_topic: str = Field(
        default="/puppet/joint_left", description="Topic for left arm state."
    )
    right_arm_state_topic: str = Field(
        default="/puppet/joint_right", description="Topic for right arm state."
    )
    left_rgb_topic: str = Field(
        default="/agilex/left_camera/color/image_raw",
        description="Left RGB camera topic.",
    )
    left_depth_topic: str = Field(
        default="/agilex/left_camera/aligned_depth_to_color/image_raw",
        description="Left depth camera topic.",
    )
    left_camera_intrinsic_topic: str = Field(
        default="/agilex/left_camera/color/camera_info",
        description="Left camera intrinsics topic.",
    )
    middle_rgb_topic: str = Field(
        default="/agilex/middle_camera/color/image_raw",
        description="Middle RGB camera topic.",
    )
    middle_depth_topic: str = Field(
        default="/agilex/middle_camera/aligned_depth_to_color/image_raw",
        description="Middle depth camera topic.",
    )
    middle_camera_intrinsic_topic: str = Field(
        default="/agilex/middle_camera/color/camera_info",
        description="Middle camera intrinsics topic.",
    )
    right_rgb_topic: str = Field(
        default="/agilex/right_camera/color/image_raw",
        description="Right RGB camera topic.",
    )
    right_depth_topic: str = Field(
        default="/agilex/right_camera/aligned_depth_to_color/image_raw",
        description="Right depth camera topic.",
    )
    right_camera_intrinsic_topic: str = Field(
        default="/agilex/right_camera/color/camera_info",
        description="Right camera intrinsics topic.",
    )
    instruction: str = Field(
        default="Please follow the instructions to perform the task.",
        description="Instruction for the model.",
    )
    server_url: str = Field(
        default="http://localhost:5000/openpi", description="Server URL."
    )
    control_frequency: float = Field(
        default=25.0, description="Control frequency in Hz."
    )
    infer_frequency: float = Field(
        default=1.0, description="Inference frequency in Hz."
    )
