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
import logging
from io import BytesIO

import numpy as np
from flask import Flask, Response, jsonify, request
from gevent.pywsgi import WSGIServer

app = Flask(__name__)

logger = logging.getLogger("model_infer_server")
logger.setLevel(logging.INFO)
handler = logging.StreamHandler()
handler.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s")
handler.setFormatter(formatter)
logger.addHandler(handler)


class ModelInference:
    def __init__(self):
        # Load your model here
        class FakeModel:
            def infer(self, input_data):
                return {"action": None}

        self.model = FakeModel()
        logger.info("Model loaded successfully.")

    def infer(self, request_data):
        try:
            images = np.stack(
                [
                    np.load(BytesIO(request_data["left_rgb"].read())).astype(
                        np.uint8
                    ),
                    np.load(BytesIO(request_data["middle_rgb"].read())).astype(
                        np.uint8
                    ),
                    np.load(BytesIO(request_data["right_rgb"].read())).astype(
                        np.uint8
                    ),
                ]
            )

            depths = (
                np.stack(
                    [
                        np.load(
                            BytesIO(request_data["left_depth"].read())
                        ).astype(np.float64),
                        np.load(
                            BytesIO(request_data["middle_depth"].read())
                        ).astype(np.float64),
                        np.load(
                            BytesIO(request_data["right_depth"].read())
                        ).astype(np.float64),
                    ]
                )
                / 1000.0
            )

            joint_state = np.load(
                BytesIO(request_data["joint_states"].read())
            ).astype(np.float64)

            intrinsics = np.eye(4)[None].repeat(3, axis=0)
            intrinsics[0, :3] = np.load(
                BytesIO(request_data["left_camera_intrinsic"].read())
            ).astype(np.float64)
            intrinsics[1, :3] = np.load(
                BytesIO(request_data["middle_camera_intrinsic"].read())
            ).astype(np.float64)
            intrinsics[2, :3] = np.load(
                BytesIO(request_data["right_camera_intrinsic"].read())
            ).astype(np.float64)

            input_data = {
                "images": images,
                "depths": depths,
                "joint_state": joint_state,
                "intrinsics": intrinsics,
                "instruction": request_data["instruction"]
                .read()
                .decode("utf-8"),
            }
            actions = self.model.infer(input_data)
            return actions

        except Exception as e:
            logging.exception(f"Error during inference: {e}")
            return None


model_inference = ModelInference()


@app.route("/your_server_name", methods=["POST"])
def model_infer():
    try:
        data = request.files
        required_keys = [
            "left_rgb",
            "middle_rgb",
            "right_rgb",
            "left_depth",
            "middle_depth",
            "right_depth",
            "joint_states",
            "instruction",
            "left_camera_intrinsic",
            "middle_camera_intrinsic",
            "right_camera_intrinsic",
        ]
        for key in required_keys:
            if key not in data:
                return jsonify({"error": f"Missing key: {key}"}), 400
        logger.info("Received request for inference.")
        actions = model_inference.infer(data)
        return Response(
            json.dumps(actions),
        )
    except Exception as e:
        logging.error(f"Error in /sem endpoint: {e}")
        return jsonify({"error": str(e)}), 500


if __name__ == "__main__":
    http_server = WSGIServer(("", 2000), app)
    http_server.serve_forever()
