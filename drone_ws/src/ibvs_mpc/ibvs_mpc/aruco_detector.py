#!/usr/bin/env python3

from dataclasses import dataclass

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo


CAMERA_TOPIC = "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image"
CAMERA_INFO_TOPIC = "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info"


@dataclass
class Detection:
    marker_id: int
    corners: np.ndarray
    center_x: float
    center_y: float
    area: float


class ArucoDetector:

    def __init__(self, dictionary=cv2.aruco.DICT_4X4_50):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict)

    def detect(self, frame) -> list[Detection]:

        corners, ids, _ = self.detector.detectMarkers(frame)

        detections = []

        if ids is None:
            return detections

        for marker_corners, marker_id in zip(corners, ids.flatten()):

            pts = marker_corners[0]

            detections.append(
                Detection(
                    marker_id=int(marker_id),
                    corners=pts,
                    center_x=float(pts[:, 0].mean()),
                    center_y=float(pts[:, 1].mean()),
                    area=float(
                        cv2.contourArea(
                            pts.astype(np.float32)
                        )
                    )
                )
            )

        return detections

    def draw(self, frame, detections: list[Detection]):

        output = frame.copy()

        for detection in detections:

            pts = detection.corners.astype(np.int32)

            cv2.polylines(
                output,
                [pts],
                True,
                (0, 255, 0),
                2
            )

            for i, corner in enumerate(pts):

                x = int(corner[0])
                y = int(corner[1])

                cv2.circle(
                    output,
                    (x, y),
                    6,
                    (255, 0, 0),
                    -1
                )

                cv2.putText(
                    output,
                    f"P{i+1}",
                    (x + 5, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    2
                )

            cx = int(detection.center_x)
            cy = int(detection.center_y)

            cv2.circle(
                output,
                (cx, cy),
                5,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                output,
                f"ID {detection.marker_id}",
                (cx + 10, cy),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

        return output


class ArucoDetectorNode(Node):

    def __init__(self):

        super().__init__("aruco_detector")

        self.bridge = CvBridge()
        self.detector = ArucoDetector()

        self.image_sub = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        self.image_pub = self.create_publisher(
            Image,
            "/aruco/image",
            10
        )

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            "/aruco/camera_info",
            10
        )

        self.get_logger().info(
            "ArUco Detector Started"
        )

    def camera_info_callback(self, msg):

        self.camera_info_pub.publish(msg)

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        detections = self.detector.detect(frame)

        if detections:

            target = detections[0]

            self.get_logger().info(
                f"ID={target.marker_id} "
                f"Center=({target.center_x:.1f}, "
                f"{target.center_y:.1f}) "
                f"Area={target.area:.1f}"
            )

        display = self.detector.draw(
            frame,
            detections
        )

        image_msg = self.bridge.cv2_to_imgmsg(
            display,
            encoding="bgr8"
        )

        image_msg.header = msg.header

        self.image_pub.publish(image_msg)


def main():

    rclpy.init()

    node = ArucoDetectorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()