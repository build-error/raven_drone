#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from ibvs_interfaces.msg import IBVSVelocity

from ibvs_mpc.aruco_detector import ArucoDetector


IMAGE_TOPIC = "/aruco/image"
CAMERA_INFO_TOPIC = "/aruco/camera_info"

MARKER_SIZE_M = 0.5

Z_DES = 0.5
LAMBDA = 0.4


class IBVSController(Node):

    def __init__(self):

        super().__init__("ibvs_controller")

        self.bridge = CvBridge()

        self.detector = ArucoDetector()

        self.camera_matrix = None
        self.dist_coeffs = None

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.image_sub = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            qos
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        self.velocity_pub = self.create_publisher(
            IBVSVelocity,
            "/ibvs/velocity_reference",
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            "/ibvs/image",
            10
        )

        #
        # Fixed camera extrinsics 
        #
        self.R_body_optical = np.array([
            [0.0, -1.0, 0.0],
            [1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0]
        ])

        self.p_body_camera = np.array([
            0.0,
            0.0,
            -0.10
        ])

        self.get_logger().info(
            "IBVS Controller Started"
        )

    def camera_info_callback(self, msg):

        self.camera_matrix = np.array(
            msg.k,
            dtype=np.float64
        ).reshape(3, 3)

        self.dist_coeffs = np.array(
            msg.d,
            dtype=np.float64
        )

    def image_callback(self, msg):

        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        detections = self.detector.detect(frame)

        if not detections:

            cmd = IBVSVelocity()

            cmd.vx = 0.0
            cmd.vy = 0.0
            cmd.vz = 0.0

            cmd.wx = 0.0
            cmd.wy = 0.0
            cmd.wz = 0.0

            cmd.target_visible = False

            self.velocity_pub.publish(cmd)

            return

        target = detections[0]

        image_points = target.corners.astype(
            np.float32
        )

        half = MARKER_SIZE_M / 2.0

        object_points = np.array(
            [
                [-half, -half, 0.0],
                [ half, -half, 0.0],
                [ half,  half, 0.0],
                [-half,  half, 0.0],
            ],
            dtype=np.float32
        )

        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs
        )

        if not success:
            return

        Z = float(tvec[2])

        rvec_des = np.zeros(
            (3, 1),
            dtype=np.float64
        )

        tvec_des = np.array(
            [
                [0.0],
                [0.0],
                [Z_DES]
            ],
            dtype=np.float64
        )

        desired_points, _ = cv2.projectPoints(
            object_points,
            rvec_des,
            tvec_des,
            self.camera_matrix,
            self.dist_coeffs
        )

        desired_points = desired_points.reshape(
            -1,
            2
        )

        # Draw desired points P1* P2* P3* P4*

        for i, p in enumerate(desired_points):

            x = int(p[0])
            y = int(p[1])

            cv2.circle(
                frame,
                (x, y),
                10,
                (0, 255, 255),   # Yellow
                2
            )

            cv2.putText(
                frame,
                f"P{i+1}*",
                (x + 10, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )

        # Draw error vectors

        for current, desired in zip(
            image_points,
            desired_points
        ):

            cv2.line(
                frame,
                (
                    int(current[0]),
                    int(current[1])
                ),
                (
                    int(desired[0]),
                    int(desired[1])
                ),
                (0, 0, 255),     # Red
                2
            )        

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]

        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        s = []
        s_star = []

        L_rows = []

        for p, p_star in zip(
            image_points,
            desired_points
        ):

            x = float(p[0])
            y = float(p[1])

            x_star = float(p_star[0])
            y_star = float(p_star[1])

            u = (x - cx) / fx
            v = (y - cy) / fy

            u_star = (x_star - cx) / fx
            v_star = (y_star - cy) / fy

            s.extend(
                [u, v]
            )

            s_star.extend(
                [u_star, v_star]
            )

            Li = np.array(
                [
                    [
                        -1.0 / Z,
                        0.0,
                        u / Z,
                        u * v,
                        -(1.0 + u * u),
                        v
                    ],
                    [
                        0.0,
                        -1.0 / Z,
                        v / Z,
                        1.0 + v * v,
                        -u * v,
                        -u
                    ]
                ]
            )

            L_rows.append(
                Li
            )

        s = np.array(
            s,
            dtype=np.float64
        ).reshape(-1, 1)

        s_star = np.array(
            s_star,
            dtype=np.float64
        ).reshape(-1, 1)

        e = s - s_star

        print("error norm =", np.linalg.norm(e))

        L = np.vstack(
            L_rows
        )

        v_cam = (
            -LAMBDA
            * np.linalg.pinv(L)
            @ e
        )

        vx_cam = float(v_cam[0])
        vy_cam = float(v_cam[1])
        vz_cam = float(v_cam[2])

        wx_cam = float(v_cam[3])
        wy_cam = float(v_cam[4])
        wz_cam = float(v_cam[5])

        # Camera optical velocity

        v_cam_vec = np.array([
            vx_cam,
            vy_cam,
            vz_cam
        ])

        w_cam_vec = np.array([
            wx_cam,
            wy_cam,
            wz_cam
        ])

        #
        # Optical -> Body 
        #

        #
        # Transform angular velocity
        #

        w_body = self.R_body_optical @ w_cam_vec

        #
        # Transform linear velocity
        #

        v_body = (
            self.R_body_optical @ v_cam_vec
            +
            np.cross(
                self.p_body_camera,
                w_body
            )
        )

        print("\n----------------")
        print("v_cam =", v_cam_vec)
        print("v_body =", v_body)
        print("----------------")

        cmd = IBVSVelocity()

        cmd.vx = float(v_body[0])
        cmd.vy = float(v_body[1])
        cmd.vz = float(v_body[2])

        cmd.wx = float(w_body[0])
        cmd.wy = float(w_body[1])
        cmd.wz = float(w_body[2])

        cmd.target_visible = True

        self.velocity_pub.publish(
            cmd
        )

        image_msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding="bgr8"
        )

        image_msg.header = msg.header

        self.image_pub.publish(
            image_msg
        )
        
        self.get_logger().info(
            f"Z={Z:.2f} "
            f"vx={v_body[0]:.3f} "
            f"vy={v_body[1]:.3f} "
            f"vz={v_body[2]:.3f} "
            f"wx={w_body[0]:.3f} "
            f"wy={w_body[1]:.3f} "
            f"wz={w_body[2]:.3f}"
        )


def main():

    rclpy.init()

    node = IBVSController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()