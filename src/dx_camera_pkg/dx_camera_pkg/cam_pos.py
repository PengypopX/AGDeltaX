#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
import tf2_ros
import numpy as np

class CharucoPoseNode(Node):
    def __init__(self):
        super().__init__('charuco_pose_node')
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Load intrinsics (replace with your calibration values)
        self.camera_matrix = np.array([[fx, 0, cx],
                                       [0, fy, cy],
                                       [0,  0,  1]], dtype=np.float32)
        self.dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float32)

        # Define your ChArUco board spec (match your print!)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.board = aruco.CharucoBoard_create(
            squaresX=5, squaresY=7,
            squareLength=0.04, markerLength=0.02,
            dictionary=aruco_dict
        )

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = aruco.detectMarkers(gray, self.board.dictionary)
        if ids is None:
            return

        # Refine with ChArUco corners
        retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            corners, ids, gray, self.board)

        if retval > 0:
            success, rvec, tvec = aruco.estimatePoseCharucoBoard(
                charuco_corners, charuco_ids, self.board,
                self.camera_matrix, self.dist_coeffs)

            if success:
                # Convert rvec/tvec to ROS2 transform
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = 'camera_link'
                transform.child_frame_id = 'charuco_board'
                transform.transform.translation.x = float(tvec[0][0])
                transform.transform.translation.y = float(tvec[0][1])
                transform.transform.translation.z = float(tvec[0][2])

                # Convert rotation vector to quaternion
                R, _ = cv2.Rodrigues(rvec)
                qw = np.sqrt(1.0 + R[0,0] + R[1,1] + R[2,2]) / 2.0
                qx = (R[2,1] - R[1,2]) / (4.0*qw)
                qy = (R[0,2] - R[2,0]) / (4.0*qw)
                qz = (R[1,0] - R[0,1]) / (4.0*qw)

                transform.transform.rotation.x = qx
                transform.transform.rotation.y = qy
                transform.transform.rotation.z = qz
                transform.transform.rotation.w = qw

                self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = CharucoPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
