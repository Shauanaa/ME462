#!/home/peter/venv/bin/python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge, CvBridgeError
import dt_apriltags
from math import atan2, sqrt

class LocalizationNode(Node):
    def __init__(self):
        super().__init__("Localization")
        self.bridge = CvBridge()
        self.detector = dt_apriltags.Detector(searchpath=['apriltags'],
                                        families='tag36h11',
                                        nthreads=4,
                                        quad_decimate=1,
                                        quad_sigma=0.25,
                                        refine_edges=0,
                                        decode_sharpening=0.25,
                                        debug=0)
        self.image_sub = self.create_subscription(Image, "/image_raw", self.image_callback, 1)
        self.pose_pubs = {}

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = np.array(frame, dtype=np.uint8)
            frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            tags = self.detector.detect(frame, estimate_tag_pose=True, camera_params=[995.52534,993.49579,649.25127852,366.80582718], tag_size=0.065)
            for tag in tags:
                pitch, yaw, roll = self.rotationMatrixToEulerAngles(tag.pose_R)
                if tag.tag_id not in self.pose_pubs.keys():
                    self.pose_pubs[tag.tag_id] = self.create_publisher(Pose2D, f"/agent_{tag.tag_id}/pose", 1)
                self.pose_pubs[tag.tag_id].publish(Pose2D(x=tag.pose_t[0][0], y=tag.pose_t[1][0], theta=yaw))
                print(tag.tag_id)
                print(tag.pose_t)
                print(tag.pose_R)
        except CvBridgeError as e:
            print(e)

    def rotationMatrixToEulerAngles(self, R):
        # Extract elements from the rotation matrix
        R11 = R[0][0]
        R12 = R[0][1]
        R13 = R[0][2]
        R21 = R[1][0]
        R22 = R[1][1]
        R23 = R[1][2]
        R31 = R[2][0]
        R32 = R[2][1]
        R33 = R[2][2]
        # Compute pitch, yaw, and roll angles
        pitch = atan2(-R23, R33)
        yaw = atan2(R13, sqrt(1 - R13 * R13))
        roll = atan2(-R12, R11)
        return pitch, yaw, roll

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()