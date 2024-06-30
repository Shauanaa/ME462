import numpy as np
import cv2
import dt_apriltags
from math import atan2, sqrt

class Locator:
    def __init__(self, capture_device:int|str=0):
        self._cap = cv2.VideoCapture(capture_device, cv2.CAP_V4L2)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.detector = dt_apriltags.Detector(searchpath=['apriltags'],
                                        families='tag36h11', #tagCircle49h12 , tag36h11
                                        nthreads=4,
                                        quad_decimate=1,
                                        quad_sigma=0.0,
                                        refine_edges=1,
                                        decode_sharpening=0.25,
                                        # max_hamming=1,
                                        debug=0)

    def getPoses(self) -> None|dict:
        """
        Get pose info of robots.
        A dict is returned that maps integer ids to poses in format (x, y, theta)
        None is returned if an error occured
        """
        ret, frame = self._cap.read()
        if not ret:
            return None
        frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(frame, estimate_tag_pose=True, camera_params=[995.52534,993.49579,649.25127852,366.80582718], tag_size=0.05)
        results = {}
        for tag in tags:
            pitch, yaw, roll = self.rotationMatrixToEulerAngles(tag.pose_R)
            print(tag.tag_id)
            print(tag.pose_t)
            print(tag.pose_R)
            results[tag.tag_id] = (tag.pose_t[0][0], tag.pose_t[1][0], yaw)
        return results

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
    node = Locator()

if __name__ == "__main__":
    main()