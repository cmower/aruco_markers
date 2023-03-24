import cv2
import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
import aruco_markers as am


class MyCameraCallback(am.CameraViewerCallback):
    def __init__(self, camera, marker_length, dict_name, marker_id):
        super().__init__()
        self.dict_name = dict_name
        self.marker_id = marker_id
        self.detector = am.load_detector(dict_name)
        self.pose_detector = am.SingleMarkerPoseEstimation(camera, marker_length)

        # Setup tf broadcaster
        self.tf_br = tf2_ros.TransformBroadcaster()

    def call(self, img):
        corners, ids, _ = self.detector.detectMarkers(img)
        for i, c in enumerate(corners):
            if ids[i] == self.marker_id:
                # Draw detected marker
                img = cv2.aruco.drawDetectedMarkers(img, (c,), np.array([ids[i]]))

                # Estimate marker pose
                pose = self.pose_detector.estimate_marker_pose(c)

                # Convert to transform stamped message
                tf = TransformStamped()
                tf.header.stamp = rospy.Time.now()
                tf.header.frame_id = "camera"
                tf.child_frame_id = f"tag/{self.dict_name}/{self.marker_id}"
                tf.transform = pose.to_ros_msg()

                # Broadcast transform to ROS
                self.tf_br.sendTransform(tf)

        return img


def main():
    # Initialize ROS
    rospy.init_node("aruco_marker_publisher")

    # Setup camera
    camera_index = int(rospy.get_param("~camera_index", 0))
    camera = am.cvCamera(camera_index)

    # Setup callback
    marker_length = float(rospy.get_param("~marker_length", 0.05))
    dict_name = str(rospy.get_param("~dict_name", "DICT_4X4_50"))
    marker_id = int(rospy.get_param("~marker_id", 0))
    callback = MyCameraCallback(camera, marker_length, dict_name, marker_id)

    # Start camera
    am.CameraViewer("aruco_marker_publisher", camera, callback).spin()


if __name__ == "__main__":
    main()
