import cv2
import rclpy
from rclpy.node import Node
import tf2_ros 
import numpy as np
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import aruco_markers as am


class MyCameraCallback(Node, am.CameraViewerCallback):
    def __init__(self):
        Node.__init__(self, "aruco_marker_publisher") 
        am.CameraViewerCallback.__init__(self)

        # Declare and read parameters
        self.declare_parameter(name = "camera_index", value = 0, 
                               descriptor = ParameterDescriptor(type = ParameterType.PARAMETER_INTEGER, description = "This corresponds to /dev/videoN where N is the camera index."))
        self.declare_parameter(name = "marker_length", value = 0.05, 
                               descriptor = ParameterDescriptor(type = ParameterType.PARAMETER_DOUBLE, description = "Size of the ArUCo tag."))
        self.declare_parameter(name = "dict_name", value = "DICT_4X4_50",
                               descriptor = ParameterDescriptor(type = ParameterType.PARAMETER_STRING, description = "The dictionary for the ArUCo marker."))
        self.declare_parameter(name = "marker_id", value = 0, 
                               descriptor = ParameterDescriptor(type = ParameterType.PARAMETER_INTEGER, description = "Identifier of the marker. It has to be a valid id in the specified dictionary."))

        self.camera_index = self.get_parameter("camera_index").get_parameter_value().integer_value
        self.marker_length = self.get_parameter("marker_length").get_parameter_value().double_value
        self.dict_name = self.get_parameter("dict_name").get_parameter_value().string_value
        self.marker_id = self.get_parameter("marker_id").get_parameter_value().integer_value

        self.detector = am.load_detector(self.dict_name)

        # Setup camera
        self.camera = am.cvCamera(self.camera_index)

        self.pose_detector = am.SingleMarkerPoseEstimation(self.camera, self.marker_length)

        # Setup tf broadcaster
        self.tf_br = tf2_ros.TransformBroadcaster(self)


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
                tf.header.stamp = self.get_clock().now().to_msg()
                tf.header.frame_id = "camera"
                tf.child_frame_id = f"tag/{self.dict_name}/{self.marker_id}"
                tf.transform = pose.to_ros_msg()

                # Broadcast transform to ROS
                self.tf_br.sendTransform(tf)

        return img

def main():
    # Initialize ROS
    rclpy.init()

    # Setup callback
    callback = MyCameraCallback()

    # Start camera
    am.CameraViewer("aruco_marker_publisher", callback.camera, callback).spin()

    callback.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()