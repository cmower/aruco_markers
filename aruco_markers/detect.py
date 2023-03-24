import pickle
from typing import List, Tuple, Union

## True when ROS is available on the host system.
ROS_AVAILABLE = True

try:
    from geometry_msgs.msg import Transform
except ImportError:
    Transform = None
    ROS_AVAILABLE = False

import attrs

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .camera import Camera, CameraViewer, CameraViewerCallback
from .marker import Marker, ARUCO_DICT
from .utils import join_path, isdir, isfile


@attrs.define
class MarkerPose:

    """! Helper class that contains the marker pose information and several methods for converting the data."""

    ## Rotation part of the pose. This is assumed to be passed as a rotation vector. Note, the input is converted to a Rotation class from the scipy spatial module. This allows you to convert the rotation to several different formats (e.g. unit-quaternions, matrix, etc).
    rotation: Rotation = attrs.field(
        converter=lambda r: Rotation.from_rotvec(r.flatten()),
        default=Rotation.identity(),
    )

    ## Translation part of the pose.
    translation: np.ndarray = attrs.field(
        converter=lambda t: t.flatten(), default=np.zeros(3)
    )

    def transform(self) -> np.ndarray:
        """! Homogenous transformation matrix representing the pose."""
        T = np.eye(4)
        T[:3, :3] = self.rotation.as_matrix()
        T[:3, 3] = self.translation
        return T

    def to_ros_msg(self) -> Transform:
        """! Converts the pose to a geometry_msg/Transform ROS message. This method is only available when ROS is installed on your system."""
        assert ROS_AVAILABLE, "ROS not found (perhaps you didn't source it?)"
        tf = Transform()
        tf.translation.x = self.translation[0]
        tf.translation.y = self.translation[1]
        tf.translation.z = self.translation[2]
        q = self.rotation.as_quat()
        tf.rotation.x = q[0]
        tf.rotation.y = q[1]
        tf.rotation.z = q[2]
        tf.rotation.w = q[3]
        return tf


def load_camera_parameters(camera_name: str) -> Tuple[np.ndarray]:
    """! Load the camera parameters. This assumes you have calibrated the camera with the given name."""
    # Load parameters
    param_file_name = join_path(
        Marker.marker_path(),
        "calibrate",
        camera_name,
        "camera_parameters.dat",
    )

    if not isfile(param_file_name):
        raise RuntimeError(
            f"Calibration parameters not found for the camera named '{camera_name}'."
        )

    with open(param_file_name, "rb") as f:
        params = pickle.load(f)

    return np.asarray(params["camera_matrix"]), np.asarray(
        params["distortion_coefficients"]
    )


def load_detector(dict_name: str) -> cv2.aruco.ArucoDetector:
    """! Load an aruco detector."""
    return cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(ARUCO_DICT[dict_name]),
        cv2.aruco.DetectorParameters(),
    )


def marker_length_to_object_points(marker_length: float) -> np.ndarray:
    """! Prepares the object points, given marker length. This is for the OpenCV solvePnP method."""
    half_marker_length = 0.5 * marker_length
    return np.array(
        [
            [-half_marker_length, half_marker_length, 0],
            [half_marker_length, half_marker_length, 0],
            [half_marker_length, -half_marker_length, 0],
            [-half_marker_length, -half_marker_length, 0],
        ],
    )


class SingleMarkerPoseEstimation:

    """! Class for estimating a single marker pose."""

    def __init__(self, camera: Camera, marker_length: float):
        """! Initializer for the SingleMarkerPoseEstimation class."""
        # Setup object points
        self.object_points = marker_length_to_object_points(marker_length)

        # Get camera parameters
        self.camera_matrix, self.distortion_coefficients = load_camera_parameters(
            camera.name
        )

    def estimate_marker_pose(self, corners: np.ndarray) -> Union[MarkerPose, None]:
        """Estimate the marker pose given the detected corners. Note, if the estimation was unsuccessful None is returned."""
        # https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d
        ret, rvec, tvec = cv2.solvePnP(
            self.object_points,
            corners,
            self.camera_matrix,
            self.distortion_coefficients,
            flags=0,
        )

        if ret:
            return MarkerPose(rvec, tvec)


class BatchMarkerPoseEstimation:

    """! Class for estimating the pose for several markers."""

    def __init__(self, camera: Camera, ids: List[int], marker_lengths: List[float]):
        """! Initializer for the BatchMarkerPoseEstimation class."""
        # Setup object points
        self.ids = ids
        self.marker_lengths = marker_lengths
        self.object_points = [marker_length_to_object_points(l) for l in marker_lengths]

        # Get camera parameters
        self.camera_matrix, self.distortion_coefficients = load_camera_parameters(
            camera.name
        )

    def estimate_marker_pose(
        self, marker_id: int, corners: np.ndarray
    ) -> Union[Marker, None]:
        """! Estimate the marker pose given the detected corners and its identifier. Note, if the estimation was unsuccessful None is returned."""
        for i, id in enumerate(self.ids):
            if id == marker_id:
                ret, rvec, tvec = cv2.solvePnP(
                    self.object_points[i],
                    corners,
                    self.camera_matrix,
                    self.distortion_coefficients,
                    flags=0,
                )
                if ret:
                    return MarkerPose(rvec, tvec)


class DetectSingleMarkerPoseFromCameraCallback(CameraViewerCallback):
    """! Callback class for the Camera viewer that detects corners in an image for a single marker, then draws the marker."""

    def __init__(self, camera: Camera, dict_name: str, marker_index: int):
        """! Initializer for the DetectSingleMarkerPoseFromCameraCallback class."""

        # Setup base class
        super().__init__()

        # Class attributes
        self.marker_index = marker_index

        # Get camera parameters
        self.camera_matrix, self.distortion_coefficients = load_camera_parameters(
            camera.name
        )

        # Load detector
        self.detector = load_detector(dict_name)

        # For debugging
        # marker_length = 0.05  # using marker: {dict: DICT_4X4_50, id: 0}
        # self.pose_detector = SingleMarkerPoseEstimation(camera, marker_length)

    def call(self, img):
        """! Main callback for the DetectSingleMarkerPoseFromCameraCallback class."""
        # https://docs.opencv.org/4.x/d2/d1a/classcv_1_1aruco_1_1ArucoDetector.html#a0c1d14251bf1cbb06277f49cfe1c9b61
        corners, ids, _ = self.detector.detectMarkers(img)

        # Draw marker if found
        for i, c in enumerate(corners):
            if ids[i] == self.marker_index:
                # https://docs.opencv.org/4.x/de/d67/group__objdetect__aruco.html#ga2ad34b0f277edebb6a132d3069ed2909
                img = cv2.aruco.drawDetectedMarkers(img, (c,), np.array([ids[i]]))

                # For debugging
                # pose = self.pose_detector.estimate_marker_pose(c)
                # print("Pose")
                # print(" ", pose.transform())

        return img


def detect_poses_from_camera(
    camera: Camera, callback: CameraViewerCallback, report_loop_duration: bool
) -> None:
    """! Run the pose detection process."""
    # Run camera collection
    CameraViewer(
        "Marker detection",
        camera,
        callback=callback,
        report_loop_duration=report_loop_duration,
    ).spin()

    # Close camera
    camera.close()
