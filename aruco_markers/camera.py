import abc
import time

import cv2
import numpy as np


class Camera(abc.ABC):

    """! Base class for a Camera."""

    @property
    @abc.abstractmethod
    def name(self) -> str:
        """! Name of the camera."""
        pass

    @property
    @abc.abstractmethod
    def width(self) -> int:
        """! Width of the camera image."""
        pass

    @property
    @abc.abstractmethod
    def height(self) -> int:
        """! Height of the camera image."""
        pass

    @abc.abstractmethod
    def read(self) -> np.ndarray:
        """! Read an image from the camera."""
        pass

    @abc.abstractmethod
    def close(self) -> None:
        """! Close the camera interface."""
        pass


class cvCamera(Camera):

    """! Interface to the OpenCV camera."""

    def __init__(self, index: int):
        """! Initializer for the OpenCV camera interface."""
        self.index = index
        self.camera = cv2.VideoCapture(self.index)

    @property
    def name(self) -> str:
        """! Name for the camera."""
        return self.camera.getBackendName()

    @property
    def width(self) -> int:
        """! Width of the camera image."""
        return int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))

    @property
    def height(self) -> int:
        """! Height of the camera image."""
        return int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def read(self) -> np.ndarray:
        """! Read an image from the camera."""
        _, img = self.camera.read()
        return img

    def close(self) -> None:
        """! Close the camera interface."""
        self.camera.release()


class CameraViewerCallback(abc.ABC):

    """! Callback class for the camera viewer. You must implement the call method that inputs an image (from the interface) and outputs the image to be viewed."""

    def __init__(self):
        """! Initializer for the CameraViewerCallback class."""
        self.num_calls = 0

    @abc.abstractmethod
    def call(self, img: np.ndarray) -> np.ndarray:
        """! Callback method that needs to be implemented."""
        pass

    def __call__(self, img):
        """! Callback used by the CameraViewer class, this should not be overwritten."""
        self.num_calls += 1
        return self.call(img)


class NullCameraViewerCallback(CameraViewerCallback):

    """! A null camera viewer callback, this does nothing in the callback. This is for debugging purposes to check you camera is working."""

    def call(self, img: np.ndarray) -> np.ndarray:
        """! Callback that does nothing."""
        return img


class CameraViewer:
    """! A class that views images from the camera. A callback can be passed which allows you to define how images should be processed."""

    def __init__(
        self,
        window_name: str,
        camera: Camera,
        callback: CameraViewerCallback,
        report_loop_duration: bool = False,
    ):
        """! Initializer for the CameraViewer class."""
        # Setup class attributes
        self.window_name = window_name
        self.camera = camera
        self.report_loop_duration = report_loop_duration

        # Setup callback handler
        self.callback = callback

    def spin(self) -> None:
        """Call this method to start the viewer."""
        # Report to user how to quit
        print("Press ESC to quit.")

        # Start main loop
        done = False
        while not done:
            # Get time the loop starts
            t0 = time.time()

            # Get image, pass through callback, and display
            img = self.callback(self.camera.read())
            cv2.imshow(self.window_name, img)

            # Check if user quit
            if cv2.waitKey(1) == 27:
                done = True

            # Get time the loop ends, compute duration, and report to user (optionally)
            t1 = time.time()
            duration = t1 - t0
            if self.report_loop_duration:
                print(f"Duration (sec): {duration:.5f} (fits {1./duration:.2f}Hz)")

        cv2.destroyAllWindows()
