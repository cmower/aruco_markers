import sys
import abc
import time
import socket

import cv2
import numpy as np

from typing import Union


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


class ServerCamera(Camera):
    ANY = "0.0.0.0"
    MCAST_ADDR = "237.252.249.227"
    MCAST_PORT = 1600
    BUFFER_SIZE = 65507  # max UDP packet size

    def __init__(self, name):
        self._name = name
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.sock.bind((self.MCAST_ADDR, self.MCAST_PORT))
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 255)
        self.sock.setsockopt(
            socket.IPPROTO_IP,
            socket.IP_ADD_MEMBERSHIP,
            socket.inet_aton(self.MCAST_ADDR) + socket.inet_aton(self.ANY),
        )
        self._img = None

    @property
    def name(self):
        return self._name

    @property
    def width(self):
        return self._img.shape[1]

    @property
    def height(self):
        return self._img.shape[0]

    def read(self):
        data, _ = self.sock.recvfrom(self.BUFFER_SIZE)
        img_data = np.frombuffer(data, dtype=np.uint8)
        self._img = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
        return self._img

    def close(self):
        self.sock.close()


class ZEDMCamera(cvCamera):
    def __init__(self, index, side="left"):
        super().__init__(index)
        self.side = side

        if side == "left":
            self.extract = self.extract_left
        elif side == "right":
            self.extract = self.extract_right
        else:
            raise ValueError(f"did not recognize side '{side}'")

    @property
    def name(self):
        return super().name + f"_zedm_{self.side}"

    def extract_left(self, full_img):
        return full_img[:, : self.width]

    def extract_right(self, full_img):
        return full_img[:, self.width :]

    @property
    def width(self):
        return int(super().width / 2)

    def read(self):
        full_img = super().read()
        return self.extract(full_img)


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

    def close(self):
        pass


class NullCameraViewerCallback(CameraViewerCallback):

    """! A null camera viewer callback, this does nothing in the callback. This is for debugging purposes to check you camera is working."""

    def call(self, img: np.ndarray) -> np.ndarray:
        """! Callback that does nothing."""
        return img


class SenderServer(abc.ABC):
    @property
    @abc.abstractmethod
    def buffer_size(self):
        pass

    @abc.abstractmethod
    def send(self, img_bytes):
        """! Send image to the network."""
        pass


class UDPSenderServer(SenderServer):
    MCAST_ADDR = "237.252.249.227"
    MCAST_PORT = 1600

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 255)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    @property
    def buffer_size(self):
        return 65507  # max UDP packet size

    def send(self, img_bytes: np.ndarray):
        self.sock.sendto(img_bytes, (self.MCAST_ADDR, self.MCAST_PORT))

    def shutdown(self):
        self.sock.close()


class ServerCameraViewerCallback(CameraViewerCallback):

    """! A callback for the camera viewer that broadcasts the images on a network."""

    def __init__(self, sender_server, compression_quality=80):
        super().__init__()
        self.server = sender_server

        quality = int(compression_quality)
        self.encode_param = (cv2.IMWRITE_JPEG_QUALITY, quality)

    def tobytes(self, img: np.ndarray) -> Union[bytes, None]:
        """! Convert an image to a bytes."""
        success, img_jpg = cv2.imencode(".jpg", img, self.encode_param)
        if success:
            data = img_jpg.tobytes()
            if sys.getsizeof(data) <= self.server.buffer_size:
                return data
            else:
                print("[WARN] image to big to send to network.")
        else:
            raise RuntimeError("Unable to encode image.")

    def call(self, img: np.ndarray) -> np.ndarray:
        img_bytes = self.tobytes(img)
        if img_bytes is not None:
            self.server.send(img_bytes)
        return img

    def close(self):
        self.server.shutdown()


class CameraViewer:
    """! A class that views images from the camera. A callback can be passed which allows you to define how images should be processed."""

    def __init__(
        self,
        window_name: str,
        camera: Camera,
        callback: CameraViewerCallback,
        report_loop_duration: bool = False,
        show: bool = True,
    ):
        """! Initializer for the CameraViewer class."""
        # Setup class attributes
        self.window_name = window_name
        self.camera = camera
        self.report_loop_duration = report_loop_duration
        self.show = show
        self.is_closed = False

        # Setup callback handler
        self.callback = callback

    def step(self):
        done = False
        # Get image, pass through callback
        img = self.callback(self.camera.read())
        assert img is not None, "Remember, the 'call' method must return the image."

        # Show image
        if self.show:
            cv2.imshow(self.window_name, img)

            # Check if user quit
            if cv2.waitKey(1) == 27:
                done = True

        return done

    def spin(self) -> None:
        """Call this method to start the viewer."""
        # Report to user how to quit
        print("Press ESC to quit.")

        # Start main loop
        done = False
        while not done:
            # Get time the loop starts
            t0 = time.time()

            # Step the viewer
            done = self.step()

            # Get time the loop ends, compute duration, and report to user (optionally)
            t1 = time.time()
            duration = t1 - t0
            if self.report_loop_duration:
                print(f"Duration (sec): {duration:.5f} (fits {1./duration:.2f}Hz)")

        cv2.destroyAllWindows()
        self.close()

    def close(self):
        if not self.is_closed:
            self.callback.close()
        self.is_closed = True
