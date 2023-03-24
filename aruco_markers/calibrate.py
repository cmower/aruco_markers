import re
import time
import pickle
import zipfile
import random
import datetime
from dataclasses import dataclass
from typing import Tuple, List

import cv2
import numpy as np

from .marker import Marker
from .utils import mkdir, join_path, listdir, remove
from .camera import CameraViewer, CameraViewerCallback, Camera


@dataclass
class Checkerboard:

    """! A class that represents a checkerboard."""

    ## Name of camera used to capture checkerboard.
    cameraname: str

    ## Number of squares in the checkerboard width.
    width: int

    ## Number of squares in the checkerboard height.
    height: int

    # Size of each square on the checkerboard in mm.
    squaresize: int  

    @property
    def size(self) -> Tuple[int]:
        """! The size W, H of the checkerboard."""
        return (self.width, self.height)

    @property
    def num_squares(self) -> int:
        """! The number of squares in the checkerboard."""
        return self.width * self.height

    @property
    def squaresize_m(self) -> float:
        """! The square size of the checker board in meters."""
        return float(self.squaresize) * 1e-3

    @property
    def data_path(self) -> str:
        """! The path to save data into for the calibration."""
        return mkdir(self.calibrate_path(), self.cameraname)

    @staticmethod
    def calibrate_path() -> str:
        """! The path to the calibration directory where images are saved."""
        return mkdir(Marker.marker_path(), "calibrate")

    @staticmethod
    def from_file_name(file_name: str):
        """! Load instance of Checkerboard class from a file name, note the file name should have been generated using the generate_unique_file_name function."""
        cameraname = re.search(r"cameraname_([^_]+)_", file_name).group(1)
        width = int(re.search(r"width_(\d+)", file_name).group(1))
        height = int(re.search(r"height_(\d+)", file_name).group(1))
        squaresize = int(re.search(r"squaresize_(\d+)", file_name).group(1))
        return Checkerboard(cameraname, width, height, squaresize)

    def generate_unique_file_name(self) -> str:
        """! Generate a unique file name for the data collection process."""
        n = "cameraname_" + self.cameraname
        w = "width_" + str(self.width)
        h = "height_" + str(self.height)
        ss = "squaresize_" + str(self.squaresize)
        s = "stamp_" + str(time.time_ns())
        file_name = "_".join(["checkerboard", n, w, h, ss, s]) + ".png"
        return join_path(self.data_path, file_name)

    def init_object_points(self) -> np.ndarray:
        """! Prepares the object points for calibration."""
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
        objp = np.zeros((self.num_squares, 3), np.float32)
        objp[:, :2] = np.mgrid[0 : self.width, 0 : self.height].T.reshape(-1, 2)
        objp = objp * self.squaresize_m
        return objp


class CalibrationDataCollectionCallback(CameraViewerCallback):

    """! Callback for the CameraViewer class, that collects data."""
    
    def __init__(self, checkerboard: Checkerboard):
        """! Initializer for the CalibrationDataCollectionCallback class."""
        super().__init__()
        self.num_saved_data = 0
        self.checkerboard = checkerboard

    def call(self, img: np.ndarray) -> np.ndarray:
        """! Callback method, the image is converted to gray scale, the checkerboard is detected, and if it is detected the image is saved."""
        # Convert image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find chessboard corners
        found, corners = cv2.findChessboardCorners(gray, self.checkerboard.size)

        if found:
            # Get unique file name
            file_name = self.checkerboard.generate_unique_file_name()

            # Save image
            cv2.imwrite(file_name, gray)
            self.num_saved_data += 1

            # Draw corners of checkerboard
            img = cv2.drawChessboardCorners(img, self.checkerboard.size, corners, found)

        return img


def collect_data(camera: Camera, checkerboard: Checkerboard, report_loop_duration: bool) -> None:
    """! Runs the data collection process."""
    # Define a callback for the data collection
    callback = CalibrationDataCollectionCallback(checkerboard)

    # Run data collection
    CameraViewer("Data collection", camera, callback, report_loop_duration).spin()

    # Close camera
    camera_name = camera.name
    camera.close()

    # Zip files
    zip_file_name = join_path(checkerboard.data_path, "data.zip")
    with zipfile.ZipFile(zip_file_name, "a") as zipf:
        for file_name in listdir(checkerboard.data_path):
            if not file_name.endswith(".png"):
                continue
            full_path = join_path(checkerboard.data_path, file_name)
            zipf.write(full_path, arcname=file_name)
            remove(full_path)

    # Ensure
    if callback.num_saved_data > 0:
        print(f"Data collection completed for the '{camera_name}' camera.")
        print("Data saved in", checkerboard.data_path)
    else:
        raise RuntimeError("No data captured.")


def downsample_file_names(file_names: List[str], max_num_files: int) -> List[str]:
    """! Downsamples the file names. Only downsample if we have more file names max_num_files."""
    return (
        random.sample(file_names, max_num_files)
        if len(file_names) > max_num_files
        else file_names
    )


def load_gray_image_from_zip(zipf: zipfile.ZipFile, file_name: str, path: str) -> np.ndarray:
    """! Loads the gray scale image from the zip file."""
    # Extract the file from the zip
    zipf.extract(file_name, path=path)

    # Load image
    fullpath = join_path(path, file_name)
    img = cv2.imread(fullpath)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Remove the image file
    remove(fullpath)

    return img


def calibrate(camera_name: str, max_num_files: int) -> None:
    """! Calibrate the camera. This assumes the data is already collected."""
    
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    # Iterate over file names in data_path
    print("Processing checkerboard images ...")

    # Get path to data
    path = join_path(Marker.marker_path(), "calibrate", camera_name)
    data_path = join_path(path, "data.zip")

    # Get file names from zip file
    with zipfile.ZipFile(data_path, "r") as zipf:
        file_names = downsample_file_names(
            [file_name for file_name in zipf.namelist() if file_name.endswith(".png")],
            max_num_files,
        )

        n_file_names = len(file_names)

        # Iterate over file names
        for i, file_name in enumerate(file_names):
            # Extract parameters from filename
            checkerboard = Checkerboard.from_file_name(file_name)
            objp = checkerboard.init_object_points()

            # Load image
            img = load_gray_image_from_zip(zipf, file_name, path)
            print(f"Loaded ({i+1}/{n_file_names})", file_name)

            # Find the chess board corners
            print("  Processing ...", end=" ", flush=True)
            found, corners = cv2.findChessboardCorners(img, checkerboard.size, None)

            # If found, add object points, image points (after refining them)
            if found:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)

                print("success!")

            else:
                print(
                    "\x1b[33m"
                    + "\n[WARN] "
                    + f"Did not find checkerboard in {fullpath}."
                    + "\x1b[0m"
                )

    # Ensure we have data to process
    if len(imgpoints) == 0:
        raise RuntimeError("No usable data.")
    print("Done!")

    # Compute calibration parameters
    print("Calibrating camera ...", end=" ", flush=True)
    t0 = time.time()
    ret, mtx, dist, _, _ = cv2.calibrateCamera(
        objpoints, imgpoints, img.shape[::-1], None, None
    )
    t1 = time.time()
    print("done!")
    duration = t1 - t0

    if duration < 1.0:
        calib_duration = f"{1e6 * duration:.3f} microseconds"
    elif duration > 60.0:
        calib_duration = str(datetime.timedelta(seconds=duration))
    else:
        calib_duration = f"{duration:.4f} seconds"
    print("Calibration took " + calib_duration)

    print("Camera matrix:\n", mtx, "\n\nDistorion Cooeficients:\n", dist)

    # Save parameters
    if ret:
        camera_parameters = {"camera_matrix": mtx, "distortion_coefficients": dist}
        fullpath = join_path(path, "camera_parameters.dat")
        with open(fullpath, "wb") as f:
            pickle.dump(camera_parameters, f)
            print("Saved", fullpath)
    else:
        raise RuntimeError("Unable to calibrate camera.")
