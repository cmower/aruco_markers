import os
import re
import math
import time
import pickle
import zipfile
import random
import datetime
from dataclasses import dataclass
from typing import Tuple, List, Union, Any

import cv2

import numpy as np

np.set_printoptions(
    precision=3, suppress=True, linewidth=os.get_terminal_size().columns
)
DBL_EPSILON = np.finfo(np.float64).eps

import matplotlib.pyplot as plt

plt.rcParams["text.usetex"] = True


from .marker import Marker
from .utils import mkdir, join_path, listdir, remove
from .camera import CameraViewer, CameraViewerCallback, Camera


def _pdist(p1, p2):
    """
    Distance bwt two points. p1 = (x, y), p2 = (x, y)
    """
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


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


def collect_data(
    camera: Camera, checkerboard: Checkerboard, report_loop_duration: bool
) -> None:
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


def load_gray_image_from_zip(
    zipf: zipfile.ZipFile, file_name: str, path: str
) -> np.ndarray:
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


@dataclass
class cvCalibrateInputArgs:
    objectPoints: List[np.ndarray]
    imagePoints: List[np.ndarray]
    imageSize: Tuple[int]
    cameraMatrix: Union[np.ndarray, None]
    distCoeffs: Union[np.ndarray, None]
    rvecs: Union[Tuple[np.ndarray], None]
    tvecs: Union[Tuple[np.ndarray], None]
    stdDeviationsIntrinsics: Union[np.ndarray, None]
    stdDeviationsExtrinsics: Union[np.ndarray, None]
    perViewErrors: Union[np.ndarray, None]
    flags: int
    criteria: Tuple[Union[float, int]]

    @property
    def args(self):
        return [
            self.objectPoints,
            self.imagePoints,
            self.imageSize,
            self.cameraMatrix,
            self.distCoeffs,
            self.rvecs,
            self.tvecs,
            self.stdDeviationsIntrinsics,
            self.stdDeviationsExtrinsics,
            self.perViewErrors,
            self.flags,
            self.criteria,
        ]


@dataclass
class cvCalibrateOutput:
    output: Tuple[Any]

    @property
    def rmsError(self):
        return self.output[0]

    @property
    def cameraMatrix(self):
        return self.output[1]

    @property
    def distCoeffs(self):
        return self.output[2]

    @property
    def stdDeviationsIntrinsics(self):
        return self.output[5]

    @property
    def perViewErrors(self):
        return self.output[7]

    @property
    def avgViewError(self):
        return np.mean(self.perViewErrors)


class Calibrator:
    def __init__(
        self,
        object_points,
        image_points,
        image_size,
        plot_stats=False,
        max_iter=30,
        eps=None,
    ):
        self.plot_stats = plot_stats
        self.max_iter = max_iter

        # Setup input args
        eps = eps if eps is not None else DBL_EPSILON
        max_iter = 1 if plot_stats else max_iter
        criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, max_iter, eps)

        self.input = cvCalibrateInputArgs(
            object_points,
            image_points,
            image_size,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            criteria,
        )

    def run(self):
        t0 = time.time()

        if self.plot_stats:
            iterations = list(range(1, self.max_iter + 1))
            iter_durations = []
            data = []
            for it in iterations:
                print(
                    "\rCalibrating ... " + str(it) + "/" + str(self.max_iter),
                    end="",
                    flush=True,
                )

                tt0 = time.time()

                # Calibrate one step
                output = cvCalibrateOutput(
                    cv2.calibrateCameraExtended(*self.input.args)
                )
                data.append(output)

                # Update input args
                self.input.cameraMatrix = output.cameraMatrix
                self.input.distCoeffs = output.distCoeffs
                self.input.flags = cv2.CALIB_USE_INTRINSIC_GUESS

                tt1 = time.time()

                iter_durations.append(tt1 - tt0)

        else:
            print("Calibrating ...", end=" ", flush=True)
            output = cvCalibrateOutput(cv2.calibrateCameraExtended(*self.input.args))
            print("done", end="")

        t1 = time.time()

        # Report duration
        duration = t1 - t0

        if duration < 1.0:
            calib_duration = f"{1e6 * duration:.3f} microseconds."
        elif duration > 60.0:
            calib_duration = str(datetime.timedelta(seconds=duration)) + "."
        else:
            calib_duration = f"{duration:.3f} seconds."
        print("\n\n\033[95mCalibration took\033[0m " + calib_duration, end="\n\n")

        print("\033[95mCamera Matrix:\033[0m")
        print(output.cameraMatrix)

        print("\n\033[95mDistortion Coeficients:\033[0m")
        print(output.distCoeffs.flatten())

        print("\n\033[95mrmsError:\033[0m")
        print(f"{output.rmsError:.3f}")

        print("\n\033[95mstdDeviationsIntrinsics:\033[0m")
        print(output.stdDeviationsIntrinsics.flatten())

        print("\n\033[95mAverage View Error:\033[0m")
        print(f"{output.avgViewError:.3f}")

        if self.plot_stats:
            # Plot stdDeviationsIntrinsics
            intrinsics_labels = [
                r"f_x",
                r"f_y",
                r"c_x",
                r"c_y",
                r"k_1",
                r"k_2",
                r"p_1",
                r"p_2",
                r"k_3",
                r"k_4",
                r"k_5",
                r"k_6",
                r"s_1",
                r"s_2",
                r"s_3",
                r"s_4",
                r"\tau_1",
                r"\tau_2",
            ]
            intrinsics_labels = [r"$" + label + r"$" for label in intrinsics_labels]

            n_intrinsics = len(intrinsics_labels)

            fig, ax = plt.subplots(
                nrows=3, ncols=6, sharex=True, layout="constrained", figsize=(20, 10)
            )

            for idx, (label, a) in enumerate(zip(intrinsics_labels, ax.flatten())):
                a.plot(
                    iterations,
                    [d.stdDeviationsIntrinsics[idx] for d in data],
                    "-x",
                    linewidth=2,
                    color="blue",
                    markersize=3,
                )
                a.grid()
                a.set_ylabel(label, fontsize=20)

            for i in range(6):
                ax[-1, i].set_xlabel("Iterations", fontsize=20)

            # Plot RMS error
            fig, ax = plt.subplots(
                nrows=3, sharex=True, layout="constrained", figsize=(10, 10)
            )
            ax[0].plot(iterations, [d.rmsError for d in data])
            ax[0].set_ylabel("RMS Error", fontsize=20)

            ax[1].plot(
                iterations,
                [d.avgViewError for d in data],
                "-x",
                linewidth=2,
                markersize=3,
                color="blue",
            )
            ax[1].set_ylabel("Average View Errror", fontsize=20)

            ax[2].plot(
                iterations,
                1e3 * np.array(iter_durations),
                "-x",
                linewidth=2,
                markersize=3,
                color="blue",
            )
            ax[2].set_ylabel("Iteration Duration (ms)", fontsize=20)

            ax[-1].set_xlabel("Iterations", fontsize=20)

            for a in ax.flatten():
                a.grid()

            # Show plots
            plt.show()

        return output


def calibrate(
    camera_name: str, max_num_files: int, plot: bool, max_iter: int, eps: float
) -> None:
    """! Calibrate the camera. This assumes the data is already collected."""

    # termination criteria for cv2.cornerSubPix
    criteria = (
        cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS,
        50,
        DBL_EPSILON,
    )

    # Arrays to store object points and image points from all the images.
    object_points = []  # 3d point in real world space
    image_points = []  # 2d points in image plane.

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
                object_points.append(objp)

                # Use a radius of half the minimum distance between
                # corners. This should be large enough to snap to the
                # correct corner, but not so large as to include a
                # wrong corner in the search window.
                min_distance = float("inf")
                for row in range(checkerboard.height):
                    for col in range(checkerboard.width - 1):
                        index = row * checkerboard.height + col
                        min_distance = min(
                            min_distance,
                            _pdist(corners[index, 0], corners[index + 1, 0]),
                        )
                for row in range(checkerboard.height - 1):
                    for col in range(checkerboard.width):
                        index = row * checkerboard.height + col
                        min_distance = min(
                            min_distance,
                            _pdist(
                                corners[index, 0],
                                corners[index + checkerboard.width, 0],
                            ),
                        )
                radius = int(math.ceil(min_distance * 0.5))

                # Refine corners
                corners2 = cv2.cornerSubPix(
                    img, corners, (radius, radius), (-1, -1), criteria
                )
                image_points.append(corners2)

                print("success!")

            else:
                print(
                    "\x1b[33m"
                    + "\n[WARN] "
                    + f"Did not find checkerboard in {fullpath}."
                    + "\x1b[0m"
                )

    # Ensure we have data to process
    if len(image_points) == 0:
        raise RuntimeError("No usable data.")

    # Compute calibration parameters
    output = Calibrator(
        object_points,
        image_points,
        img.shape[::-1],
        plot_stats=plot,
        max_iter=max_iter,
        eps=eps,
    ).run()

    # Save parameters
    camera_parameters = {
        "camera_matrix": output.cameraMatrix,
        "distortion_coefficients": output.distCoeffs,
    }
    fullpath = join_path(path, "camera_parameters.dat")
    with open(fullpath, "wb") as f:
        pickle.dump(camera_parameters, f)
        print("\nSaved", fullpath)
