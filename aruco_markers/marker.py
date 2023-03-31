import cv2
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Union

from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas

from .utils import mkdir, join_path, homedir

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


@dataclass
class Marker:

    """! Class representing a marker."""

    ## Name of dictionary, see keys of ARUCO_DICT.
    dict_name: str

    ## Identifier of the marker. It has to be a valid id in the specified dictionary.
    id: int

    ## Size of the image in pixels.
    sidepixels: int

    ## Width of the marker border.
    borderbits: int

    ## Width of the marker in millimeters.
    width: int

    ## Padding around marker image when positioned on an A4 pdf in millimeters.
    pad: int

    ## Label for the marker (optional)
    label: float = None

    @property
    def file_name(self) -> str:
        """! File anme for the marker when generated."""
        d = self.dict_name
        i = self.id
        s = self.sidepixels
        b = self.borderbits
        w = self.width
        p = self.pad
        l = f"_label_{self.label}" if self.label else ""
        return (
            f"marker_dict_{d}_id_{i}_sidepixels_{s}_borderbits_{b}_width_{w}_pad_{p}"
            + l
        )

    @property
    def dictionary(self):
        """! Dictionary for the marker."""
        return cv2.aruco.getPredefinedDictionary(ARUCO_DICT[self.dict_name])

    @property
    def tag(self):
        """! Image for the tag."""
        tag = np.zeros((self.sidepixels, self.sidepixels, 1), dtype="uint8")
        cv2.aruco.generateImageMarker(
            self.dictionary, self.id, self.sidepixels, tag, self.borderbits
        )
        return tag

    @staticmethod
    def marker_path():
        """! Path to save data."""
        return mkdir(homedir, "aruco_markers_data")

    def save_png(self, dirpath: str) -> str:
        """! Save the tag image as a PNG."""
        file_name = join_path(dirpath, self.file_name + ".png")
        cv2.imwrite(file_name, self.tag)
        return file_name

    def save_pdf(
        self,
        dirpath: str,
        pagesize: Tuple[float] = A4,
        include_text: bool = True,
        txt_loc: Union[Tuple[float], None] = None,
        txt_fs: int = 8,
    ) -> str:
        """! Save the tag image as a PDF."""
        total_width = self.width + 2 * self.pad
        if total_width > min(pagesize):
            print("\x1b[33m" + "[WARN] Marker image does not fit on page." + "\x1b[0m")
        file_name_png = join_path(dirpath, self.file_name + ".png")
        file_name_pdf = join_path(dirpath, self.file_name + ".pdf")
        c = canvas.Canvas(file_name_pdf, pagesize=pagesize)
        if include_text:
            c.setFont("Helvetica", txt_fs)
            if txt_loc is None:
                txt_loc = (10, pagesize[1] - 20)
            x, y = txt_loc
            c.drawString(x, y, self.file_name)
        c.drawImage(file_name_png, self.pad, self.pad, self.width, self.width)
        c.save()
        return file_name_pdf

    def save(self) -> None:
        """! Save the marker tag PNG and PDF files."""
        # Get path to directory to save marker png/pdf
        tags_path = mkdir(self.marker_path(), "tags")

        # Save png/pdf and report filenames
        print("Saved")
        print(" ", self.save_png(tags_path))
        print(" ", self.save_pdf(tags_path))

    def show(self) -> None:
        """! Show the tag in an external window."""
        window_name = "ArUCo Tag"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, self.sidepixels, self.sidepixels)
        cv2.imshow(window_name, self.tag)
        print("Press Q to quit.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
