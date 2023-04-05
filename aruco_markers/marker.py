import cv2
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Union

from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm

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
        include_text: bool = True,
        add_half_marks: bool = False,
    ) -> str:
        """! Save the tag image as a PDF."""

        border = 30 * mm
        txt_fs = 3 * mm
        pad = self.pad * mm
        width = self.width * mm

        total_width = width + 2 * pad + 2 * border
        if total_width > min(A4):
            print("\x1b[33m" + "[WARN] Marker image does not fit on page." + "\x1b[0m")
        file_name_png = join_path(dirpath, self.file_name + ".png")
        file_name_pdf = join_path(dirpath, self.file_name + ".pdf")
        c = canvas.Canvas(file_name_pdf, pagesize=A4)
        if include_text:
            c.setFont("Helvetica", txt_fs)
            c.drawString(15 * mm, 15 * mm, self.file_name)
        c.drawImage(file_name_png, border + pad, border + pad, width, width)

        rect_width = 2 * pad + width
        c.rect(border, border, rect_width, rect_width, fill=0, stroke=1)

        if add_half_marks:

            def draw_line_on_rect(c, x1, y1, x2, y2):
                x1 += border
                y1 += border
                x2 += border
                y2 += border
                c.line(x1, y1, x2, y2)

            mark_length = 2 * mm
            half_mark_length = 0.5 * mark_length
            half_rect_width = rect_width * 0.5

            # Draw lower mark
            x1, y1 = half_rect_width, -half_mark_length
            x2, y2 = half_rect_width, half_mark_length
            draw_line_on_rect(c, x1, y1, x2, y2)

            # Draw left mark
            x1, y1 = -half_mark_length, half_rect_width
            x2, y2 = half_mark_length, half_rect_width
            draw_line_on_rect(c, x1, y1, x2, y2)

            # Draw right mark
            x1, y1 = rect_width - half_mark_length, half_rect_width
            x2, y2 = rect_width + half_mark_length, half_rect_width
            draw_line_on_rect(c, x1, y1, x2, y2)

            # Draw right mark
            x1, y1 = half_rect_width, rect_width - half_mark_length
            x2, y2 = half_rect_width, rect_width + half_mark_length
            draw_line_on_rect(c, x1, y1, x2, y2)

        c.save()
        return file_name_pdf

    def save(self, add_half_marks: bool = False) -> None:
        """! Save the marker tag PNG and PDF files."""
        # Get path to directory to save marker png/pdf
        tags_path = mkdir(self.marker_path(), "tags")

        # Save png/pdf and report filenames
        print("Saved")
        print(" ", self.save_png(tags_path))
        print(" ", self.save_pdf(tags_path, add_half_marks=add_half_marks))

    def show(self) -> None:
        """! Show the tag in an external window."""
        window_name = "ArUCo Tag"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, self.sidepixels, self.sidepixels)
        cv2.imshow(window_name, self.tag)
        print("Press Q to quit.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
