import sys
import argparse
from collections import OrderedDict

from .marker import Marker
from .camera import cvCamera
from .calibrate import collect_data, calibrate, Checkerboard
from .detect import detect_poses_from_camera, DetectSingleMarkerPoseFromCameraCallback

#
# Helper variables/methods for setting up documentation
#

command_descr = OrderedDict()
command_descr["collect"] = "Collect data with checkerboard to calibrate a camera."
command_descr["calibrate"] = "Calibrate a camera."
command_descr["generate"] = "Generate marker tags."
command_descr["detect"] = "Detect marker poses from camera feed."

longest_cmd_name = max(command_descr, key=lambda k: len(k))
len_longest_cmd_name = len(longest_cmd_name)

doc_main = """usage: aruco COMMAND [OPTIONS]

Available commands

"""

for command in command_descr:
    len_cmd_name = len(command)
    pad = " " * (len_longest_cmd_name - len_cmd_name) + "  "
    descr = command_descr[command]
    doc_main += f"{command}{pad}{descr}\n"

doc_main += """
Use --help for details on each command, e.g.

$ aruco generate --help
"""


def init_argparser(cmd):
    parser = argparse.ArgumentParser(
        usage="%(prog)s " + cmd + " [OPTIONS]",
        description=command_descr[cmd],
        add_help=False,
    )

    parser.add_argument(
        "-h",
        "--help",
        action="help",
        default=argparse.SUPPRESS,
        help="Show this help message and exit.",
    )

    return parser


def helptxt(*descr, default=None):
    txt = " ".join(descr)
    if default is not None:
        txt += f" The default is '{default}'."

    return txt


#
# Main functions for each command
#


def generate_main(argv):
    # Setup argument parser
    parser = init_argparser("generate")

    dict_param_default = "DICT_ARUCO_ORIGINAL"
    parser.add_argument(
        "-d",
        "--dict",
        type=str,
        default=dict_param_default,
        help=helptxt(
            "The dictionary for the ArUCo marker.", default=dict_param_default
        ),
    )

    id_param_default = 100
    parser.add_argument(
        "-i",
        "--id",
        type=int,
        default=id_param_default,
        help=helptxt(
            "Identifier of the marker.",
            "It has to be a valid id in the specified dictionary.",
            default=id_param_default,
        ),
    )

    sidepixels_param_default = 200
    parser.add_argument(
        "-s",
        "--sidepixels",
        type=int,
        default=sidepixels_param_default,
        help=helptxt("Size of the ArUCo tag.", default=sidepixels_param_default),
    )

    borderbits_param_default = 1
    parser.add_argument(
        "-b",
        "--borderbits",
        type=int,
        default=borderbits_param_default,
        help=helptxt("Width of the marker border.", default=borderbits_param_default),
    )

    width_param_default = 500
    parser.add_argument(
        "-w",
        "--width",
        type=int,
        default=width_param_default,
        help=helptxt(
            "Width of the marker in millimeters.", default=width_param_default
        ),
    )

    pad_param_default = 20
    parser.add_argument(
        "-p",
        "--pad",
        type=int,
        default=pad_param_default,
        help=helptxt(
            "Padding around marker in millimeters.",
            "The image is positioned on an A4 pdf.",
            default=pad_param_default,
        ),
    )

    label_param_default = ""
    parser.add_argument(
        "-l",
        "--label",
        type=str,
        default=label_param_default,
        help=helptxt(
            "Add an optional label to the marker.",
            default=label_param_default,
        ),
    )

    parser.add_argument(
        "--noshow",
        action="store_true",
        default=False,
        help=helptxt("Display the marker.", "If unspecified the tag is shown."),
    )

    parser.add_argument(
        "--save",
        action="store_true",
        help=helptxt(
            "Save the marker.",
            "Tags are saved in ~/aruco_markers/tags;",
            "this directory is created when it does not already exist.",
        ),
    )

    args = parser.parse_args(argv)

    # Create marker object and optionally show/save
    marker = Marker(
        args.dict,
        args.id,
        args.sidepixels,
        args.borderbits,
        args.width,
        args.pad,
        args.label,
    )

    if not args.noshow:
        marker.show()

    if args.save:
        marker.save()


def collect_main(argv):
    # Setup argument parser
    parser = init_argparser("collect")

    cameraindex_param_default = 0
    parser.add_argument(
        "-c",
        "--cameraindex",
        type=int,
        default=cameraindex_param_default,
        help=helptxt(
            "Camera index.",
            "This corresponds to /dev/videoN where N is the camera index.",
            default=cameraindex_param_default,
        ),
    )

    width_param_default = 9
    parser.add_argument(
        "-wb",
        "--width",
        type=int,
        default=width_param_default,
        help=helptxt(
            "The number of points in the checkerboard width.",
            default=width_param_default,
        ),
    )

    height_param_default = 6
    parser.add_argument(
        "-hb",
        "--height",
        type=int,
        default=height_param_default,
        help=helptxt(
            "The number of points in the checkerboard height.",
            default=height_param_default,
        ),
    )

    squaresize_param_default = 20
    parser.add_argument(
        "-s",
        "--squaresize",
        type=float,
        default=squaresize_param_default,
        help=helptxt(
            "The size of each square in the checkerboard in millimeters.",
            default=squaresize_param_default,
        ),
    )

    parser.add_argument(
        "-r",
        "--reportduration",
        action="store_true",
        default=False,
        help=helptxt(
            "Report the duration of each loop in the camera feed callback.",
            "By default, the duration is not reported.",
        ),
    )

    args = parser.parse_args(argv)

    # Setup camera and checkerboard
    camera = cvCamera(args.cameraindex)
    checkerboard = Checkerboard(
        camera.name.replace(" ", "-"),  # ensure no space in name
        args.width,
        args.height,
        args.squaresize,
    )

    # Collect data
    collect_data(camera, checkerboard, args.reportduration)


def calibrate_main(argv):
    # Setup argument parser
    parser = init_argparser("calibrate")

    parser.add_argument(
        "-c",
        "--cameraname",
        type=str,
        required=True,
        help="The name of the camera to be calibrated.",
    )

    param_maxfiles_default = 200
    parser.add_argument(
        "-m",
        "--maxfiles",
        type=int,
        default=param_maxfiles_default,
        help=helptxt(
            "The maximum number of files to process.",
            "When there is more files than the given maximum, the data is downsampled.",
            default=param_maxfiles_default,
        ),
    )

    parser.add_argument(
        "-p",
        "--plot",
        action="store_true",
        default=False,
        help=helptxt(
            "Plot statistics.",
            "Note, this changes the algorithm used to estimate the camera parameters.",
            "The calibration uses the LM algorithm as an optimizer which adapts the step length on each iteration.",
            "The difference when plotting is that the step length is constant.",
            "This is because, in order to collect the evolution of the algorithm outputs, we only perform 1 iteration of the algorithm.",
            "The initial guess for the optimization is then set as the previous solution.",
        ),
    )

    param_maxiter_default = 50
    parser.add_argument(
        "-mi",
        "--maxiter",
        type=int,
        default=param_maxiter_default,
        help=helptxt(
            "The maximum number of iterations for the calibration procedure.",
            default=param_maxiter_default,
        ),
    )

    param_eps_default = -1.0
    parser.add_argument(
        "-e",
        "--eps",
        type=float,
        default=param_eps_default,
        help=helptxt(
            "Termination epsilon for calibration procedure. When less than or equal to zero, then the double precision machine epsilon is used.",
            default=param_eps_default,
        ),
    )

    args = parser.parse_args(argv)

    # Compute camera parameters
    eps = args.eps if args.eps > 0.0 else None
    calibrate(args.cameraname, args.maxfiles, args.plot, max_iter=args.maxiter, eps=eps)

    print("\033[92mCompleted calibration successfully.\033[0m")


def detect_main(argv):
    # Setup argument parser
    parser = init_argparser("detect")

    cameraindex_param_default = 0
    parser.add_argument(
        "-c",
        "--cameraindex",
        type=int,
        default=cameraindex_param_default,
        help=helptxt(
            "Camera index.",
            "This corresponds to /dev/videoN where N is the camera index.",
            default=cameraindex_param_default,
        ),
    )

    dict_param_default = "DICT_ARUCO_ORIGINAL"
    parser.add_argument(
        "-d",
        "--dict",
        type=str,
        default=dict_param_default,
        help=helptxt(
            "The dictionary for the ArUCo marker.", default=dict_param_default
        ),
    )

    markerindex_param_default = 0
    parser.add_argument(
        "-m",
        "--markerindex",
        type=int,
        default=markerindex_param_default,
        help=helptxt(
            "Marker index.",
            default=cameraindex_param_default,
        ),
    )

    parser.add_argument(
        "-r",
        "--reportduration",
        action="store_true",
        default=False,
        help="Report the duration of each loop in the camera feed callback. "
        + "By default, the duration is not reported.",
    )

    args = parser.parse_args(argv)

    # Run pose detection
    camera = cvCamera(args.cameraindex)
    callback = DetectSingleMarkerPoseFromCameraCallback(
        camera, args.dict, args.markerindex
    )

    detect_poses_from_camera(camera, callback, args.reportduration)


#
# Main entry point
#


def main():
    cmd_main_map = {
        "generate": generate_main,
        "collect": collect_main,
        "calibrate": calibrate_main,
        "detect": detect_main,
    }

    # Get command and arguments
    try:
        cmd = sys.argv[1]
    except IndexError:
        print("\033[91m" + "[ERROR] unable to parse command line input" + "\033[0m")
        print(doc_main)
        return 1

    if cmd in {"-h", "--help", "help"}:
        print(doc_main)
        return 0

    # Get main function
    try:
        main = cmd_main_map[cmd]
    except KeyError:
        print("\033[91m" + f"[ERROR] did not recognize command '{cmd}'" + "\033[0m")
        print(doc_main)
        return 1

    # Execute main function and return value
    return main(sys.argv[2:])
