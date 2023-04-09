import sys
import time
import aruco_markers as am

"""

Server Listener Example
=======================

This example assumes you have started the aruco server.

First open a new terminal and run the following.

$ aruco server

Then in a second terminal, execute the following

1. $ cd /path/to/aruco_markers/example
2. $ python server_listener.py LABEL

The LABEL should be an identifier, it can be anything.

NOTE, you can start multiple listeners concurrently by opening a new
terminal and running the server_listener.py script again. A window
will appear for every process.

"""


def main():
    try:
        label = sys.argv[1]
    except IndexError:
        label = str(time.time_ns())
    camera = am.ServerCamera("server_camera")
    callback = am.NullCameraViewerCallback()
    am.CameraViewer(f"server_listener_{label}", camera, callback).spin()


if __name__ == "__main__":
    main()
