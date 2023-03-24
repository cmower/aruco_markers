from setuptools import setup

setup(
    name="aruco_markers",
    description="A compact Python library for ArUCo marker detection.",
    version="1.0.0",
    author="Christopher E. Mower",
    author_email="christopher.mower@kcl.ac.uk",
    packages=["aruco_markers"],
    entry_points={
        "console_scripts": [
            "aruco = aruco_markers.main:main",
        ],
    },
)
