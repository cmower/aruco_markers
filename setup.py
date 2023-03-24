from setuptools import setup

setup(
    name="aruco_markers",
    description="A compact Python package for handling ArUCo markers.",
    version="1.0.0",
    author="Christopher E. Mower",
    author_email="christopher.mower@kcl.ac.uk",
    packages=["aruco_markers"],
    entry_points={
        "console_scripts": [
            "aruco = aruco_markers.main:main",
        ],
    },
    install_requires=[
        "opencv-contrib-python",
        "numpy",
        "reportlab",
        "attrs",
    ],
)
