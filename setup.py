from setuptools import setup

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="aruco_markers",
    description="A compact Python package for handling ArUCo markers.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    version="1.1.2",
    author="Christopher E. Mower",
    author_email="christopher.mower@kcl.ac.uk",
    url="https://github.com/cmower/aruco_markers",
    packages=["aruco_markers"],
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "aruco = aruco_markers.main:main",
        ],
    },
    project_urls={
        "Homepage": "https://cmower.github.io/aruco_markers/",
        "Documentation": "https://cmower.github.io/aruco_markers/",
        "Bug Tracker": "https://github.com/cmower/aruco_markers/issues",
        "Source": "https://github.com/cmower/aruco_markers",
    },
    install_requires=[
        "opencv-contrib-python",
        "numpy",
        "matplotlib",
        "scipy",
        "reportlab",
        "attrs",
    ],
)
