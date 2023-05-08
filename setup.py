from glob import glob
from pathlib import Path

from pybind11.setup_helpers import Pybind11Extension
from setuptools import setup

PROJDIR = Path(__file__).resolve().parent
README = (PROJDIR / "README.md").read_text()

ext_modules = [
    Pybind11Extension(
        "norlab_trajectory",
        sorted(
            str(a) for a in PROJDIR.glob("src/*.cpp")
        ),  # Sort source files for reproducibility
        include_dirs=[str(PROJDIR / "include")],
    ),
]

setup(
    name="norlab-trajectory",
    version="0.1.1",
    description="Trajectory interpolation using Gaussian processes",
    long_description=README,
    long_description_content_type="text/markdown",
    url="https://github.com/norlab-ulaval/norlab_trajectory",
    author="simon-pierre",
    author_email="simon-pierre.deschenes.1@ulaval.ca",
    license="BSD-2.0",
    classifiers=[
        "License :: OSI Approved :: BSD-2.0 License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.9",
    ],
    include_package_data=True,
    ext_modules=ext_modules,
)
