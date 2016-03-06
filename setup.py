#!/usr/bin/env python
import ez_setup
ez_setup.use_setuptools()
from setuptools import setup, find_packages

setup(
    name = "sr.robot",
    version = "0.1",
    packages = find_packages(),
    namespace_packages = ["sr"],
    description = "Student Robotics robot hardware API",
    install_requires = ["pyserial >= 2.6",
                        "pyudev >= 0.15",
                        "libusb1 >= 1.4",
                        "pykoki >= 0.0.1"
                    ],
)
