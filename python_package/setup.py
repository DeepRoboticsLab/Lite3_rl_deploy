"""
Setup script for PyLite3 package
"""
from setuptools import setup, find_packages
import os

# Read README
long_description = ""
if os.path.exists("README.md"):
    with open("README.md", "r", encoding="utf-8") as f:
        long_description = f.read()

setup(
    name="pylite3",
    version="1.0.0",
    author="DeepRobotics & Claude Code",
    author_email="support@deeprobotics.cn",
    description="Python API for Lite3 Quadruped Robot Control",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/DeepRoboticsLab/Lite3_rl_deploy",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Science/Research",
        "Intended Audience :: Developers",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Scientific/Engineering :: Robotics",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.19.0,<2.0.0",
        "matplotlib>=3.3.0",
        "pybullet>=3.0.0",
    ],
    extras_require={
        "gym": ["gymnasium>=0.28.0"],
        "torch": ["torch>=1.10.0"],
        "viz": ["meshcat>=0.2.0"],
        "all": [
            "gymnasium>=0.28.0",
            "torch>=1.10.0",
            "meshcat>=0.2.0",
            "h5py>=3.0.0",
            "pandas>=1.3.0",
        ],
    },
    package_data={
        "pylite3": ["*.so", "*.pyd"],  # Include compiled extension
    },
    include_package_data=True,
)
