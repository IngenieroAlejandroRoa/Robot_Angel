from setuptools import setup, find_packages

setup(
    name="robot-angel",
    version="0.1.0",
    packages=find_packages(),
    include_package_data=True,
    install_requires=[
        "click>=8.1.7",
        "rich>=14.0.0"
    ],
    entry_points={
        "console_scripts": [
            "robot-angel = robot_angel.__main__:cli"
        ],
    },
    author="Robot Angel Team",
    description="🤖 Open Source Robotics IDE CLI - Core for ROS 2 + ESP32 + MicroPython",
    license="MIT",
)
