# OnRobot ROS 2

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![repo size](https://img.shields.io/github/repo-size/ABC-iRobotics/onrobot-ros2)

ROS 2 drivers for OnRobot Grippers.
This repository was inspired by [Osaka-University-Harada-Laboratory/onrobot](https://github.com/Osaka-University-Harada-Laboratory/onrobot).

## Features

- [Ubuntu 22.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&category=Laptop&vendor=Dell&vendor=HP&vendor=Lenovo&release=22.04+LTS)
- [ROS 2 Humble (Python3)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Controller for OnRobot [RG2](https://onrobot.com/en/products/rg2-gripper) / [RG6](https://onrobot.com/en/products/rg6-gripper) via Modbus/TCP

## Dependency

- pyModbusTcp>=0.3.0

## Installation

```
cd ros2_ws/src && git clone https://github.com/ABC-iRobotics/onrobot-ros2.git && cd ..
colcon build --packages-select onrobot_rg_msgs onrobot_rg_description onrobot_rg_modbus_tcp onrobot_rg_control
```

## Author

[Makány András](https://github.com/andras-makany)  - Graduate student at Obuda University

## License

This software is released under the MIT License, see [LICENSE](./LICENSE).
