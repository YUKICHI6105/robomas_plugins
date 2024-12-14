robomas_plugins
====
## Description
It is for my Debug_CAN board.
It works on ros2 Humble Hawksbill and Foxy Fitzroy. (Using Foxy is not recommended.)

This supports only environment as 1 byte of 8 bits. (uint8_t == unsigned char)

## Usage

TODO: add example.

## Install

copy udev rule
```
sudo cp ~/ros2_ws/src/robomas_plugins/udev/60-robomas.rules /etc/udev/rules.d/60-robomas.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```