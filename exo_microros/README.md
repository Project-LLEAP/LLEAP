# Exo Micro ROS

This is a ROS2 package for the ESP32-C3 microcontroller. It is used to control the exoskeleton legs.

## Dependencies

I haven't tested this on windows or mac. It's recommended to use WSL or a linux machine.

Install either [PlatformIO](https://platformio.org/) vscode IDE or CLI tool. 

## Installation

Upload the current package to the esp32-c3 by clicking upload or running 

```bash
platformio run -t upload
```

Next we need to create an agent on your machine that will communicate with the microros program on the esp32. [Micro ROS on freeRTOS](https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/), follow the instructions to **install ros2 and the micro-ROS build system**. After you should continue the microros tutorial at the **Creating the micro-ROS agent** section. When running the micro-ros agent make sure to run `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baud 921600`. Reset the esp32-s3 and it should take a minute for the esp32 to fully boot and start posting data.

Run `ros2 topic list` to see all the new topics that are created.


## WSL Extra Steps

To bind the esp32 you must follow these steps: [WSL USB Device Setup](https://learn.microsoft.com/en-us/windows/wsl/connect-usb#attach-a-usb-device)

## Issues

- ESP32CAM on WSL: DTR/RTS lines cause auto-reset into bootloader. Attempted to fix but instead uses new esp32-c3

## Docker

You can use docker to create the microros agent. I have not tested this.

```bash
docker run -it --net=host -v /dev:/dev --privileged ros:humble
```




