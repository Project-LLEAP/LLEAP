# Exo Micro ROS

This is a ROS2 package for the ESP32-C3 microcontroller. It is used to control the exoskeleton legs.

## Dependencies

- [PlatformIO](https://platformio.org/)
- [ESP32-Development](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/hw-reference/esp32c3/user-guide-sdk-environment.html)

I'm using WSL in order to bind the esp32 you must follow these steps: [WSL USB Device Setup](https://learn.microsoft.com/en-us/windows/wsl/connect-usb#attach-a-usb-device)

I followed these tutorials in order to get it working. 

[Micro ROS on RTOS](https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/) use freeRTOS, fallow installing ros2 and the micro-ROS build system. After you shouldn't have to create the firmware. Instead you will just use the [Micro ROS PlatformIO](https://github.com/micro-ROS/micro_ros_platformio/tree/main) template. I have set it up so install platformio and upload the main to the esp32. After you should continue the microros tutorial at the agent part. 

## Issues

- This issue should only be on esp32CAM. On WSL when binding the [esp32 to wsl](https://learn.microsoft.com/en-us/windows/wsl/connect-usb) it pulls the DTR/RTS handshake lines low. This auto resets holds the esp32 in the ROM bootloader. Must use windows as a com-bridge, it will be in /dev/ttyS(n-1) n is the COM port of the esp32 in device manager under Ports (COM & LPT). 




