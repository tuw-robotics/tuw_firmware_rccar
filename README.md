# micro-ROS based RCCar Firmware

## Quick Start

1. Clone this repository
2. Open the repo in a DevContainer
3. In the root of the repository, run `idf.py set-target esp32c6`
4. Connect to the ESP32-C6 board on the back of the car
   - Use the JTAG USB-C port for this, so that micro-ROS can use the UART port for communication
5. You can run `idf.py build flash monitor -p /dev/ttyACM0` to build flash and monitor the system all through the JTAG port. When doing everything through the JTAG, the micro-ROS agent can stay running on ttyUSB0 without interference
6. On the host (where the UART port is connected), run the micro-ROS agent using the command:
   ```bash
   docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0
   ```
7. If everything is correctly configured, you should see the ESP32 connecting to the micro-ROS agent and the node with topics and parameters being listed in the ROS 2 environment

## Notes

- You can configure the project through `idf.py menuconfig`
  - `Application-specific settings` contain the most important settings for the RCCar
- For UDP transport, the micro-ROS agent must be running in the same network as the ESP32
  - Use `docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888` to run the agent
- You can run another container with ros to interact with the system
  - Use `docker run -it --rm --net=host osrf/ros:jazzy-desktop` to run a container with ROS
  - `ros2 node list` should show the ESP32 node
  - `ros2 param list` should show the parameters of the ESP32 node
  - Parameters can be retreiaved and set using `ros2 param get` and `ros2 param set`
- Custom message definitions can be added as a package in "components/micro_ros_espidf_component/extra_packages"
  - After changing anything within the micro-ros component, you must run `idf.py clean-microros` to clean the component before building the project again
- For debugging:
  - first execute `openocd -f board/esp32c6-builtin.cfg` to start the OpenOCD server (assuming you have the ESP32-C6 board)
  - then you can simply add breakpoints and use the debugger in VSCode
- For logging to the USB JTAG:
  - Configure `menuconfig -> Component config -> ESP System Settings -> Channel for console output ---> USB Serial/JTAG Controller`
  - In the code, set it up using `usb_serial_jtag_driver_install(&usb_serial_jtag_config);`
- To configure the ODrives with the correct configuration, call `odrivetool restore-config ./scripts/working_config_HIGH_CURRENT_AUTO_CALIBRATION_M0.json` and `odrivetool restore-config ./scripts/working_config_HIGH_CURRENT_AUTO_CALIBRATION_M1.json` respectively for both motors
  - It is configured for auto calibration on startup and node id 0 and 1 respectively on the CAN bus
- To test the steup, stamped twist messages need to be published to the `/cmd_vel` topic
  - Use `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=True` to publish the messages

## On Debugging and Tracing
- The ESP32C6 has two USB type C ports, one of which is used for the JTAG interface and the other one is used for the USB serial interface
- For logging to have minimal impact on the performance (or if the serial interface is for example used for micro ros transport), the JTAG interface should be used
  - For this, configure the firmware with
  ```c
      const size_t BUF_SIZE = 1024;
    usb_serial_jtag_driver_config_t usb_cfg = {
        .rx_buffer_size = BUF_SIZE,
        .tx_buffer_size = BUF_SIZE,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
  ```
  - And port the logging config using the menuconfig option
  ```
  `menuconfig -> Component config -> ESP System Settings -> Channel for console output ---> USB Serial/JTAG Controller`
  ```
- To use debugging, the devcontainer already has the OpenOCD server installed
- To start the OpenOCD server, run the following command in the devcontainer:
  ```bash
  openocd -f board/esp32c6-builtin.cfg
  ```

For Tracing:
```
openocd   -f interface/esp_usb_jtag.cfg   -f board/esp32c6-builtin.cfg
```

Separate window:
```
telnet localhost 4444
```

Then in the telnet window:
```
reset init
resume
esp sysview start file://traces/mros_trace_01.SVDat 0 -1 -1 0 0
```

## Future Enhancements
- Add cleaner parameter handling
- Test odometry correctness and accuracy
- Add torque control
- Add IMU support (hardware is already there, but not yet implemented)