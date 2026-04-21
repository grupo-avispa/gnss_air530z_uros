# ESP32 micro-ROS template using PlatformIO
![ROS2](https://img.shields.io/badge/ros2-jazzy-blue?logo=ros&logoColor=white)
![License](https://img.shields.io/github/license/grupo-avispa/micro-ros-esp32-template)

A professional-grade firmware template for ESP32 microcontrollers running [micro-ROS](https://micro.ros.org/) with FreeRTOS multitasking support. Built with [PlatformIO](https://platformio.org/), this template provides a solid foundation for building distributed robotics systems with ROS 2 integration.

## Features

- **micro-ROS Integration**: Full ROS 2 publisher/subscriber capabilities over WiFi
- **FreeRTOS Multitasking**: Concurrent task execution with priority-based scheduling
- **Time Synchronization**: Automatic clock synchronization with ROS agent
- **Hardware Abstraction**: Modular serial communication layer

## Dependencies
- [PlatformIO](https://docs.platformio.org/) (Cross-platform build system),
- [Robot Operating System (ROS) 2](https://docs.ros.org/en/jazzy/) (middleware for robotics),
- [micro-ROS](https://micro.ros.org/) (ROS 2 client library for microcontrollers),


## Configuration

### WiFi and Network Settings

Edit `include/config_transport.hpp` to set your network parameters:

```cpp
/// WiFi network SSID
static char* WIFI_SSID = "YOUR_SSID";

/// WiFi network password
static char* WIFI_PASSWORD = "YOUR_PASSWORD";

/// Micro-ROS agent IP address
static IPAddress AGENT_IP(192, 168, 0, 186);

/// Micro-ROS agent TCP port
static uint16_t AGENT_PORT = 8888;

/// ROS 2 Domain ID (0-232)
static constexpr uint32_t ROS_DOMAIN_ID = 0;
```

## Building and Flashing

Build the project:
```bash
platformio run
```

Upload to ESP32:
```bash
platformio run --target upload
```

Monitor serial output:
```bash
platformio device monitor --baud 115200
```

## Usage

### 1. Start the micro-ROS Agent

On your ROS 2 enabled machine:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
# or for WiFi:
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 2. Verify Connection

Monitor device output:
```bash
platformio device monitor --baud 115200
```

You should see:
```
Micro-ROS configuration set...
Serial read task created...
Data process task created...
Micro-ROS task created...
[TIMER] Sync timer callback called
Synchronized timestamp with PC agent
```
