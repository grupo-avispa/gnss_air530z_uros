# gnss_air530z_uros
![ROS2](https://img.shields.io/badge/ros2-jazzy-blue?logo=ros&logoColor=white)
![License](https://img.shields.io/github/license/grupo-avispa/gnss_air530z_uros)

Professional-grade firmware for ESP32 with support for [micro-ROS](https://micro.ros.org/) and the multi-mode **Grove GPS Air530** module. Integrates FreeRTOS for concurrent multitasking and is built with [PlatformIO](https://platformio.org/). Ideal for distributed GNSS positioning applications in robotic systems.

## Features

- **micro-ROS Integration**: Full ROS 2 publisher/subscriber capabilities over WiFi
- **Multi-Mode Positioning**: Support for GPS, Beidou, GLONASS, Galileo, QZSS, and SBAS
- **FreeRTOS Multitasking**: Concurrent task execution with priority-based scheduling
- **Time Synchronization**: Automatic clock synchronization with ROS agent
- **Communication Layer**: Modular abstraction for GPS serial communication

## Dependencies
- [PlatformIO](https://docs.platformio.org/) (Cross-platform build system),
- [Robot Operating System (ROS) 2](https://docs.ros.org/en/jazzy/) (robotics middleware),
- [micro-ROS](https://micro.ros.org/) (ROS 2 client library for microcontrollers),

## Hardware

### Grove GPS Air530
High-performance multi-mode GNSS module for precise positioning:

| Specification            | Value                                     |
| ------------------------ | ----------------------------------------- |
| **Power Supply Voltage** | 3.3V/5V                                   |
| **Operating Current**    | Up to 60mA                                |
| **Warm Start Time**      | 4s                                        |
| **Cold Start Time**      | 30s                                       |
| **Baudrate**             | 9600 bps                                  |
| **Protocol**             | NMEA                                      |
| **Positioning Modes**    | GPS, Beidou, GLONASS, Galileo, QZSS, SBAS |

**Physical Connection:**
- Red: 5V (or 3.3V)
- Black: GND
- Yellow: RX (GPIO)
- White: TX (GPIO)

Refer to the [GPS Air530 documentation](https://wiki.seeedstudio.com/Grove-GPS-Air530/) for more details.

## Configuration

### WiFi and Transport Configuration

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
