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

### Hardware Configuration

In `src/main.cpp`, configure serial communication pins:

```cpp
constexpr uint8_t RXD2 = 17;   ///< UART2 Receive pin
constexpr uint8_t TXD2 = 255;  ///< UART2 Transmit pin (255 = disabled)
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
Stack usage (executor task): XXX words
[TIMER] Sync timer callback called
Synchronized timestamp with PC agent
```

### 3. Implement Your Sensors

Modify the tasks to implement your specific sensor logic:

#### Serial Reading Task
```cpp
void SerialReadTask(void* pvParameters) {
  // 1. Read from serial interface (Serial2)
  // 2. Parse data according to your protocol
  // 3. Send to data_queue
  
  while (true) {
    if (Serial2.available()) {
      uint8_t byte = Serial2.read();
      // Process and send
      xQueueSend(data_queue, &processed_data, portMAX_DELAY);
    }
    vTaskDelay(10);
  }
}
```

#### Data Processing Task
```cpp
void DataProcessTask(void* pvParameters) {
  // 1. Receive from data_queue
  // 2. Process/filter data
  // 3. Create ROS message
  // 4. Publish
  
  uint16_t received_data;
  
  while (true) {
    if (xQueueReceive(data_queue, &received_data, portMAX_DELAY) == pdPASS) {
      // Create message
      std_msgs__msg__Int32 msg = {static_cast<int32_t>(received_data)};
      rcl_publish(&your_publisher, &msg, nullptr);
    }
  }
}
```

### 4. ROS 2 Integration

Create publishers and subscribers in `setup()`:

```cpp
// Create publisher
RCCHECK(rclc_publisher_init_default(
  &your_publisher,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  "/sensor_data"));

// Add to executor
RCCHECK(rclc_executor_add_timer(&executor, &sync_timer));
```

## Task Architecture

The firmware uses three main FreeRTOS tasks:

| Task                | Core | Priority | Purpose                                |
| ------------------- | ---- | -------- | -------------------------------------- |
| **SerialReadTask**  | 0    | 1        | Read sensor data from serial interface |
| **DataProcessTask** | 1    | 2        | Process data and publish ROS messages  |
| **vTaskMicroROS**   | 1    | 1        | Execute micro-ROS event loop           |

Task communication uses thread-safe FreeRTOS queues.

## Troubleshooting

### Connection Issues
- Verify WiFi credentials in `config_transport.hpp`
- Ensure agent IP and port are correct
- Check network connectivity: `ping agent_ip_address`

### Serial Communication
- Verify baud rate (default: 115200)
- Check pin configuration (RXD2=GPIO17, TXD2)
- Use `Serial.printf()` for debugging

### Compilation Errors
- Ensure all micro-ROS dependencies are installed
- Clear cache: `platformio run --target clean`
- Check PlatformIO version: `platformio --version`

## Performance Considerations

- **Serial Read Task**: 10ms delay (prevents watchdog timeout)
- **Data Process Task**: Waits on queue (no CPU waste)
- **ROS Executor**: 10ms spin duration + 500ms delay for timing
