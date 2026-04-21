// Copyright (c) 2026 Alberto J. Tudela Roldán
// Copyright (c) 2026 Grupo Avispa, DTE, Universidad de Málaga
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file main.cpp
 * @brief Main firmware for ESP32 micro-ROS node with FreeRTOS multitasking.
 *
 * This template provides a foundation for building micro-ROS enabled embedded
 * systems using ESP32 with FreeRTOS. It includes:
 * - Serial communication infrastructure
 * - ROS2 publisher/subscriber setup
 * - Time synchronization with ROS agent
 * - Multi-task architecture for concurrent operations
 */

// Arduino Platform
#include <Arduino.h>

// Local configuration
#include "config_ros.hpp"
#include "config_transport.hpp"  // TODO: Update with transport settings
#include "macros.hpp"

// micro-ROS Libraries
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

// ROS2 Context and Executor
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t sync_timer;  ///< Timer for periodic time synchronization with agent
rcl_publisher_t navsat_publisher;
sensor_msgs__msg__NavSatFix msg;

// FreeRTOS Task Management
QueueHandle_t data_queue;        ///< Queue for inter-task communication
TaskHandle_t serialGNSSTaskHandle;   ///< Handle for serial read task
TaskHandle_t navSatPublishTaskHandle; ///< Handle for NavSatFix publish task
TaskHandle_t rosTaskHandle;      ///< Handle for ROS executor task

// Hardware Configuration (TODO: Update for your specific hardware)
constexpr uint8_t RXD2 = 17;   ///< UART2 Receive pin
constexpr uint8_t TXD2 = 255;  ///< UART2 Transmit pin (disabled with 255)

// Time Synchronization Variables
/// Timeout for ROS agent sync session in milliseconds
const int SYNC_TIMEOUT_MS = 2000;
/// Synchronized system time in milliseconds from agent
int64_t ros_synced_time_ms = 0;
/// Synchronized system time in nanoseconds from agent
int64_t ros_synced_time_ns = 0;
/// Local time (milliseconds) at last synchronization point
long ms_before_sync = 0;

/**
 * @brief Task for reading GNSS data from serial communication.
 * @param pvParameters Task parameters (unused).
 */
void SerialGNSSReadTask(void* pvParameters);

/**
 * @brief Task for publishing NavSatFix messages to ROS2.
 * @param pvParameters Task parameters (unused).
 */
void NavSatPublishTask(void* pvParameters);

/**
 * @brief Task for running the micro-ROS executor.
 * @param pvParameters Task parameters (expected value: 1).
 */
void vTaskMicroROS(void* pvParameters);

/**
 * @brief Callback function for synchronizing time with the ROS agent.
 * @param timer Pointer to the timer object.
 * @param last_call_time Time of the last callback invocation.
 */
void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time);

/**
 * @brief Arduino setup function. Initializes hardware, ROS2, and creates tasks.
 */
void setup() {
  // Initialize serial communication with PC
  Serial.begin(115200);

  // Initialize UART2 for sensor communication
  // Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // Configure micro-ROS WiFi transport to connect to ROS agent
  // Uncomment and update the following line with your transport settings
  // (e.g., WiFi credentials, agent IP/port)
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
  //set_microros_serial_transports(Serial);
  Serial.println("micro-ROS configuration set...");

  allocator = rcl_get_default_allocator();

  // Initialize ROS2 options and set domain ID
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);

  // Initialize rclc support object with custom options
  RCCHECK(rclc_support_init_with_options(&support, 0, nullptr, &init_options, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, ROS_NODE_NAME, "", &support));

  // Create NavSatFix publisher
  RCCHECK(rclc_publisher_init_default(
    &navsat_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
    ROS_TOPIC_NAME));

  // Initialize synchronization timer (5 seconds)
  RCCHECK(
    rclc_timer_init_default2(&sync_timer, &support, RCL_MS_TO_NS(5000), sync_timer_callback, true));

  // Create ROS2 executor with capacity for 1 timer (the sync timer)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  // Add sync timer to executor
  RCCHECK(rclc_executor_add_timer(&executor, &sync_timer));

  // Create FreeRTOS queue for communication between tasks
  // Adjust queue size and item size according to your needs
  data_queue = xQueueCreate(100, sizeof(uint16_t));

  // Create task for reading GNSS serial data (core 0, priority 1)
  xTaskCreatePinnedToCore(
    SerialGNSSReadTask, "SerialGNSSReadTask", 4096, NULL, 1, &serialGNSSTaskHandle, 0);
  Serial.println("Serial GNSS read task created...");

  // Create task for publishing the NavSatFix message (core 1, priority 2)
  xTaskCreatePinnedToCore(
    NavSatPublishTask, "NavSatPublishTask", 4096, NULL, 2, &navSatPublishTaskHandle, 1);
  Serial.println("NavSat publish task created...");

  // Create task for micro-ROS executor (core 1, priority 3)
  xTaskCreatePinnedToCore(
      vTaskMicroROS, "vTaskMicroROS", 10000, (void *)1, 3, &rosTaskHandle, 1);

  if (rosTaskHandle == nullptr) {
    Serial.println("ERROR: Failed to create micro-ROS task");
    error_loop();
  }
  Serial.println("micro-ROS tasks created...");
}

/**
 * @brief Arduino loop function. Main loop is handled by FreeRTOS tasks.
 */
void loop() {
  // Main loop does nothing, tasks handle everything via FreeRTOS
}

void vTaskMicroROS(void* pvParameters) {
  configASSERT(((uint32_t)pvParameters) == 1);

  // Spin the micro-ROS executor
  for (;;) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    vTaskDelay(10);
  }
}

void SerialGNSSReadTask(void* pvParameters) {
  // TODO: Implement your serial reading logic here
  // This task should:
  // 1. Read data from serial communication (Serial2 or your interface)
  // 2. Parse the incoming data according to your protocol
  // 3. Send processed data to the data_queue for further processing

  while (true) {
    // TODO: Add your serial reading implementation
    // Example structure:
    // if (Serial2.available()) {
    //   uint8_t byte = Serial2.read();
    //   // Process byte and parse your protocol
    //   // Send parsed data to queue:
    //   // if (xQueueSend(data_queue, &processed_data,
    //   //                 portMAX_DELAY) != pdTRUE) {
    //   //   Serial.println("ERROR: Failed to send data to queue");
    //   // }
    // }

    vTaskDelay(10);  // Prevent watchdog timer issues
  }
}

void NavSatPublishTask(void* pvParameters) {
  while (true) {
    // Calculate seconds and nanoseconds from epoch nanoseconds
    msg.header.stamp.sec = ros_synced_time_ns / 1000000000;
    msg.header.stamp.nanosec = ros_synced_time_ns % 1000000000;
    msg.header.frame_id.data = const_cast<char*>(ROS_FRAME_ID);
    msg.header.frame_id.size = strlen(ROS_FRAME_ID);
    msg.latitude = 37.7749;   // Example latitude (San Francisco)
    msg.longitude = -122.4194; // Example longitude (San Francisco)
    msg.altitude = 30.0;     // Example altitude in meters
    Serial.printf("Publishing NavSatFix: lat=%.4f, lon=%.4f, alt=%.1f\n", msg.latitude, msg.longitude, msg.altitude);
    
    rcl_ret_t ret = rcl_publish(&navsat_publisher, &msg, NULL);
    if (ret != RCL_RET_OK) {
      Serial.printf("ERROR: Failed to publish NavSatFix: %d\n", ret);
    }
    vTaskDelay(100);  // Publish at 10 Hz (adjust as needed)
  }
}

void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  Serial.println("[TIMER] Sync timer callback called");

  if (timer == NULL) {
    Serial.println("Error in timer_callback: timer parameter is nullptr");
    return;
  }

  // Synchronize time with the ROS agent
  rmw_uros_sync_session(SYNC_TIMEOUT_MS);

  if (rmw_uros_epoch_synchronized()) {
    // Get time in milliseconds and nanoseconds
    ros_synced_time_ms = rmw_uros_epoch_millis();
    ros_synced_time_ns = rmw_uros_epoch_nanos();
    ms_before_sync = millis();
    Serial.println("Synchronized timestamp with PC agent");
  } else {
    Serial.println("Error in sync_timer_callback: time not synchronized");
  }
}
