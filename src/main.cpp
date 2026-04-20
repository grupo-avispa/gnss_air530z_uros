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

// C++ Standard Libraries
#include <vector>
#include <cmath>
#include <cstdint>

// Arduino Platform
#include <Arduino.h>

// Local configuration
#include "config_transport.hpp"  // TODO: Update with transport settings

// micro-ROS Libraries
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

// ROS2 Context and Executor
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// TODO: Add your specific publishers/subscribers member variables here
rcl_timer_t sync_timer;  ///< Timer for periodic time synchronization with agent

// FreeRTOS Task Management
QueueHandle_t data_queue;        ///< Queue for inter-task communication
TaskHandle_t serialTaskHandle;   ///< Handle for serial read task
TaskHandle_t dataProcessTaskHandle;  ///< Handle for data processing task
TaskHandle_t rosTaskHandle;      ///< Handle for ROS executor task

// Hardware Configuration (TODO: Update for your specific hardware)
constexpr uint8_t RXD2 = 17;   ///< UART2 Receive pin
constexpr uint8_t TXD2 = 255;  ///< UART2 Transmit pin (disabled with 255)

// TODO: Define your sensor-specific constants here

// TODO: Define your sensor-specific data structures here

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
 * @brief Task for reading data from serial communication.
 * @param pvParameters Task parameters (unused).
 */
void SerialReadTask(void* pvParameters);

/**
 * @brief Task for processing data and publishing ROS messages.
 * @param pvParameters Task parameters (unused).
 */
void DataProcessTask(void* pvParameters);

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
 * @brief Macro for checking ROS2 function return values.
 *        Enters error loop if the function fails.
 * @param fn The ROS2 function to check.
 */
#define RCCHECK(fn)                  \
  {                                  \
    rcl_ret_t temp_rc = fn;          \
    if ((temp_rc != RCL_RET_OK)) {   \
      error_loop();                  \
    }                                \
  }

/**
 * @brief Soft version of RCCHECK that logs but doesn't stop execution.
 * @param fn The ROS2 function to check.
 */
#define RCSOFTCHECK(fn)              \
  {                                  \
    rcl_ret_t temp_rc = fn;          \
    if ((temp_rc != RCL_RET_OK)) {   \
    }                                \
  }

/**
 * @brief Infinite error loop function. If something fails, the device stops here.
 *        Flashes the device in a continuous loop.
 */
void error_loop() {
  while (true) {
    delay(100);
  }
}

/**
 * @brief Arduino setup function. Initializes hardware, ROS2, and creates tasks.
 */
void setup() {
  // Initialize serial communication with PC
  Serial.begin(115200);

  // Initialize UART2 for sensor communication
  // Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // Configure micro-ROS WiFi transport to connect to ROS agent
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
  Serial.println("Micro-ROS configuration set...");

  allocator = rcl_get_default_allocator();

  // Initialize ROS2 options and set domain ID
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);

  // Create ROS2 support structure
  RCCHECK(rclc_support_init_with_options(&support, 0, nullptr, &init_options, &allocator));

  // Create ROS2 node
  RCCHECK(rclc_node_init_default(&node, "minibot_node", "", &support));

  // TODO: Create your specific publishers/subscribers here
  // Example:
  // RCCHECK(rclc_publisher_init_default(
  //   &your_publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
  //   "/your_topic"));

  // Initialize synchronization timer (5 seconds)
  RCCHECK(rclc_timer_init_default(&sync_timer, &support, RCL_MS_TO_NS(5000), sync_timer_callback));

  // Create ROS2 executor
  RCCHECK(rclc_executor_init(
    &executor,
    &support.context,
    1,  // Number of entities the executor can handle
    &allocator));

  // Add timer to executor
  RCCHECK(rclc_executor_add_timer(&executor, &sync_timer));

  // Create FreeRTOS queue for communication between tasks
  // TODO: Adjust queue size and item size according to your needs
  data_queue = xQueueCreate(100, sizeof(uint16_t));

  // Create task for reading serial data (core 0, priority 1)
  xTaskCreatePinnedToCore(
    SerialReadTask,
    "SerialReadTask",
    4096,
    nullptr,
    1,
    &serialTaskHandle,
    0);
  Serial.println("Serial read task created...");

  // Create task for data processing (core 1, priority 2)
  xTaskCreatePinnedToCore(
    DataProcessTask,
    "DataProcessTask",
    4096,
    nullptr,
    2,
    &dataProcessTaskHandle,
    1);
  Serial.println("Data process task created...");

  // Create task for micro-ROS executor (core 1, priority 1)
  xTaskCreatePinnedToCore(
    vTaskMicroROS,
    "vTaskMicroROS",
    10000,
    reinterpret_cast<void*>(1),
    1,
    &rosTaskHandle,
    1);

  if (rosTaskHandle == nullptr) {
    Serial.println("ERROR: Failed to create micro-ROS task");
    error_loop();
  }
  Serial.println("Micro-ROS task created...");
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
    const uint32_t stack_high_water = uxTaskGetStackHighWaterMark(nullptr);
    Serial.printf("Stack usage (executor task): %u words\n", stack_high_water);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    vTaskDelay(500);
  }
}

void SerialReadTask(void* pvParameters) {
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

void DataProcessTask(void* pvParameters) {
  // TODO: Implement your data processing and ROS publishing logic here
  // This task should:
  // 1. Receive data from data_queue
  // 2. Process the data according to your application logic
  // 3. Create ROS messages
  // 4. Publish messages using rcl_publish()

  uint16_t received_data;

  while (true) {
    // Attempt to receive data from the queue
    if (xQueueReceive(data_queue, &received_data, portMAX_DELAY) == pdPASS) {
      // TODO: Process received_data
      // Serial.printf("Received data: %u\n", received_data);

      // TODO: Create and publish your ROS message
      // Examples:
      // rcl_publish(&your_publisher, &your_message, nullptr);
      // Serial.println("Message published");
    }
  }
}

// TODO: Implement your utility functions here
// Examples of functions you might need:
// - Data parsing functions
// - Message creation functions
// - Sensor calibration functions
// - Data validation functions
// - Protocol-specific handlers

void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  Serial.println("[TIMER] Sync timer callback called");

  if (timer == nullptr) {
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
