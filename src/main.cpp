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
#include "config_transport.hpp"
#include "macros.hpp"

// micro-ROS Libraries
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include <TinyGPS++.h>

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

// Hardware Configuration
constexpr uint8_t RXD2 = 16;   ///< UART2 Receive pin
constexpr uint8_t TXD2 = 17;  ///< UART2 Transmit pin (disabled with 255)

// Time Synchronization Variables
/// Timeout for ROS agent sync session in milliseconds
const int SYNC_TIMEOUT_MS = 2000;
/// Synchronized system time in nanoseconds from agent
int64_t ros_synced_time_ns = 0;
/// Local time (microseconds) at last synchronization point
uint64_t micros_before_sync = 0;
/// Mutex to protect time synchronization variables
portMUX_TYPE timeSyncMutex = portMUX_INITIALIZER_UNLOCKED;
/// Flag to indicate if time has been synchronized with agent
volatile bool time_synchronized = false;

// Mutexes for shared data access
portMUX_TYPE gpsMutex = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE msgMutex = portMUX_INITIALIZER_UNLOCKED;

// Structure to hold GPS data read from serial communication
struct GpsData {
    double lat;
    double lon;
    double alt;
    uint8_t status;
    uint8_t service;
};

TinyGPSPlus gps;
GpsData gps_data;

/**
 * @brief Task for reading GNSS data from serial communication.
 * @param pvParameters Task parameters (unused).
 */
void SerialGNSSReadTask(void* pvParameters);

/**
 * @brief Helper function to get the name of the GNSS service as a string.
 * @param service The service identifier from the NavSatStatus message.
 * @return A constant character pointer to the name of the service.
 */
const char* getServiceName(uint8_t service);

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
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Configure micro-ROS WiFi transport to connect to ROS agent
  // Uncomment and update the following line with your transport settings
  // (e.g., WiFi credentials, agent IP/port)
  //set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
  set_microros_serial_transports(Serial);
  Serial.println("micro-ROS configuration set...");

  allocator = rcl_get_default_allocator();

  // Initialize ROS2 options and set domain ID
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

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
  data_queue = xQueueCreate(1, sizeof(GpsData));

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
  while (true) {
    while (Serial2.available() > 0) {
      if (gps.encode(Serial2.read())) {
          portENTER_CRITICAL(&gpsMutex);
          gps_data.lat = gps.location.lat();
          gps_data.lon = gps.location.lng();
          gps_data.alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
          gps_data.service = static_cast<u_int8_t>(gps.getCurrentSentenceSystem());
          portEXIT_CRITICAL(&gpsMutex);
          xQueueOverwrite(data_queue, &gps_data);
      }
    }
    vTaskDelay(10);  // Prevent watchdog timer issues
  }
}

const char* getServiceName(uint8_t service) {
  if (service & sensor_msgs__msg__NavSatStatus__SERVICE_GPS)     return "GPS";
  if (service & sensor_msgs__msg__NavSatStatus__SERVICE_GLONASS) return "GLONASS";
  if (service & sensor_msgs__msg__NavSatStatus__SERVICE_GALILEO) return "GALILEO";
  if (service & sensor_msgs__msg__NavSatStatus__SERVICE_COMPASS) return "BEIDOU";
  return "UNKNOWN";
}

void NavSatPublishTask(void* pvParameters) {
  while (true) {
    // Only publish if the time is synchronized with the ROS2 agent
    if (!time_synchronized) {
      Serial.println("Time not synchronized, skipping publish...");
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // Attempt to dequeue data (500ms timeout)
    // This prevents CPU polling/busy-waiting if the GPS is idle
    if (xQueueReceive(data_queue, &gps_data, pdMS_TO_TICKS(500)) == pdTRUE) {
      // Synchronized timestamp
      portENTER_CRITICAL(&timeSyncMutex);
      int64_t synced_time_ns = ros_synced_time_ns;
      uint64_t sync_micros = micros_before_sync;
      portEXIT_CRITICAL(&timeSyncMutex);

      uint64_t elapsed_micros = micros() - sync_micros;
      int64_t current_time_ns = synced_time_ns + (elapsed_micros * 1000);

      // Filling the NavSatFix message
      msg.header.stamp.sec = current_time_ns / 1000000000LL;
      msg.header.stamp.nanosec = current_time_ns % 1000000000LL;
      msg.header.frame_id.data = const_cast<char*>(ROS_FRAME_ID);
      msg.header.frame_id.size = strlen(ROS_FRAME_ID);

      portENTER_CRITICAL(&gpsMutex);
      msg.latitude = gps_data.lat;
      msg.longitude = gps_data.lon;
      msg.altitude = gps_data.alt;
      msg.status.status = gps_data.status;

      switch (gps_data.service) {
        case 0:
          msg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;
          break;
        case 1:
          msg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GLONASS;
          break;
        case 2:
          msg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GALILEO;
          break;
        case 3:
          msg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_COMPASS;
          break;
        default:
          msg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_COMPASS;
          break;
      }
      portEXIT_CRITICAL(&gpsMutex);

      // Indicate that covariance is unknown (standard if error is not calculated)
      msg.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_UNKNOWN;

      // Serial.printf(
      //   "Publishing NavSatFix: lat=%.4f, lon=%.4f, alt=%.1f, time=%ld.%09lu, service=%s\n",
      //   msg.latitude,
      //   msg.longitude,
      //   msg.altitude,
      //   msg.header.stamp.sec,
      //   msg.header.stamp.nanosec,
      //   getServiceName(msg.status.service));

      rcl_ret_t ret = rcl_publish(&navsat_publisher, &msg, NULL);
    }
    // Short delay for FreeRTOS scheduling
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time){
  Serial.println("[TIMER] Sync timer callback called");

  if (timer == NULL) {
    Serial.println("Error in timer_callback: timer parameter is nullptr");
    return;
  }

  // Synchronize time with the ROS agent
  rmw_uros_sync_session(SYNC_TIMEOUT_MS);

  if (rmw_uros_epoch_synchronized()) {
    // Get synchronized time in nanoseconds
    int64_t synced_time_ns = rmw_uros_epoch_nanos();
    
    // Acquire mutex to safely update synchronized time
    portENTER_CRITICAL(&timeSyncMutex);
    ros_synced_time_ns = synced_time_ns;
    micros_before_sync = micros();
    time_synchronized = true;
    portEXIT_CRITICAL(&timeSyncMutex);
    
    Serial.printf("Synchronized timestamp with PC agent: %lld ns\n", synced_time_ns);
  } 
  else {
      Serial.println("Error in sync_timer_callback: time not synchronized");
      portENTER_CRITICAL(&timeSyncMutex);
      time_synchronized = false;
      portEXIT_CRITICAL(&timeSyncMutex);
  }
}
