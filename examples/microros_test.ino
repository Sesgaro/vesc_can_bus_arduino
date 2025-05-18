#include <mcp_can.h>
#include "vesc_can_bus_arduino.h"
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Core definitions for multitasking
#define CORE_0 0
#define CORE_1 1

// CAN bus setup
MCP_CAN CAN0(10); // CS pin

// VESC drivers
VescCAN driver1(CAN0, 10);    // ID 10
VescCAN driver2(CAN0, 11);    // ID 11
VescCAN* drivers[] = {&driver1, &driver2};
const int numdrivers = 2;

// Shared variables for RPM
int targetRpms[2] = {0, 0}; // Target RPMs for each driver
SemaphoreHandle_t rpmMutex; // Mutex to protect targetRpms
unsigned long last_can_update = 0; // For periodic CAN updates
const unsigned long CAN_UPDATE_INTERVAL = 10; // Interval in ms for VESC updates
unsigned long last_publish_time = 0; // For controlling publish frequency
const unsigned long PUBLISH_INTERVAL = 10; // 100 Hz
unsigned long last_target_update = 0; // Last update of target_rpms
const unsigned long TARGET_TIMEOUT = 100; // Timeout in ms to consider target as not received

// ROS 2 node and handles
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

// Publishers and subscribers
rcl_publisher_t read_rpms_publisher;
rcl_subscription_t target_rpms_subscriber;
std_msgs__msg__Float32MultiArray read_rpms_msg;
std_msgs__msg__Float32MultiArray target_rpms_msg;

// Task handles
TaskHandle_t microRosTaskHandle;
TaskHandle_t arduinoTaskHandle;

// Error handling macro
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.printf("Micro-ROS error in %s: %d\n", #fn, temp_rc);}}
#define EXECUTE(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) {Serial.printf("Executor error in spin_some: %d\n", rc);}}

// Callback for receiving target RPMs
void target_rpms_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    if (msg->data.size == numdrivers) {
        if (xSemaphoreTake(rpmMutex, portMAX_DELAY) == pdTRUE) {
            for (int i = 0; i < numdrivers; i++) {
                targetRpms[i] = (int)(msg->data.data[i] * 7); // Convert RPM to ERPM (approx. 1 RPM = 7 ERPM)
            }
            last_target_update = millis();
            Serial.printf("Received target_rpms: [%d, %d]\n", targetRpms[0], targetRpms[1]);
            xSemaphoreGive(rpmMutex);
        }
    } else {
        Serial.printf("Error: target_rpms message with incorrect size: %zu\n", msg->data.size);
    }
}

// Task for Micro-ROS (Core 0)
void microRosTask(void *pvParameters)
{
    while (1) {
        if (millis() - last_publish_time >= PUBLISH_INTERVAL) {
            // Publish current RPMs to the "read_rpms" topic
            if (xSemaphoreTake(rpmMutex, portMAX_DELAY) == pdTRUE) {
                for (int i = 0; i < numdrivers; i++) {
                    read_rpms_msg.data.data[i] = drivers[i]->erpm / 7; // Convert ERPM back to RPM
                }
                xSemaphoreGive(rpmMutex);
            }
            RCCHECK(rcl_publish(&read_rpms_publisher, &read_rpms_msg, NULL));
            last_publish_time = millis();
        }

        // Execute the executor with a short timeout for responsiveness
        EXECUTE(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
        vTaskDelay(1); // Yield to avoid watchdog
    }
}

// Task for Arduino (Core 1)
void arduinoTask(void *pvParameters)
{
    while (1) {
        // Update VESC state with error handling
        if (CAN0.checkReceive() == CAN_MSGAVAIL) {
            long unsigned int rxId;
            unsigned char len = 0;
            unsigned char rxBuf[8];
            if (CAN0.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
                for (int i = 0; i < numdrivers; i++) {
                    drivers[i]->processMessage(rxId, rxBuf);
                }
            } else {
                Serial.println("Error reading CAN message");
            }
        }

        // Periodically send speed commands to the VESCs
        if (millis() - last_can_update >= CAN_UPDATE_INTERVAL) {
            if (xSemaphoreTake(rpmMutex, portMAX_DELAY) == pdTRUE) {
                for (int i = 0; i < numdrivers; i++) {
                    drivers[i]->vesc_set_erpm(targetRpms[i]);
                }
                xSemaphoreGive(rpmMutex);
            }
            last_can_update = millis();
        }

        vTaskDelay(1); // Yield to avoid watchdog
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) delay(10); // Wait for Serial to be ready

    // Initialize CAN bus
    CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ); // Set the CAN bus speed and clock frequency
    CAN0.setMode(MCP_NORMAL);

    // Initialize Micro-ROS
    set_microros_serial_transports(Serial);
    delay(1000); // Allow time for the agent to connect

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "vesc_control_node", "", &support));

    // Configure publisher for read_rpms
    RCCHECK(rclc_publisher_init_default(
        &read_rpms_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/read_rpms"));
    read_rpms_msg.data.capacity = numdrivers;
    read_rpms_msg.data.size = numdrivers;
    read_rpms_msg.data.data = (float*)malloc(numdrivers * sizeof(float));

    // Configure subscriber for target_rpms
    RCCHECK(rclc_subscription_init_default(
        &target_rpms_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/target_rpms"));
    target_rpms_msg.data.capacity = numdrivers;
    target_rpms_msg.data.size = 0;
    target_rpms_msg.data.data = (float*)malloc(target_rpms_msg.data.capacity * sizeof(float));

    // Configure executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &target_rpms_subscriber, &target_rpms_msg, &target_rpms_callback, ON_NEW_DATA));

    // Create mutex
    rpmMutex = xSemaphoreCreateMutex();

    // Create tasks for the cores with adjusted priority
    xTaskCreatePinnedToCore(
        microRosTask,
        "MicroRosTask",
        10000,
        NULL,
        2, // Higher priority for Micro-ROS
        &microRosTaskHandle,
        CORE_0);

    xTaskCreatePinnedToCore(
        arduinoTask,
        "ArduinoTask",
        10000,
        NULL,
        1,
        &arduinoTaskHandle,
        CORE_1);

    Serial.println("Micro-ROS initialized. Connect the agent on /dev/ttyUSB0.");
}

void loop(){}
