/*******************************************************************************
 * Copyright 2016 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho, Valentin Starlinger */

#ifndef TURTLEBOT3_MECANUM_CORE_CONFIG_H_
#define TURTLEBOT3_MECANUM_CORE_CONFIG_H_

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/Sound.h>
#include <turtlebot3_msgs/VersionInfo.h>

#include <TurtleBot3.h>

#include <math.h>

#include "turtlebot3_mecanum_core_motor_driver.h"

#define INIT_LOG_DATA "This core is adjusted for direct command drive."

#define HARDWARE_VER "1.0.0"
#define SOFTWARE_VER "1.0.0"
#define FIRMWARE_VER "1.0.15"

#define CONTROL_MOTOR_SPEED_PERIOD 30         // hz
#define CONTROL_MOTOR_TIMEOUT 100             // ms
#define IMU_PUBLISH_PERIOD 100                // hz
#define CMD_VEL_PUBLISH_PERIOD 30             // hz
#define DRIVE_INFORMATION_PUBLISH_PERIOD 30   // hz
#define VERSION_INFORMATION_PUBLISH_PERIOD 1  // hz

#define NUM_WHEELS 4
#define ENCODER_MIN -2147483648  // raw
#define ENCODER_MAX 2147483648   // raw

#define MAX_WHEEL_VEL 300  // 0.229 rev/60s

#define TICK2RAD 0.001533981            // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
#define RAD2VEL 41.6999850896           //
#define DEG2RAD(x) (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x) (x * 57.2957795131)  // *180/PI

// Callback function prototypes
void commandJointStateCallback(const sensor_msgs::JointState& cmd_msg);
void motorPowerCallback(const std_msgs::Bool& power_msg);
void resetCallback(const std_msgs::Empty& reset_msg);

// Function prototypes
void publishDriveInformation(void);

ros::Time rosNow(void);
ros::Time addMicros(ros::Time& t, uint32_t _micros);

void updateMotorInfo();
void updateTime(void);

void initJointStates(void);

void sendLogMsg(void);

/*****
 * Kinetmatik stuff
 * Motor ids: front left, front right, rear left, rear right
 **** */

int32_t motor_ids_[NUM_WHEELS] = { 1, 2, 3, 4 };
char* joint_states_name[NUM_WHEELS] = {
  "wheel_front_left_joint",
  "wheel_rear_left_joint",
  "wheel_rear_right_joint",
  "wheel_front_right_joint",
};

/*******************************************************************************
 * ROS NodeHandle
 *******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
 * Subscriber
 *******************************************************************************/
ros::Subscriber<sensor_msgs::JointState> cmd_js_sub_("wheel_command", commandJointStateCallback);

ros::Subscriber<std_msgs::Bool> motor_power_sub("motor_power", motorPowerCallback);

ros::Subscriber<std_msgs::Empty> reset_sub("reset", resetCallback);

/*******************************************************************************
 * Publisher
 *******************************************************************************/

std_msgs::Byte status_byte_msg_;
ros::Publisher status_byte_pub_("status_byte", &status_byte_msg_);

// Joint(Dynamixel) state of Turtlebot3
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("wheel_state", &joint_states);

/*******************************************************************************
 * SoftwareTimer of Turtlebot3
 *******************************************************************************/
static uint32_t tTime[6];

/*******************************************************************************
 * Declaration for motor
 *******************************************************************************/
Turtlebot3MecanumCoreMotorDriver motor_driver(motor_ids_[0], motor_ids_[1], motor_ids_[2], motor_ids_[3]);

/*******************************************************************************
 * Calculation for odometry
 *******************************************************************************/
bool init_encoder;
int32_t last_tick[NUM_WHEELS] = { 0.0, 0.0, 0.0, 0.0 };
int32_t last_diff_tick[NUM_WHEELS] = { 0.0, 0.0, 0.0, 0.0 };
double last_rad[NUM_WHEELS] = { 0.0, 0.0, 0.0, 0.0 };

double target_velocity[NUM_WHEELS] = { 0.0, 0.0, 0.0, 0.0 };

/*******************************************************************************
 * Update Joint State
 *******************************************************************************/
double last_velocity[NUM_WHEELS] = { 0.0, 0.0, 0.0, 0.0 };

/*******************************************************************************
 * Declaration for sensors
 *******************************************************************************/
// Turtlebot3Sensor sensors;

/*******************************************************************************
 * Declaration for controllers
 *******************************************************************************/
// Turtlebot3Controller controllers;
float goal_velocity[3] = { 0.0, 0.0, 0.0 };

/*******************************************************************************
 * Declaration for diagnosis
 *******************************************************************************/
// Turtlebot3Diagnosis diagnosis;

/*******************************************************************************
 * Declaration for SLAM and navigation
 *******************************************************************************/
unsigned long prev_update_time;

/*******************************************************************************
 * Declaration for Battery
 *******************************************************************************/
bool setup_end = false;
uint8_t battery_state = 0;

#endif  // TURTLEBOT3_MECANUM_CORE_CONFIG_H_
