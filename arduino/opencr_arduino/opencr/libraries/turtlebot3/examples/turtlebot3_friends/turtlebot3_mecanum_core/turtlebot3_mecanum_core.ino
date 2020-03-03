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

/* Authors: Jannik Abbenseth*/

#include "turtlebot3_mecanum_core_config.h"

/*******************************************************************************
 * Setup function
 *******************************************************************************/
void setup()
{
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  // nh.getHardware()->setBaud(115200);
  nh.subscribe(cmd_js_sub_);
  nh.subscribe(motor_power_sub);
  nh.subscribe(reset_sub);

  nh.advertise(joint_states_pub);
  nh.advertise(status_byte_pub_);

  // Setting for Dynamixel motors
  motor_driver.init();

  initJointStates();

  prev_update_time = millis();

  pinMode(LED_WORKING_CHECK, OUTPUT);
  pinMode(BDPIN_LED_USER_3, OUTPUT);

  // SerialBT2.begin(57600);
  init_encoder = true;
  setup_end = true;
}

/*******************************************************************************
 * Loop function
 *******************************************************************************/
void loop()
{
  uint32_t t = millis();
  updateTime();
  if ((t - tTime[5]) >= CONTROL_MOTOR_TIMEOUT)
  {
    motor_driver.writeVelocity(0, 0, 0, 0);
  }
  else if ((t - tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD))
  {
    motor_driver.writeVelocity(target_velocity[0], target_velocity[1], target_velocity[2], target_velocity[3]);
    tTime[0] = t;
  }

  if ((t - tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_PERIOD))
  {
    publishDriveInformation();
    tTime[2] = t;
  }

  // Send log message after ROS connection
  sendLogMsg();

  // Show LED status
  // diagnosis.showLedStatus(nh.connected());

  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();

  // give the serial link time to process
  delay(10);
}

void motorPowerCallback(const std_msgs::Bool& power_msg)
{
  bool dxl_power = power_msg.data;

  motor_driver.setTorque(DXL_LEFT_FRONT_ID, dxl_power);
  motor_driver.setTorque(DXL_RIGHT_FRONT_ID, dxl_power);
  motor_driver.setTorque(DXL_LEFT_REAR_ID, dxl_power);
  motor_driver.setTorque(DXL_RIGHT_REAR_ID, dxl_power);
}

void resetCallback(const std_msgs::Empty& reset_msg)
{
  char log_msg[50];

  sprintf(log_msg, "Start Calibration of Gyro");
  nh.loginfo(log_msg);

  // sensors.calibrationGyro();

  sprintf(log_msg, "Calibration End");
  nh.loginfo(log_msg);
}

void commandJointStateCallback(const sensor_msgs::JointState& cmd_msg)
{
  bool clamping = false;
  tTime[5] = millis();
  for (unsigned int i = 0; i < NUM_WHEELS; i++)
  {
    target_velocity[i] = cmd_msg.velocity[i] * RAD2VEL;
    if (target_velocity[i] > MAX_WHEEL_VEL)
    {
      target_velocity[i] = MAX_WHEEL_VEL;
      clamping = true;
    }
    else if (target_velocity[i] < -MAX_WHEEL_VEL)
    {
      target_velocity[i] = -MAX_WHEEL_VEL;
      clamping = true;
    }
  }
  digitalWrite(BDPIN_LED_USER_3, !clamping);
}
/*******************************************************************************
 * Publish msgs (odometry, joint states, tf)
 *******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now = rosNow();

  updateMotorInfo();

  status_byte_msg_.data |= 0x02;
  status_byte_pub_.publish(&status_byte_msg_);

  // joint states
  updateJointStates(0.001 * (double)step_time);
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);
}

/*******************************************************************************
 * Update the joint states
 *******************************************************************************/
void updateJointStates(double dt)
{
  static float joint_states_pos[NUM_WHEELS] = { 0.0, 0.0, 0.0, 0.0 };
  static float joint_states_vel[NUM_WHEELS] = { 0.0, 0.0, 0.0, 0.0 };
  static float joint_states_eff[NUM_WHEELS] = { 0.0, 0.0, 0.0, 0.0 };

  for (unsigned int i = 0; i < NUM_WHEELS; i++)
  {
    joint_states_pos[i] = last_rad[i];
    joint_states_vel[i] = (TICK2RAD * (double)last_diff_tick[i]) / dt;
  }
  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}

/*******************************************************************************
 * Update motor information
 *******************************************************************************/
void updateMotorInfo()
{
  int32_t current_tick[NUM_WHEELS] = { 0.0, 0.0, 0.0, 0.0 };
  bool dxl_comm_result = motor_driver.readEncoder(current_tick[0], current_tick[1], current_tick[2], current_tick[3]);
  if (dxl_comm_result)
  {
    digitalWrite(BDPIN_LED_USER_4, HIGH);
    status_byte_msg_.data |= 0x01;
    status_byte_pub_.publish(&status_byte_msg_);
  }
  else
  {
    digitalWrite(BDPIN_LED_USER_4, LOW);
    status_byte_msg_.data &= 0xFE;
    status_byte_pub_.publish(&status_byte_msg_);
    return;
  }

  if (init_encoder)
  {
    for (int index = 0; index < NUM_WHEELS; index++)
    {
      last_tick[index] = current_tick[index];
    }

    init_encoder = false;
  }

  for (unsigned int i = 0; i < NUM_WHEELS; i++)
  {
    last_diff_tick[i] = current_tick[i] - last_tick[i];
    last_tick[i] = current_tick[i];
    last_rad[i] += TICK2RAD * (double)last_diff_tick[i];
  }
}

/*******************************************************************************
 * Update the base time for interpolation
 *******************************************************************************/
void updateTime()
{
  current_offset = micros();
  current_time = nh.now();
}

/*******************************************************************************
 * ros::Time::now() implementation
 *******************************************************************************/
ros::Time rosNow()
{
  return addMicros(current_time, micros() - current_offset);
}

/*******************************************************************************
 * Time Interpolation function
 *******************************************************************************/
ros::Time addMicros(ros::Time& t, uint32_t _micros)
{
  uint32_t sec, nsec;

  sec = _micros / 1000000 + t.sec;
  nsec = _micros % 1000000 + 1000 * (t.nsec / 1000);

  if (nsec >= 1e9)
  {
    sec++, nsec--;
  }
  return ros::Time(sec, nsec);
}

/*******************************************************************************
 * Send log message
 *******************************************************************************/
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[50];
  const char* init_log_data = INIT_LOG_DATA;

  if (nh.connected())
  {
    if (log_flag == false)
    {
      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      sprintf(log_msg, "Connected to OpenCR board!");
      nh.loginfo(log_msg);

      sprintf(log_msg, init_log_data);
      nh.loginfo(log_msg);

      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      log_flag = true;
    }
  }
  else
  {
    log_flag = false;
  }
}

void initJointStates(void)
{
  joint_states.header.frame_id = "base_link";
  joint_states.name = joint_states_name;

  joint_states.name_length = NUM_WHEELS;
  joint_states.position_length = NUM_WHEELS;
  joint_states.velocity_length = NUM_WHEELS;
  joint_states.effort_length = NUM_WHEELS;

  static float efforts[NUM_WHEELS] = { 0.0, 0.0, 0.0, 0.0 };
  joint_states.effort = efforts;
}
