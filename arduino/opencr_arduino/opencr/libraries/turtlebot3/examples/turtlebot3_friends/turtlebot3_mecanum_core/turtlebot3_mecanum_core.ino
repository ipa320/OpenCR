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

#include "turtlebot3_mecanum_core_config.h"

/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(sound_sub);
  nh.subscribe(motor_power_sub);
  nh.subscribe(reset_sub);
  nh.advertise(sensor_state_pub);  
  nh.advertise(version_info_pub);
  nh.advertise(imu_pub);
  nh.advertise(cmd_vel_rc100_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  nh.advertise(battery_state_pub);
  nh.advertise(mag_pub);
  tf_broadcaster.init(nh);

  // Setting for Dynamixel motors
  motor_driver.init();

  // Setting for IMU
  sensors.init();

  diagnosis.init();

  // Setting for ROBOTIS RC100 remote controller and cmd_vel
  controllers.init(MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY);

  // Setting for SLAM and navigation (odometry, joint states, TF)
  initOdom();

  initJointStates();

  prev_update_time = millis();

  pinMode(LED_WORKING_CHECK, OUTPUT);

  SerialBT2.begin(57600);

  setup_end = true;
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{
  uint32_t t = millis();
  updateTime();
  updateVariable();

  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD))
  {
    motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION_X, WHEEL_SEPARATION_Y, goal_velocity);
    tTime[0] = t;
  }

  if ((t-tTime[1]) >= (1000 / CMD_VEL_PUBLISH_PERIOD))
  {
    cmd_vel_rc100_pub.publish(&cmd_vel_rc100_msg);
    tTime[1] = t;
  }

  if ((t-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_PERIOD))
  {
    publishSensorStateMsg();
    publishBatteryStateMsg();
    publishDriveInformation();
    tTime[2] = t;
  }

  if ((t-tTime[3]) >= (1000 / IMU_PUBLISH_PERIOD))
  {
    publishImuMsg();
    publishMagMsg();
    tTime[3] = t;
  }

  if ((t-tTime[4]) >= (1000 / VERSION_INFORMATION_PUBLISH_PERIOD))
  {
    publishVersionInfoMsg();
    tTime[4] = t;
  }

  // Send log message after ROS connection
  sendLogMsg();

  // Receive data from RC100 
  controllers.getRCdata(goal_velocity);

  // Update the IMU unit
  sensors.updateIMU();

  // Start Gyro Calibration after ROS connection
  updateGyroCali();

  // Show LED status
  diagnosis.showLedStatus(nh.connected());

  // Update Voltage
  battery_state = diagnosis.updateVoltageCheck(setup_end);

  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();

  // give the serial link time to process
  delay(10);
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{  
  goal_velocity[LINEAR_X]  = cmd_vel_msg.linear.x;
  goal_velocity[LINEAR_Y]  = cmd_vel_msg.linear.y;
  goal_velocity[ANGULAR] = cmd_vel_msg.angular.z;
}

/*******************************************************************************
* Callback function for sound msg
*******************************************************************************/
void soundCallback(const turtlebot3_msgs::Sound& sound_msg)
{  
  const uint16_t NOTE_C4 = 262;
  const uint16_t NOTE_D4 = 294;
  const uint16_t NOTE_E4 = 330;
  const uint16_t NOTE_F4 = 349;
  const uint16_t NOTE_G4 = 392;
  const uint16_t NOTE_A4 = 440;
  const uint16_t NOTE_B4 = 494;
  const uint16_t NOTE_C5 = 523;
  const uint16_t NOTE_C6 = 1047;

  const uint8_t OFF         = 0;
  const uint8_t ON          = 1;
  const uint8_t LOW_BATTERY = 2;
  const uint8_t ERROR       = 3;
  const uint8_t BUTTON1     = 4;
  const uint8_t BUTTON2     = 5;

  uint16_t note[8]     = {0, 0};
  uint8_t  duration[8] = {0, 0};

  switch (sound_msg.value)
  {
    case ON:
      note[0] = NOTE_C4;   duration[0] = 4;
      note[1] = NOTE_D4;   duration[1] = 4;
      note[2] = NOTE_E4;   duration[2] = 4;
      note[3] = NOTE_F4;   duration[3] = 4;
      note[4] = NOTE_G4;   duration[4] = 4;
      note[5] = NOTE_A4;   duration[5] = 4;
      note[6] = NOTE_B4;   duration[6] = 4;
      note[7] = NOTE_C5;   duration[7] = 4;   
     break;

    case OFF:
      note[0] = NOTE_C5;   duration[0] = 4;
      note[1] = NOTE_B4;   duration[1] = 4;
      note[2] = NOTE_A4;   duration[2] = 4;
      note[3] = NOTE_G4;   duration[3] = 4;
      note[4] = NOTE_F4;   duration[4] = 4;
      note[5] = NOTE_E4;   duration[5] = 4;
      note[6] = NOTE_D4;   duration[6] = 4;
      note[7] = NOTE_C4;   duration[7] = 4;  
     break;

    case LOW_BATTERY:
      note[0] = 1000;      duration[0] = 1;
      note[1] = 1000;      duration[1] = 1;
      note[2] = 1000;      duration[2] = 1;
      note[3] = 1000;      duration[3] = 1;
      note[4] = 0;         duration[4] = 8;
      note[5] = 0;         duration[5] = 8;
      note[6] = 0;         duration[6] = 8;
      note[7] = 0;         duration[7] = 8;
     break;

    case ERROR:
      note[0] = 1000;      duration[0] = 3;
      note[1] = 500;       duration[1] = 3;
      note[2] = 1000;      duration[2] = 3;
      note[3] = 500;       duration[3] = 3;
      note[4] = 1000;      duration[4] = 3;
      note[5] = 500;       duration[5] = 3;
      note[6] = 1000;      duration[6] = 3;
      note[7] = 500;       duration[7] = 3;
     break;

    case BUTTON1:
     break;

    case BUTTON2:
     break;

    default:
      note[0] = NOTE_C4;   duration[0] = 4;
      note[1] = NOTE_D4;   duration[1] = 4;
      note[2] = NOTE_E4;   duration[2] = 4;
      note[3] = NOTE_F4;   duration[3] = 4;
      note[4] = NOTE_G4;   duration[4] = 4;
      note[5] = NOTE_A4;   duration[5] = 4;
      note[6] = NOTE_B4;   duration[6] = 4;
      note[7] = NOTE_C4;   duration[7] = 4; 
     break;
  }

  melody(note, 8, duration);
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

  sensors.calibrationGyro();

  sprintf(log_msg, "Calibration End");
  nh.loginfo(log_msg);

  initOdom();

  sprintf(log_msg, "Reset Odometry");
  nh.loginfo(log_msg);  
}

/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg = sensors.getIMU();

  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = "imu_link";

  imu_pub.publish(&imu_msg);
}

/*******************************************************************************
* Publish msgs (Magnetic data)
*******************************************************************************/
void publishMagMsg(void)
{
  mag_msg = sensors.getMag();

  mag_msg.header.stamp    = rosNow();
  mag_msg.header.frame_id = "mag_link";

  mag_pub.publish(&mag_msg);
}

/*******************************************************************************
* Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
*******************************************************************************/
void publishSensorStateMsg(void)
{
  bool dxl_comm_result = false;

  sensor_state_msg.header.stamp = rosNow();
  sensor_state_msg.battery = sensors.checkVoltage();

  //dxl_comm_result = motor_driver.readEncoder(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);

  if (dxl_comm_result == true){
    sensor_state_msg.left_encoder = 0;
    sensor_state_msg.right_encoder = 0;
    updateMotorInfo(0,0,0,0);//sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);
  } else{
    return;
  }

  sensor_state_msg.button = sensors.checkPushButton();

  sensor_state_msg.torque = motor_driver.getTorque();

  sensor_state_pub.publish(&sensor_state_msg);
}

/*******************************************************************************
* Publish msgs (version info)
*******************************************************************************/
void publishVersionInfoMsg(void)
{
  version_info_msg.hardware = HARDWARE_VER;
  version_info_msg.software = SOFTWARE_VER;
  version_info_msg.firmware = FIRMWARE_VER;

  version_info_pub.publish(&version_info_msg);
}

/*******************************************************************************
* Publish msgs (battery_state)
*******************************************************************************/
void publishBatteryStateMsg(void)
{
  battery_state_msg.header.stamp = rosNow();
  battery_state_msg.design_capacity = 1.8f; //Ah
  battery_state_msg.voltage = sensors.checkVoltage();
  battery_state_msg.percentage = (float)(battery_state_msg.voltage / 11.1f);

  if (battery_state == 0)
    battery_state_msg.present = false;
  else
    battery_state_msg.present = true;  

  battery_state_pub.publish(&battery_state_msg);
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

  // calculate odometry
  calcOdometry((double)(step_time * 0.001));

  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);

  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);
}

/*******************************************************************************
* Update the odometry
*******************************************************************************/
void updateOdometry(void)
{
  odom.header.frame_id = "odom_combined";
  odom.child_frame_id  = "base_link";

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.linear.y  = odom_vel[1];
  odom.twist.twist.angular.z = odom_vel[2];
}

/*******************************************************************************
* Update the joint states 
*******************************************************************************/
void updateJointStates(void)
{
  static float joint_states_pos[NUM_WHEELS] = {0.0, 0.0, 0.0, 0.0};
  static float joint_states_vel[NUM_WHEELS] = {0.0, 0.0, 0.0, 0.0};
  static float joint_states_eff[NUM_WHEELS] = {0.0, 0.0, 0.0, 0.0};

  joint_states_pos[LEFT_FRONT]  = last_rad[LEFT_FRONT];
  joint_states_pos[RIGHT_FRONT] = last_rad[RIGHT_FRONT];
  joint_states_pos[LEFT_REAR]  = last_rad[LEFT_REAR];
  joint_states_pos[RIGHT_REAR] = last_rad[RIGHT_REAR];

  joint_states_vel[LEFT_FRONT]  = last_velocity[LEFT_FRONT];
  joint_states_vel[RIGHT_FRONT] = last_velocity[RIGHT_FRONT];
  joint_states_vel[LEFT_REAR]  = last_velocity[LEFT_REAR];
  joint_states_vel[RIGHT_REAR] = last_velocity[RIGHT_REAR];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}

/*******************************************************************************
* Updateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{

  odom_tf.header = odom.header;
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

/*******************************************************************************
* Update motor information
*******************************************************************************/
void updateMotorInfo(int32_t left_front_tick, int32_t right_front_tick, int32_t left_rear_tick, int32_t right_rear_tick)
{
  int32_t current_tick[NUM_WHEELS] = {0.0, 0.0, 0.0, 0.0};
  static int32_t last_tick[NUM_WHEELS] = {0.0, 0.0, 0.0, 0.0};

  if (init_encoder)
  {
    for (int index = 0; index < NUM_WHEELS; index++)
    {
      last_diff_tick[index] = 0.0;
      last_tick[index]      = 0.0;
      last_rad[index]       = 0.0;

      last_velocity[index]  = 0.0;
    }

    last_tick[LEFT_FRONT] = left_front_tick;
    last_tick[RIGHT_FRONT] = right_front_tick;
    last_tick[LEFT_REAR] = left_rear_tick;
    last_tick[RIGHT_REAR] = right_rear_tick;

    init_encoder = false;
    return;
  }

  current_tick[LEFT_FRONT] = left_front_tick;
  current_tick[RIGHT_FRONT] = right_front_tick;
  current_tick[LEFT_REAR] = left_rear_tick;
  current_tick[RIGHT_REAR] = right_rear_tick;

  last_diff_tick[LEFT_FRONT] = current_tick[LEFT_FRONT] - last_tick[LEFT_FRONT];
  last_tick[LEFT_FRONT]      = current_tick[LEFT_FRONT];
  last_rad[LEFT_FRONT]       += TICK2RAD * (double)last_diff_tick[LEFT_FRONT];

  last_diff_tick[RIGHT_FRONT] = current_tick[RIGHT_FRONT] - last_tick[RIGHT_FRONT];
  last_tick[RIGHT_FRONT]      = current_tick[RIGHT_FRONT];
  last_rad[RIGHT_FRONT]       += TICK2RAD * (double)last_diff_tick[RIGHT_FRONT];

  last_diff_tick[LEFT_REAR] = current_tick[LEFT_REAR] - last_tick[LEFT_REAR];
  last_tick[LEFT_REAR]      = current_tick[LEFT_REAR];
  last_rad[LEFT_REAR]       += TICK2RAD * (double)last_diff_tick[LEFT_REAR];

  last_diff_tick[RIGHT_REAR] = current_tick[RIGHT_REAR] - last_tick[RIGHT_REAR];
  last_tick[RIGHT_REAR]      = current_tick[RIGHT_REAR];
  last_rad[RIGHT_REAR]       += TICK2RAD * (double)last_diff_tick[RIGHT_REAR];
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  float* orientation;
  double wheel_l_f_radps, wheel_r_f_radps, wheel_l_r_radps, wheel_r_r_radps;      // rotation value of wheel [rad]
  double delta_x, delta_y, theta, delta_theta;
  static double last_theta = 0.0;
  double vx, vy, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l_f_radps = wheel_r_f_radps = wheel_l_r_radps = wheel_r_r_radps = 0.0;
  delta_x = delta_y = delta_theta = theta = 0.0;
  vx = vy = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l_f_radps = (TICK2RAD * (double)last_diff_tick[LEFT_FRONT])/step_time;
  wheel_r_f_radps = (TICK2RAD * (double)last_diff_tick[RIGHT_FRONT])/step_time;
  wheel_l_r_radps = (TICK2RAD * (double)last_diff_tick[LEFT_REAR])/step_time;
  wheel_r_r_radps = (TICK2RAD * (double)last_diff_tick[RIGHT_REAR])/step_time;

  
  if (isnan(wheel_l_f_radps))
    wheel_l_f_radps = 0.0;
  if (isnan(wheel_r_f_radps))
    wheel_r_f_radps = 0.0;
  if (isnan(wheel_l_r_radps))
    wheel_l_r_radps = 0.0;
  if (isnan(wheel_r_r_radps))
    wheel_r_r_radps = 0.0;

  
  //should be + + + + for vx but because two motors are upside-down it is + - + - 
  //same for the other velocities as well
  vx = (wheel_l_f_radps - wheel_r_f_radps + wheel_l_r_radps - wheel_r_r_radps) * (WHEEL_RADIUS / 4);
  vy = ( -wheel_l_f_radps - wheel_r_f_radps + wheel_l_r_radps + wheel_r_r_radps) * (WHEEL_RADIUS / 4);

  w = ( -wheel_l_f_radps - wheel_r_f_radps - wheel_l_r_radps - wheel_r_r_radps) * (WHEEL_RADIUS/(4 * (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y))) ;
  
  /*orientation = sensors.getOrientation();
  theta       = atan2f(orientation[1] * orientation[2] + orientation[0] * orientation[3],
                       0.5f - orientation[2] * orientation[2] - orientation[3] * orientation[3]);
*/
  delta_theta = w * step_time;
  odom_pose[2] += delta_theta/2.0;

  delta_x = (vx * cos(odom_pose[2]) - vy * sin(odom_pose[2])) * step_time;
  delta_y = (vx * sin(odom_pose[2]) + vy * cos(odom_pose[2])) * step_time;
  //w = delta_theta / step_time;
  
  odom_pose[0] += delta_x/2.0;
  odom_pose[1] += delta_y/2.0;

  // compute odometric instantaneouse velocity
  odom_vel[0] = vx;
  odom_vel[1] = vy;
  odom_vel[2] = w;

  //last_theta = theta;
  last_velocity[LEFT_FRONT]  = wheel_l_f_radps;
  last_velocity[RIGHT_FRONT] = wheel_r_f_radps;
  last_velocity[LEFT_REAR]  = wheel_l_r_radps;
  last_velocity[RIGHT_REAR] = wheel_r_r_radps;

  return true;
}

/*******************************************************************************
* Update variable (initialization)
*******************************************************************************/
void updateVariable(void)
{
  static bool variable_flag = false;

  if (nh.connected())
  {
    if (variable_flag == false)
    {      
      sensors.initIMU();
      initOdom();

      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
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
ros::Time addMicros(ros::Time & t, uint32_t _micros)
{
  uint32_t sec, nsec;

  sec  = _micros / 1000000 + t.sec;
  nsec = _micros % 1000000 + 1000 * (t.nsec / 1000);
  
  if (nsec >= 1e9) 
  {
    sec++, nsec--;
  }
  return ros::Time(sec, nsec);
}

/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(void)
{
  static bool isEnded = false;
  char log_msg[50];

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      sensors.calibrationGyro();

      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
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

void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;
}

void initJointStates(void)
{
  static char *joint_states_name[] = {"wheel_left_front_joint", "wheel_right_front_joint", "wheel_right_rear_joint", "wheel_left_rear_joint"};

  joint_states.header.frame_id = "base_link";
  joint_states.name            = joint_states_name;

  joint_states.name_length     = NUM_WHEELS;
  joint_states.position_length = NUM_WHEELS;
  joint_states.velocity_length = NUM_WHEELS;
  joint_states.effort_length   = NUM_WHEELS;
}

void melody(uint16_t* note, uint8_t note_num, uint8_t* durations)
{
  for (int thisNote = 0; thisNote < note_num; thisNote++) 
  {
    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / durations[thisNote];
    tone(BDPIN_BUZZER, note[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(BDPIN_BUZZER);
  }
}
