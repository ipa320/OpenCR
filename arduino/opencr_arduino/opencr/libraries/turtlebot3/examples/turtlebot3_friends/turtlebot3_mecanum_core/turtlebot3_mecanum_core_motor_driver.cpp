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

#include "turtlebot3_mecanum_core_motor_driver.h"

Turtlebot3MecanumCoreMotorDriver::Turtlebot3MecanumCoreMotorDriver()
  : baudrate_(BAUDRATE)
  , protocol_version_(PROTOCOL_VERSION)
  , left_front_wheel_id_(DXL_LEFT_FRONT_ID)
  , right_front_wheel_id_(DXL_RIGHT_FRONT_ID)
  , left_rear_wheel_id_(DXL_LEFT_REAR_ID)
  , right_rear_wheel_id_(DXL_RIGHT_REAR_ID)
  , torque_(false)
{
}

Turtlebot3MecanumCoreMotorDriver::Turtlebot3MecanumCoreMotorDriver(uint8_t left_front_motor_id,
                                                                   uint8_t right_front_motor_id,
                                                                   uint8_t left_rear_motor_id,
                                                                   uint8_t right_rear_motor_id)
  : baudrate_(BAUDRATE)
  , protocol_version_(PROTOCOL_VERSION)
  , left_front_wheel_id_(left_front_motor_id)
  , right_front_wheel_id_(right_front_motor_id)
  , left_rear_wheel_id_(left_rear_motor_id)
  , right_rear_wheel_id_(right_rear_motor_id)
  , torque_(false)
{
}

Turtlebot3MecanumCoreMotorDriver::~Turtlebot3MecanumCoreMotorDriver()
{
  closeDynamixel();
}

bool Turtlebot3MecanumCoreMotorDriver::init(void)
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort() == false)
  {
    return false;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_) == false)
  {
    return false;
  }

  // Enable Dynamixel Torque
  setTorque(left_rear_wheel_id_, true);
  setTorque(right_rear_wheel_id_, true);
  setTorque(left_front_wheel_id_, true);
  setTorque(right_front_wheel_id_, true);

  groupSyncWriteVelocity_ =
      new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncReadEncoder_ =
      new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  return true;
}

bool Turtlebot3MecanumCoreMotorDriver::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  else if (dxl_error != 0)
  {
    Serial.println(packetHandler_->getRxPacketError(dxl_error));
    return false;
  }

  torque_ = onoff;
  return true;
}

bool Turtlebot3MecanumCoreMotorDriver::getTorque()
{
  return torque_;
}

void Turtlebot3MecanumCoreMotorDriver::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(left_rear_wheel_id_, false);
  setTorque(right_rear_wheel_id_, false);
  setTorque(left_front_wheel_id_, false);
  setTorque(right_front_wheel_id_, false);

  // Close port
  portHandler_->closePort();
}

bool Turtlebot3MecanumCoreMotorDriver::readEncoder(int32_t& left_front_value,
                                                   int32_t& right_front_value,
                                                   int32_t& left_rear_value,
                                                   int32_t& right_rear_value)
{
  int dxl_comm_result = COMM_TX_FAIL;  // Communication result
  bool dxl_addparam_result = false;    // addParam result
  bool dxl_getdata_result = false;     // GetParam result

  // Set parameter
  dxl_addparam_result = groupSyncReadEncoder_->addParam(left_rear_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncReadEncoder_->addParam(right_rear_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncReadEncoder_->addParam(left_front_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  dxl_addparam_result = groupSyncReadEncoder_->addParam(right_front_wheel_id_);
  if (dxl_addparam_result != true)
    return false;

  // Syncread present position
  dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

  // Check if groupSyncRead data of Dynamixels are available
  dxl_getdata_result =
      groupSyncReadEncoder_->isAvailable(left_rear_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  dxl_getdata_result =
      groupSyncReadEncoder_->isAvailable(right_rear_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  dxl_getdata_result =
      groupSyncReadEncoder_->isAvailable(left_front_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  dxl_getdata_result =
      groupSyncReadEncoder_->isAvailable(right_front_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  if (dxl_getdata_result != true)
    return false;

  // Get data
  left_rear_value =
      groupSyncReadEncoder_->getData(left_rear_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  right_rear_value =
      groupSyncReadEncoder_->getData(right_rear_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  left_front_value =
      groupSyncReadEncoder_->getData(left_front_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  right_front_value =
      groupSyncReadEncoder_->getData(right_front_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

  groupSyncReadEncoder_->clearParam();
  return true;
}

bool Turtlebot3MecanumCoreMotorDriver::writeVelocity(int64_t left_front_value,
                                                     int64_t right_front_value,
                                                     int64_t left_rear_value,
                                                     int64_t right_rear_value)
{
  bool dxl_addparam_result;
  int8_t dxl_comm_result;

  int64_t value[4] = { left_front_value, right_front_value, left_rear_value, right_rear_value };
  uint8_t data_byte[4] = {
    0,
  };

  for (uint8_t index = 0; index < 4; index++)
  {
    data_byte[0] = DXL_LOBYTE(DXL_LOWORD(value[index]));
    data_byte[1] = DXL_HIBYTE(DXL_LOWORD(value[index]));
    data_byte[2] = DXL_LOBYTE(DXL_HIWORD(value[index]));
    data_byte[3] = DXL_HIBYTE(DXL_HIWORD(value[index]));

    dxl_addparam_result = groupSyncWriteVelocity_->addParam(index + 1, (uint8_t*)&data_byte);
    if (dxl_addparam_result != true)
      return false;
  }

  dxl_comm_result = groupSyncWriteVelocity_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }

  groupSyncWriteVelocity_->clearParam();
  return true;
}

bool Turtlebot3MecanumCoreMotorDriver::controlMotor(const float r,
                                                    const float wheel_separation_x,
                                                    const float wheel_separation_y,
                                                    float* value)
{
  bool dxl_comm_result = false;

  float wheel_velocity_rad_per_s[4];
  float goal_velocity_0_229_rpm[4];  // unit for dynamixels is 0.229 rpm

  float lin_x_vel = value[0];
  float lin_y_vel = value[1];
  float ang_vel = value[2];

  wheel_velocity_rad_per_s[LEFT_FRONT] =
      (1 / r) * (lin_x_vel - lin_y_vel - (wheel_separation_x + wheel_separation_y) * ang_vel);
  wheel_velocity_rad_per_s[RIGHT_FRONT] =
      (-1) * (1 / r) * (lin_x_vel + lin_y_vel + (wheel_separation_x + wheel_separation_y) * ang_vel);
  wheel_velocity_rad_per_s[LEFT_REAR] =
      (1 / r) * (lin_x_vel + lin_y_vel - (wheel_separation_x + wheel_separation_y) * ang_vel);
  wheel_velocity_rad_per_s[RIGHT_REAR] =
      (-1) * (1 / r) * (lin_x_vel - lin_y_vel + (wheel_separation_x + wheel_separation_y) * ang_vel);

  for (int id = 0; id < 4; id++)
  {
    // calculate 0.229 rmp from rad/s
    goal_velocity_0_229_rpm[id] = wheel_velocity_rad_per_s[id] * 60 / (2 * PI * r * 0.229);

    if (goal_velocity_0_229_rpm[id] > LIMIT_X_MAX_VELOCITY)
    {
      goal_velocity_0_229_rpm[id] = LIMIT_X_MAX_VELOCITY;
    }
    else if (goal_velocity_0_229_rpm[id] < -LIMIT_X_MAX_VELOCITY)
    {
      goal_velocity_0_229_rpm[id] = -LIMIT_X_MAX_VELOCITY;
    }
  }

  dxl_comm_result = writeVelocity((int64_t)goal_velocity_0_229_rpm[LEFT_FRONT],
                                  (int64_t)goal_velocity_0_229_rpm[RIGHT_FRONT],
                                  (int64_t)goal_velocity_0_229_rpm[LEFT_REAR],
                                  (int64_t)goal_velocity_0_229_rpm[RIGHT_REAR]);
  if (dxl_comm_result == false)
    return false;

  return true;
}
