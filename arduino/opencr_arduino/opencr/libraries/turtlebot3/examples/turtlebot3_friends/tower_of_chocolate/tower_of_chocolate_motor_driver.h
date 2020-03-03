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

#ifndef CHOCOLATE_WB_MOTOR_DRIVER_H_
#define CHOCOLATE_WB_MOTOR_DRIVER_H_

#include <DynamixelWorkbench.h>

#define DEVICE_NAME ""

// Control table address (Dynamixel X-series)
#define ADDR_X_TORQUE_ENABLE 64
#define ADDR_X_GOAL_VELOCITY 104
#define ADDR_X_GOAL_POSITION 116
#define ADDR_X_REALTIME_TICK 120
#define ADDR_X_PRESENT_VELOCITY 128
#define ADDR_X_PRESENT_POSITION 132
#define ADDR_X_OPERATING_MODE 11

// Limit values (XM430-W210-T)
#define LIMIT_X_MAX_VELOCITY 240

// Data Byte Length
#define LEN_X_TORQUE_ENABLE 1
#define LEN_X_GOAL_VELOCITY 4
#define LEN_X_GOAL_POSITION 4
#define LEN_X_REALTIME_TICK 2
#define LEN_X_PRESENT_VELOCITY 4
#define LEN_X_PRESENT_POSITION 4

#define OPERATION_MODE_VELOCITY 1
#define OPERATION_MODE_POSITION 3

//
#define ONE_REVOLUTION 4096

#define PROTOCOL_VERSION 2.0  // Dynamixel protocol version 2.0

#define BAUDRATE 57600  // baurd rate of Dynamixel
#define DEVICENAME ""   // no need setting on OpenCR

#define TORQUE_ENABLE 1   // Value for enabling the torque
#define TORQUE_DISABLE 0  // Value for disabling the torque

#define VELOCITY_CONSTANT_VALUE                                                                                        \
  1263.632956882  // V = r * w = r * RPM * 0.10472
                  //   = 0.033 * 0.229 * Goal RPM * 0.10472
                  // Goal RPM = V * 1263.632956882

class TowerOfChocolateMotorDriver
{
public:
  TowerOfChocolateMotorDriver();
  ~TowerOfChocolateMotorDriver();

  bool init(const char** log = NULL);

  bool dropItLikeItsHot(uint8_t dxl_id, const char** log = NULL);

private:
  DynamixelWorkbench dxl_wb_;
};

#endif  // CHOCOLATE_WB_MOTOR_DRIVER_H_
