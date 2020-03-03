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

#include "tower_of_chocolate_motor_driver.h"

TowerOfChocolateMotorDriver::TowerOfChocolateMotorDriver()
{
}

TowerOfChocolateMotorDriver::~TowerOfChocolateMotorDriver()
{
}

bool TowerOfChocolateMotorDriver::init(const char** log)
{
  bool success = false;

  success = dxl_wb_.init(DEVICE_NAME, BAUDRATE, log);
  if (!success)
    return false;

  return true;
}

bool TowerOfChocolateMotorDriver::dropItLikeItsHot(uint8_t dxl_id, const char** log)
{
  bool success = false;
  uint16_t model_number = 0;
  success = dxl_wb_.ping(dxl_id, &model_number, log);
  if (!success)
    return false;

  int32_t mode;
  success = dxl_wb_.readRegister(dxl_id, "Operating_Mode", &mode, log);
  if (!success)
    return false;

  if (mode != 1)
  {
    success = dxl_wb_.setVelocityControlMode(dxl_id, log);
    if (!success)
      return false;
  }

  success = dxl_wb_.torqueOn(dxl_id, log);
  if (!success)
    return false;

  int32_t position;
  int32_t old_position;
  success = dxl_wb_.getPresentPositionData(dxl_id, &position, log);
  position &= 0xFFF;

  if (!success)
    return false;
  success = dxl_wb_.goalVelocity(dxl_id, 400, log);
  if (!success)
    return false;

  do
  {
    delay(10);
    old_position = position;
    success = dxl_wb_.getPresentPositionData(dxl_id, &position, log);
    if (!success)
    {
      dxl_wb_.goalVelocity(dxl_id, 0, log);
      return false;
    }
    position &= 0xFFF;

  } while (position > old_position);

  success = dxl_wb_.goalVelocity(dxl_id, 0, log);
  if (!success)
    return false;

  return true;
}
