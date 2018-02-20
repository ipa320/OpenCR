The code is based on the [turtlebot3_core example](https://github.com/ROBOTIS-GIT/OpenCR/tree/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_waffle/turtlebot3_core) and the [turtlebot_friends example](https://github.com/ROBOTIS-GIT/OpenCR/tree/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_friends/turtlebot3_mecanum) for mecanumwheels and uses the formulae from [this paper](http://research.ijcaonline.org/volume113/number3/pxc3901586.pdf) for calculating the kinematics ([Summary](http://robotsforroboticists.com/drive-kinematics/)).

# Setup

This code requires the [OpenCR Board by ROBOTIS](https://github.com/ROBOTIS-GIT/OpenCR/wiki) and is using the turtlebot3_core libraries to set up a ROS Node to communicate with a ROS Master. 

# Installation

Make sure to install the Arduino IDE and the OpenCR libraries by following [these Instructions](emanual.robotis.com/docs/en/platform/turtlebot3/opencr1_0_software_setup/). Then use the Arduino IDE to upload the turtlebot3_core.ino file in the turtlebot3_core directory onto the OpenCR board.

# Usage

The OpenCR Board launches a ROS Node that can be connected to by using the
```
rosrun rosserial_python serial_node.py
```
command.

Once the ROS Node is connected, the OpenCR Board publishes and subscribes to various topics listed below:

### Published Topics

| Topic Name       | Type                           | Description |
| ---------------- | ------------------------------ | ----------- |
| `sensor_state`   | `turtlebot3_msgs::SensorState` |  |
| `version_info`   | `turtlebot3_msgs::VersionInfo` |  |
| `cmd_vel_rc100`  | `geometry_msgs::Twist`         |  |
| `odom`           | `nav_msgs::Odometry`           |  |
| `joint_states`   | `sensor_msgs::JointState`      |  |
| `battery_state`  | `sensor_msgs::BatteryState`    |  |
| `magnetic_field` | `sensor_msgs::MagneticField`   |  |

### Subscribed Topics

| Topic Name    | Type                     | Description |
| ------------- | ------------------------ | ----------- |
| `cmd_vel`     | `geometry_msgs::Twist`   | Used to command the motors of the robot |
| `sound`       | `turtlebot3_msgs::Sound` | Plays a set of predefined sounds on the OpenCR Board |
| `motor_power` | `std_msgs::Bool`         | Turns the motors on or off |
| `reset`       | `std_msgs::Empty`        | Resets the board |

## Configuration

The following code lines need to be modified in the [turtlebot3_mecanum_core_config.h](turtlebot3_mecanum_core_config.h) and  [turtlebot3_mecanum_core_motor_driver](turtlebot3_mecanum_core_motor_driver.h) files: 
_Note: you might also want to change the Topic names that the OpenCR Board subscribes and publishes. This can be done in the core file._

turtlebot3_mecanum_core_config.h
```
//the radius of the wheels
#define WHEEL_RADIUS                    0.1      // meter
//half the distance between the front left and front right wheel
#define WHEEL_SEPARATION_X              0.09    // meter
//half the distance between the front and the rear wheels
#define WHEEL_SEPARATION_Y              0.115     // meter
```

turtlebot3_mecanm_core_motor_driver.h
```
#define DXL_LEFT_REAR_ID                3       // ID of left rear motor
#define DXL_RIGHT_REAR_ID               4       // ID of right rear motor
#define DXL_LEFT_FRONT_ID		            1	      // ID of left front motor
#define DXL_RIGHT_FRONT_ID           		2	      // ID of right front motor
#define BAUDRATE                        1000000 // baurd rate of Dynamixel
```

If you use other motors than the Dynamixel XM430-W210, you might need to change other parameters in the turtlebot3_mecanum_core_motor_driver.h file as well

# Debugging

## Motors

For debugging the motors the [RoboPlus Manager 2.0](http://www.robotis.us/roboplus2/) application can be used. To use this software, either the [Usb2Dynamixel](http://www.robotis-shop-en.com/?act=shop_en.goods_view&GS=1289&GC=GD0B0107) adapter or the OpenCR board ([using this code](https://github.com/ROBOTIS-GIT/OpenCR/blob/develop/arduino/opencr_arduino/opencr/libraries/OpenCR/examples/10.%20Etc/usb_to_dxl/usb_to_dxl.ino)) needs to be connected.
Make sure that only one motor is connected to the board while running the initialization software.

If you have problems detecting the OpenCR board under Windows make sure that the correct drivers are installed for the STM32 chip. ([more infos here](http://forum.espruino.com/conversations/290299/))

