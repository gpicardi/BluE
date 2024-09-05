This repository contains the software developed for the **BluE project**. The project started on the 1st September 2023 and the code will be frequently updated. 

## **silver:**

ROS2 package for the control of the Underwater Legged Robot SILVER2. The code is based on ros2_control and on the dynamixel hardware plugin from the dynamixel community available here: https://github.com/dynamixel-community/dynamixel_hardware. The code can run a single leg (3 motors) and as well as the complete hexapod (18 motors).

**Load the robot modol, controller_manager and controllers:**

* ros2 launch silver robot_ros2_control_launch.py
* ros2 launch silver sim_silver.launch.py

**To interact with the controller joint_trajectory_position_controller you can use a publisher or an action client:**

* Publisher--> **ros2 run silver trajectory_publisher.py**

* Action Client --> **python3 follow_joint_position_action.py 2** (in this example a parameter is needed to select the waypoint so I use the python3 command for simplicity)

**To interact with the controller forward_position_controller you can use the publisher:**

* Publisher --> ros2 run silver forward_position_publisher.py

**To control the hexapod, the ominidirectional gait model allows to set locomotion parameters and walk for a given number of leg cycles**

omnidirectional_gait.py <gait_width> <gait_height> <direction> <step_length> <duty_cycle> <ground_clearance> <phase_lag> <nstep> <period> <rotation> <nleg_cycles>
* python3 omnidirectional_gait.py 40.0 30.0 0.0 20.0 0.75 10.0 "180.0, 0.0, 180.0, 0.0, 180.0, 0.0" 30 5.0 0 3
* ros2 run silver omnidirectional_gait.py 40.0 30.0 0.0 20.0 0.75 10.0 "180.0, 0.0, 180.0, 0.0, 180.0, 0.0" 30 5.0 0 3

The controller used for the simulator of the complete exapod robot is pid_position_controller: type: position_controllers/JointGroupPositionController. The command interface used by ros2_gazebo is position_pid

## **dht11:**

ROS2 package to read data from the DHT11 temperature and humidity sensor. It publishes temperature and humidity for 6 sensors as two Float64Multiarrays in separate topics (/dht11/temperature and /dht11/humidity). Based on the Adafruit_DHT python library. 

## **ms8607:**

RO2 package to read data from the MS8607 pressure, temperature and relative humidity sensor. It publishes pressure, temperature and relative humidity in separate topics (/ms8607/pressure, /ms8607/temperature, /ms8607/relative_humidity). Based on the adafruit_ms8607 python library. The code provides two nodes for bus i2c-0 and i2c-1.

## **ms5837:**

RO2 package to read data from the MS5837 depth, pressure, and temperature sensor (Blue Robotics). It publishes depth, pressure, and temperature in separate topics (/ms5837/pressure, /ms5837/temperature, /ms5837/humidity). Based on the smbus library.

## **torches:**

RO2 package to control the subsea lights (Blue Robotics). It subscribes to a topic called /torches_intensity (Int64 from 0 to 100) and sends PWM commands to regulate the intensity of the lights

## **ltc2945:**

ROS2 package to read data from the LTC2945 power monitor. The calibration of the sensor depends on the shunt resistor. Calibration is not implemented yet, so currently the current and power values are not correct.

## **bno055:**

ROS2 package to read data from the BNO055 IMU. I use the code from https://github.com/flynneva/bno055


