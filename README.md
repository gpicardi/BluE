This repository contains the software developed for the BluE project. The project started on the 1st September 2023 and the code will be frequently updated. 

silver:
ROS2 package for the control of the Underwater Legged Robot SILVER2. The code is based on ros2_control and on the dynamixel hardware plugin from the dynamixel community available here: https://github.com/dynamixel-community/dynamixel_hardware

dht11:
ROS2 package to read data from the DHT11 temperature and humidity sensor. It publishes temperature and humidity in separate topics (/dht11/temperature and /dht11/humidity). Based on the Adafruit_DHT python library.

ms8607:
RO2 package to read data from the MS8607 pressure, temperature and relative humidity sensor. It publishes pressure, temperature and relative humidity in separate topics (/ms8607/pressure, /ms8607/temperature, /ms8607/relative_humidity). Based on the adafruit_ms8607 python library.

bno055:
ROS2 package to read data from the BNO055 IMU. I use the code from https://github.com/flynneva/bno055


