# Micro_ROS
MicroROS is serial communicate with PC use ROS2 to Microcontroller.<br>
_ First to use this don't forget install Micro ros is [click Link for install]((https://micro.ros.org/docs/tutorials/core/first
application linux/)<br>
_ Second use setup microros [click Link for setup](https://github.com/micro-ROS/micro ros
stm32cubemx utils/tree/foxy)
## I have 3 type to Communicate:
1. Publisher from STM32 and PC Subsribe data.
   - Upload publisher file to STM32 and run subscriber.py file
2. Publisher from PC and STM32 Subscribe data.
   - Upload subscriber file to STM32 and run Publish_twist.py file
3. I use Service server from STM32 to Service client in PC.
   - Updload Service_client file to STM32 board and run py_service_client.py
