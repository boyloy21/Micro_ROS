# Micro_ROS
MicroROS is serial communicate with PC use ROS2 to Microcontroller.
## I have 3 type to Communicate:
1. Publisher from STM32 and PC Subsribe data.
   - Upload publisher file to STM32 and run subscriber.py file
2. Publisher from PC and STM32 Subscribe data.
   - Upload subscriber file to STM32 and run Publish_twist.py file
3. I use Service server from STM32 to Service client in PC.
   - Updload Service_client file to STM32 board and run py_service_client.py
