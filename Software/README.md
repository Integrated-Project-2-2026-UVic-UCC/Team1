# 3. 💻 `./Software`

A folder meant to save all the softwares of the robot, from the firmware (low-level code) to the control software (high-level code and simulations).

## 3.1 Control
`./Software/Control`:
Includes MuJoCo `mjlab` trains and models for the control. Also includes all the ROS2 packages to control the robot
<!--TODO: Guillem adds info about his packages/nodes-->

## 3.2 Firmware
`./Software/Firmware`:
Includes a PlatformIO project for a ESP32 to send PWM to the Servomotors with a custom library `ServoCtrl.h`, and read RAW data from a Inertial Mesurement Unit (IMU) with 9DOF (Accelerometer, Gyroscope and Magnetometer).  

To let the coumication between the hardware and the software be wireless we used **Zenoh**, a layer to interact with ROS2 middleware. With Zenoh, via WiFi, we can subscribe to ROS2 topics trough the `zenoh-bridge-ros2dds`.  
<!--TODO: Add bash code fragments info-->

To run the code correctly it's necessary to add the **SSID** used in the ROS2 router, if necessary specify the **locator IP** (otherwise it will scout) and put the **Key Expression** (ROS2 topic).
<!--TODO: Add cpp code fragments info-->

<!--TODO: Add IMU info-->

When running, you must wait the Builtin led (LED 2) is blue, wich means the comunication is ready.

