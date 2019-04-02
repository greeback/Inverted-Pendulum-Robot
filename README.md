# Inverted-Pendulum-Robot
This is a project of Two wheeled self balancing robot based on inverted pendulum model. It consists of two wheels with DC motors, dual full bridge driver - L298, homemade chasis, stm32f401 discovery, 8X battery AA. To measure tilt angle I utilized accelerometer and giroscope which are placed on discovery board.

Technology:
PWM - motor control
SPI - communication with Gyro
I2C - communication with Accelerometer
GPIO - LED control

Algorithms:
Complementary filter - tilt measurement with readings from accelerometer and gyro
PID controller - stabilizing the robot

Software:
Project is created in IAR and everything is written based mostly on CMSIS package. 

* if you wanna see robot in action open: https://www.youtube.com/watch?v=JkEyXQgvNcU

