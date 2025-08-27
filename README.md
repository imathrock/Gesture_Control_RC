# Gesture Control RC
Gesture control RC documentation. A capstone project for UBC PHYS 319 Embedded Systems. 

## how it works
There are 2 files, transmitter.c uses a custom I2C driver to read data from the mpu6050 accelerometer and gyroscope and uses the msprf24 library to transmit the control input over nrf24 antennae. 
The reciever.c recieves over nrf24 and sends the control output to the motor controller, the speed is controlled using the received values which are PWM values for speed control. There is a short safety cutoff, as in the robot does not respond to angles between 10 and -10 degrees both sides. This is so that the robot does not jitter from low pwm values.

## About good engineering practices
I have many debug functions in the code and also some test functions. The idea is to have good embedded programming practices, which means visual confirmation of initialization of certain functions that were known to be slightly irregular in behaviour. This is why I have code that blinks leds in certain manner that let me know if initialization of sensors and other things is complete making it easier to debug. 