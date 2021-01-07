# Check point 2
* This is a package for check point 2.
* Be familiar with the basic usages of TT motor, encoder, and L298N motor driver.
* Measure the angular velocity of motor with encoder.
* Control motor velocity with PWM commands.
* Design PID controller for angular velocity control.
* Follow the instruction [here](https://hackmd.io/7vuKIsrBSCqO7Fd8SfpNEQ).

## Script on Arduino
### Open loop control of motor
Run arduino/open_loop_two_wheel/open_loop_two_wheel.ino on Arduino.
### Encoder
Run arduino/encoder/encoder_one_wheel/encoder_one_wheel.ino or arduino/encoder/encoder_two_wheel/encoder_two_wheel.ino on Arduino.
### PID control for the motor
Run arduino/pid_control/pid_one_wheel/pid_one_wheel.ino or arduino/pid_control/pid_two_wheel/pid_two_wheel.ino on Arduino.
### For checkpoint 2
Run arduino/checkpoint_2_arduino/checkpoint_2_arduino.ino on Arduino.

## Launch file on Raspberry Pi3
```
roslaunch checkpoint_2 cp_2.launch
```
