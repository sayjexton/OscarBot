from gpiozero import Motor
from gpiozero import Servo
from gpiozero import DistanceSensor
from time import sleep

# Motors
front_left_motor = Motor(forward=25, backward=24, enable=18)
back_left_motor = Motor(forward=7, backward=8, enable=23)
back_right_motor = Motor(forward=22, backward=4, enable=11)
front_right_motor = Motor(forward=9, backward=10, enable=6)

# Servos
left_servo = Servo(13)
right_servo = Servo(1)

# Ultrasonic sensors
front_us = DistanceSensor(trigger=21, echo=19)
left_us = DistanceSensor(trigger=0, echo=5)
right_us = DistanceSensor(trigger=14, echo=15)
back_us = DistanceSensor(trigger=20, echo=16)

def servo_up():
	left_servo.value = 0.9
	right_servo.value = 1

def servo_detach():
	left_servo.detach()
	right_servo.detach()
	
def oscar_forward(speed):
	front_left_motor.forward(speed)
	front_right_motor.forward(speed)
	back_left_motor.forward(speed)
	back_right_motor.forward(speed)
	
def oscar_backward(speed):
	front_left_motor.backward(speed)
	front_right_motor.backward(speed)
	back_left_motor.backward(speed)
	back_right_motor.backward(speed)

def oscar_point_left(speed):
	front_left_motor.backward(speed)
	front_right_motor.forward(speed)
	back_left_motor.backward(speed)
	back_right_motor.forward(speed)

def oscar_point_right(speed):
	front_left_motor.forward(speed)
	front_right_motor.backward(speed)
	back_left_motor.forward(speed)
	back_right_motor.backward(speed)

def oscar_strafe_left(speed):
	front_left_motor.backward(speed)
	front_right_motor.forward(speed)
	back_left_motor.forward(speed)
	back_right_motor.backward(speed)

def oscar_strafe_right(speed):
	front_left_motor.forward(speed)
	front_right_motor.backward(speed)
	back_left_motor.backward(speed)
	back_right_motor.forward(speed)

def oscar_stop():
	front_left_motor.stop()
	front_right_motor.stop()
	back_left_motor.stop()
	back_right_motor.stop()

while True:
	servo_detach()
