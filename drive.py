from gpiozero import Motor
from gpiozero import Servo
from gpiozero import DistanceSensor
from time import sleep

# Motors
front_left_motor = Motor(forward=10, backward=22)
back_left_motor = Motor(forward=4, backward=9)
back_right_motor = Motor(forward=8, backward=7)
front_right_motor = Motor(forward=24, backward=25)

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

	
def oscar_forward():
	front_left_motor.forward()
	front_right_motor.forward()
	back_left_motor.forward()
	back_right_motor.forward()
	
def oscar_backward():
	front_left_motor.backward()
	front_right_motor.backward()
	back_left_motor.backward()
	back_right_motor.backward()

def oscar_strafe_left():
	front_left_motor.backward()
	front_right_motor.forward()
	back_left_motor.forward()
	back_right_motor.backward()
	
def oscar_strafe_right():
	front_left_motor.forward()
	front_right_motor.backward()
	back_left_motor.backward()
	back_right_motor.forward()

def oscar_point_left():
	front_left_motor.backward()
	front_right_motor.forward()
	back_left_motor.backward()
	back_right_motor.forward()

def oscar_point_right():
	front_left_motor.forward()
	front_right_motor.backward()
	back_left_motor.forward()
	back_right_motor.backward()
	
def oscar_stop():
	front_right_motor.stop()
	front_left_motor.stop()
	back_right_motor.stop()
	back_left_motor.stop()
	

while True:
	servo_detach()
	
	if (front_us.distance * 100 < 10):
		servo_up()

	print("L: ", left_us.distance * 100)
	print("F: ", front_us.distance * 100)
	print("B: ", back_us.distance * 100)
	print("R: ", right_us.distance * 100)
	
	sleep(1)
