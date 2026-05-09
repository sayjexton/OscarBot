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
limit = 25

def servo_up():
	left_servo.value = 0.9
	right_servo.value = 1

def servo_detach():
	left_servo.detach()
	right_servo.detach()

def oscar_stop():
	front_left_motor.stop()
	front_right_motor.stop()
	back_left_motor.stop()
	back_right_motor.stop()
	
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

def oscar_point_left(speed, deg):
	if (deg != "inf"):
		t = (0.77*deg)/90
		front_left_motor.backward(speed)
		front_right_motor.forward(speed)
		back_left_motor.backward(speed)
		back_right_motor.forward(speed)
		sleep(t)
		oscar_stop()
	elif(deg == "inf"):
		front_left_motor.backward(speed)
		front_right_motor.forward(speed)
		back_left_motor.backward(speed)
		back_right_motor.forward(speed)
	else:
		print("Invalid degree measure input pointing right.")

def oscar_point_right(speed, deg):
	if (deg != "inf"):
		t = (0.77*deg)/90
		front_left_motor.forward(speed)
		front_right_motor.backward(speed)
		back_left_motor.forward(speed)
		back_right_motor.backward(speed)
		sleep(t)
		oscar_stop()
	elif (deg == "inf"):
		front_left_motor.forward(speed)
		front_right_motor.backward(speed)
		back_left_motor.forward(speed)
		back_right_motor.backward(speed)
	else:
		print("Invalid degree measure input pointing right.")

def oscar_spin():
	t = 0.77*4
	oscar_point_left(1)
	sleep(t)
	oscar_stop()
	sleep(0.5)
	oscar_point_right(1)
	sleep(t)
	oscar_stop()

# Object avoidance
while True:
	servo_detach()
	
	# check if there are objects
	distance_front = front_us.distance * 100
	distance_back = back_us.distance * 100
	distance_left = left_us.distance * 100
	distance_right = right_us.distance * 100
	
	print(distance_front)
	

	if (distance_front >= limit):
		oscar_forward(1)
	else:
		oscar_stop()
		sleep(1)
		if (distance_front < limit):
			oscar_point_left(1, 90)
			sleep(0.5)
			oscar_point_right(1, 180)
			sleep(0.5)
		elif (distance_back < limit):
			print()
		elif (distance_left < limit):
			print()
		elif (distance_right < limit):
			print()
