#  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
# / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
#| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
# \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
#                  (____/ 
# Osoyoo Raspberry Pi Robot Car Line Tracking auto driving
# tutorial url: https://osoyoo.com/?p=32082

from __future__ import division
import time
# Import the PCA9685 module.
import Adafruit_PCA9685
import RPi.GPIO as GPIO
# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)
high_speed = 4000  # Max pulse length out of 4096
mid_speed = 2000  # Max pulse length out of 4096
low_speed = 1000  # Max pulse length out of 4096
short_delay=0.1
long_delay=0.2
extra_long_delay=0.3

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)
GPIO.setmode(GPIO.BCM) # GPIO number  in BCM mode
GPIO.setwarnings(False)
#define L298N(Model-Pi motor drive board) GPIO pins
IN1 = 23  #left motor direction pin
IN2 = 24  #left motor direction pin
IN3 = 27  #right motor direction pin
IN4 = 22  #right motor direction pin
ENA = 0  #left motor speed PCA9685 port 0
ENB = 1  #right motor speed PCA9685 port 1
sensor1= 5 # No.1 sensor from far left
sensor2= 6 # No.2 sensor from left
sensor3= 13 # middle sensor
sensor4= 19 # No.2 sensor from right
sensor5= 26 #No.1 sensor from far  right
sts1=0
sts2=0
sts3=0
sts4=0
sts5=0

# Define motor control  pins as output
GPIO.setup(IN1, GPIO.OUT)   
GPIO.setup(IN2, GPIO.OUT) 
GPIO.setup(IN3, GPIO.OUT)   
GPIO.setup(IN4, GPIO.OUT) 
GPIO.setup(sensor1, GPIO.IN)   
GPIO.setup(sensor2, GPIO.IN)
GPIO.setup(sensor3, GPIO.IN)   
GPIO.setup(sensor4, GPIO.IN)
GPIO.setup(sensor5, GPIO.IN)  

def changespeed(speed_left,speed_right):
	pwm.set_pwm(ENA, 0, speed_left)
	pwm.set_pwm(ENB, 0, speed_right)

def stopcar():
	GPIO.output(IN1, GPIO.LOW)
	GPIO.output(IN2, GPIO.LOW)
	GPIO.output(IN3, GPIO.LOW)
	GPIO.output(IN4, GPIO.LOW)
	changespeed(0,0)
stopcar()

def backward(speed_left,speed_right):
	GPIO.output(IN1, GPIO.HIGH)
	GPIO.output(IN2, GPIO.LOW)
	GPIO.output(IN3, GPIO.HIGH)
	GPIO.output(IN4, GPIO.LOW)
	changespeed(speed_left,speed_right)
 
	#following two lines can be removed if you want car make continuous movement without pause
	#time.sleep(short_delay) 
	#stopcar()
	
def forward(speed_left,speed_right):
	GPIO.output(IN2, GPIO.HIGH)
	GPIO.output(IN1, GPIO.LOW)
	GPIO.output(IN4, GPIO.HIGH)
	GPIO.output(IN3, GPIO.LOW)
	changespeed(speed_left,speed_right)
	#following two lines can be removed if you want car make continuous movement without pause
	#time.sleep(short_delay) 
	#stopcar()
	
def turnRight(speed_left,speed_right):
	GPIO.output(IN1, GPIO.LOW)
	GPIO.output(IN2, GPIO.HIGH)
	GPIO.output(IN3, GPIO.HIGH)
	GPIO.output(IN4, GPIO.LOW)
	changespeed(speed_left,speed_right)
	#following two lines can be removed if you want car make continuous movement without pause
	#time.sleep(short_delay) 
	#stopcar()
	
def turnLeft(speed_left,speed_right):
	GPIO.output(IN1, GPIO.HIGH)
	GPIO.output(IN2, GPIO.LOW)
	GPIO.output(IN3, GPIO.LOW)
	GPIO.output(IN4, GPIO.HIGH)
	changespeed(speed_left,speed_right)	
	#following two lines can be removed if you want car make continuous movement without pause
	#time.sleep(short_delay) 
	#stopcar()
print('Moving servo on channel 0, press Ctrl-C to quit...')

try:
	while True :

		sts1 =  0 if GPIO.input(sensor1) else 1
		sts2 =  0 if GPIO.input(sensor2) else 1
		sts3 =  0 if GPIO.input(sensor3) else 1
		sts4 =  0 if GPIO.input(sensor4) else 1
		sts5 =  0 if GPIO.input(sensor5) else 1


		sensorval = ''.join([str(sts1), str(sts2), str(sts3), str(sts4), str(sts5)])
		print(sensorval)

		if  sensorval=="10000"  or sensorval=="01000" or sensorval=="11000":
			turnLeft(low_speed,mid_speed)   #The black line left, sharp left turn
			#time.sleep(long_delay)  
			#stopcar()
			#time.sleep(short_delay) 
		
		if  sensorval=="01100"  or sensorval=="11100"  or sensorval=="11110" :
			turnLeft(0,high_speed)   #The black line left,  left turn
			#time.sleep(long_delay)  
			#stopcar()
			#time.sleep(short_delay) 	
			
		if sensorval=="00001"  or sensorval=="00010" or sensorval=="00011":
			turnRight(mid_speed,low_speed) #The black line is  on the Left of the car, need  Left turn 
			#time.sleep(long_delay)  
			#stopcar()
			#time.sleep(short_delay)	
		
		if  sensorval=="00110" or  sensorval=="00111" or sensorval=="01111":
			forward(mid_speed,low_speed) #slight right turn
			#time.sleep(long_delay)  
			#stopcar()
			#time.sleep(short_delay)	

		if  sensorval=="00100"  or sensorval=="01110":
			forward(mid_speed,mid_speed) #slight right turn
			#time.sleep(long_delay)  
			#stopcar()
			#time.sleep(short_delay) 

		
		if sensorval=="11111" :
			stopcar() #The car front touch stop line, need stop
 
except KeyboardInterrupt:
  # User pressed CTRL-C
  # Reset GPIO settings
  pwm.set_pwm(15, 0, 0)
  GPIO.cleanup()