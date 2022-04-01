#  ___   ___  ___  _   _  ___   ___   ____ ___  ____
# / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \
# | |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
# \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
#                  (____/
# OSOYOO Raspberry Pi V2.0 car lesson 2: Use Mobile APP to control the car with UDP protocol
# tutorial url: https://osoyoo.com/?p=33003

from __future__ import division
import socket
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
short_delay = 0.1
long_delay = 0.2
extra_long_delay = 0.3

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)
GPIO.setmode(GPIO.BCM)  # GPIO number  in BCM mode
GPIO.setwarnings(False)
# Set frequency to 60hz, good for servos.

ob_range = 30  # minimum obstacle distance

# define ultrasonic sensor pins
GPIO_TRIGGER = 20
GPIO_ECHO = 21
servo_lft = 500  # ultrasonic sensor facing right
servo_ctr = 300  # ultrasonic sensor facing front
servo_rgt = 150  # ultrasonic sensor facing left

pwm.set_pwm(15, 0, servo_ctr)
time.sleep(3)  # servo facing front for 3s in order to make orientation alignment

# define L298N(Model-Pi motor drive board) GPIO pins
IN1 = 23  # left motor direction pin
IN2 = 24  # left motor direction pin
IN3 = 27  # right motor direction pin
IN4 = 22  # right motor direction pin
ENA = 0  # left motor speed PCA9685 port 0
ENB = 1  # right motor speed PCA9685 port 1
sensor1 = 5  # No.1 sensor from far left
sensor2 = 6  # No.2 sensor from left
sensor3 = 13  # middle sensor
sensor4 = 19  # No.2 sensor from right
sensor5 = 26  # No.1 sensor from far  right
sts1 = 0
sts2 = 0
sts3 = 0
sts4 = 0
sts5 = 0
cur_status = '0'

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
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)  # Trigger
GPIO.setup(GPIO_ECHO, GPIO.IN)      # Echo
# Set trigger to False (Low)
GPIO.output(GPIO_TRIGGER, False)


def changespeed(speed_left, speed_right):
    pwm.set_pwm(ENA, 0, speed_left)
    pwm.set_pwm(ENB, 0, speed_right)


def stopcar():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    changespeed(0, 0)


stopcar()


def backward(speed_left, speed_right):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    changespeed(speed_left, speed_right)

    # following two lines can be removed if you want car make continuous movement without pause
    # time.sleep(short_delay)
    # stopcar()


def forward(speed_left, speed_right):
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    changespeed(speed_left, speed_right)
    # following two lines can be removed if you want car make continuous movement without pause
    # time.sleep(short_delay)
    # stopcar()


def turnRight(speed_left, speed_right):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    changespeed(speed_left, speed_right)
    # following two lines can be removed if you want car make continuous movement without pause
    # time.sleep(short_delay)
    # stopcar()


def turnLeft(speed_left, speed_right):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    changespeed(speed_left, speed_right)
    # following two lines can be removed if you want car make continuous movement without pause
    # time.sleep(short_delay)
    # stopcar()


def line_tracking():
    sts1 = 0 if GPIO.input(sensor1) else 1
    sts2 = 0 if GPIO.input(sensor2) else 1
    sts3 = 0 if GPIO.input(sensor3) else 1
    sts4 = 0 if GPIO.input(sensor4) else 1
    sts5 = 0 if GPIO.input(sensor5) else 1
    sensorval = ''.join(
        [str(sts1), str(sts2), str(sts3), str(sts4), str(sts5)])
    print(sensorval)

    if sensorval == "10000" or sensorval == "01000" or sensorval == "11000":
        turnLeft(low_speed, mid_speed)  # The black line left, sharp left turn
        time.sleep(long_delay)
        stopcar()
        time.sleep(short_delay)

    if sensorval == "11100" or sensorval == "10100":
        turnLeft(0, high_speed)  # The black line left,  left turn
        time.sleep(long_delay)
        stopcar()
        time.sleep(short_delay)

    if sensorval == "11110" or sensorval == "01100" or sensorval == "10010" or sensorval == "10110" or sensorval == "11010":
        forward(low_speed, mid_speed)  # slight eft turn
        time.sleep(long_delay)
        stopcar()
        time.sleep(short_delay)

    if sensorval == "01110" or sensorval == "01010" or sensorval == "00100" or sensorval == "10001" or sensorval == "10101" or sensorval == "10011" or sensorval == "11101" or sensorval == "10111" or sensorval == "11011" or sensorval == "11001":
        forward(mid_speed, mid_speed)  # slight eft turn
        time.sleep(long_delay)
        stopcar()
        time.sleep(short_delay)

    if sensorval == "00110" or sensorval == "01111" or sensorval == "01001" or sensorval == "01011" or sensorval == "01101":
        forward(mid_speed, low_speed)  # slight right turn
        time.sleep(long_delay)
        stopcar()
        time.sleep(short_delay)

    if sensorval == "00111" or sensorval == "00101":
        forward(high_speed, low_speed)
        # turnRight(high_speed,0) #The black line is  on the Left of the car, need  Left turn
        time.sleep(long_delay)
        stopcar()
        time.sleep(short_delay)

    if sensorval == "00001" or sensorval == "00010" or sensorval == "00011":
        # The black line is  on the Left of the car, need  Left turn
        turnRight(mid_speed, low_speed)
        time.sleep(long_delay)
        stopcar()
        time.sleep(short_delay)

    if sensorval == "11111":
        stopcar()  # The car front touch stop line, need stop

    if sensorval == "00000":
        # The car front touch stop line, need stop
        backward(low_speed, low_speed)
        time.sleep(short_delay)
        stopcar()
        time.sleep(short_delay)


def measure():
    # This function measures a distance
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    start = time.time()
    while GPIO.input(GPIO_ECHO) == 0:
        start = time.time()
    while GPIO.input(GPIO_ECHO) == 1:
        stop = time.time()
    elapsed = stop-start
    distance = (elapsed * 34300)/2
    return distance


def obstacle_avoid():
    pwm.set_pwm(15, 0, servo_lft)
    time.sleep(0.3)
    distance = measure()
    sts1 = 0 if distance > ob_range else 1

    pwm.set_pwm(15, 0, servo_ctr)
    time.sleep(0.3)
    distance = measure()
    sts2 = 0 if distance > ob_range else 1

    pwm.set_pwm(15, 0, servo_rgt)
    time.sleep(0.3)
    distance = measure()
    sts3 = 0 if distance > ob_range else 1
    sensorval = ''.join([str(sts1), str(sts2), str(sts3)])

    if sensorval == "100":
        print("100 slight right")
        forward(high_speed, mid_speed)  # slight right turn
        time.sleep(long_delay)
        stopcar()
        time.sleep(short_delay)

    if sensorval == "001":
        print("001 slight left")
        forward(mid_speed, high_speed)  # slight left turn
        time.sleep(long_delay)
        stopcar()
        time.sleep(short_delay)

    if sensorval == "110":
        print("110 sharp right")
        turnRight(high_speed, low_speed)  # shart right turn
        time.sleep(long_delay)
        stopcar()
        time.sleep(short_delay)

    if sensorval == "011" or sensorval == "010":
        print(sensorval+" sharp left")
        turnLeft(low_speed, high_speed)  # sharp left turn
        time.sleep(long_delay)
        stopcar()
        time.sleep(short_delay)

    if sensorval == "111" or sensorval == "101":
        print(sensorval+" back to left")
        turnRight(low_speed, high_speed)  # back to left side
        time.sleep(extra_long_delay)
        stopcar()
        time.sleep(short_delay)

    if sensorval == "000":
        print(sensorval+" forward")
        forward(mid_speed, mid_speed)  # go forward
        time.sleep(long_delay)
        stopcar()
        time.sleep(short_delay)


def ticker():
    if cur_status == 'R':
        turnRight(high_speed, 0)
    if cur_status == 'L':
        turnLeft(0, high_speed)
    if cur_status == 'A':
        forward(mid_speed, mid_speed)
    if cur_status == 'B':
        backward(mid_speed, mid_speed)
    if cur_status == 'E':
        stopcar()
    if cur_status == 'T':
        line_tracking()
    if cur_status == 'O':
        obstacle_avoid()


UDP_IP = ""
UDP_PORT = 8888

sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    print(cur_status)

    sock.settimeout(0.1)
    try:
        data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
    except socket.timeout:
        ticker()
        continue
    if data == b'R':
        cur_status = 'R'

    if data == b'L':
        cur_status = 'L'

    if data == b'A':
        cur_status = 'A'

    if data == b'B':
        cur_status = 'B'

    if data == b'E':
        cur_status = 'E'

    if data == b'T':
        cur_status = 'T'

    if data == b'O':
        cur_status = 'O'
