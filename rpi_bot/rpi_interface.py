import RPi.GPIO as GPIO
import pigpio
import time

class RPi_Motors(object):

	def __init__(self, ena, in1, in2, in3, in4, enb):
		self.ENA = ena
		self.IN1 = in1
		self.IN2 = in2
		self.IN3 = in3
		self.IN4 = in4
		self.ENB = enb

		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)

		GPIO.setup(self.ENA, GPIO.OUT)
		GPIO.setup(self.IN1, GPIO.OUT)
		GPIO.setup(self.IN2, GPIO.OUT)

		GPIO.setup(self.IN3, GPIO.OUT)
		GPIO.setup(self.IN4, GPIO.OUT)
		GPIO.setup(self.ENB, GPIO.OUT)

		self.PWMA = GPIO.PWM(self.ENA, 500)
		self.PWMB = GPIO.PWM(self.ENB, 500)
		self.PWMA.start(0)
		self.PWMB.start(0)

	def __del__(self):
		GPIO.cleanup([self.ENA, self.IN1, self.IN2, self.IN3, self.IN4, self.ENB])

	def setMotors(self, left_vel, right_vel):
   		# Left Motor(s)
		if ((left_vel >= 0) and (left_vel <= 100)):
			GPIO.output(self.IN3, GPIO.HIGH)
			GPIO.output(self.IN4, GPIO.LOW)
			self.PWMB.ChangeDutyCycle(left_vel)
		elif ((left_vel < 0) and (left_vel >= -100)):
			GPIO.output(self.IN3, GPIO.LOW)
			GPIO.output(self.IN4, GPIO.HIGH)
			self.PWMB.ChangeDutyCycle(0 - left_vel)

   	 	# Right Motor(s)
		if ((right_vel >= 0) and (right_vel <= 100)):
			GPIO.output(self.IN1, GPIO.HIGH)
			GPIO.output(self.IN2, GPIO.LOW)
			self.PWMA.ChangeDutyCycle(right_vel)
		elif ((right_vel < 0) and (right_vel >= -100)):
			GPIO.output(self.IN1, GPIO.LOW)
			GPIO.output(self.IN2, GPIO.HIGH)
			self.PWMA.ChangeDutyCycle(0 - right_vel)


class RPi_HCS04(object):

	def __init__(self, trig, echo):
		self.TRIG = trig
		self.ECHO = echo

		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)

		GPIO.setup(self.TRIG, GPIO.OUT)
		GPIO.setup(self.ECHO, GPIO.IN)

	def __del__(self):
		GPIO.cleanup([self.TRIG, self.ECHO])

	# In cm?
	def measure_distance(self):
		GPIO.output(self.TRIG, GPIO.HIGH)
		time.sleep(0.00001) # Setting TRIG high for 10 microseconds sends out ultrasonic sound pulse
		GPIO.output(self.TRIG, GPIO.LOW)

		pulseStart = time.time()
		pulseStop = time.time()
		
		while GPIO.input(self.ECHO) == 0:
			pulseStart = time.time()

		while GPIO.input(self.ECHO) == 1:
			pulseStop = time.time()

		pulseDuration = pulseStop - pulseStart
		print(pulseDuration)

		distance = pulseDuration * 17150
		distance = round(distance, 2)

		#if distance < 2 or distance > 400:
		#	return -1
		return distance