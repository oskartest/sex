#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import math
from time import sleep
from gpiozero import Motor
STOP  = 0
FORWARD  = 1
BACKWARD = 2

CH1 = 0
OUTPUT = 1
INPUT = 0

HIGH = 1
LOW = 0

ENA = 5
ENB = 27

IN1 = 21
IN2 = 20


motors = Motor(forward=20, backward=21)

class motor:
    def __init__(self, inPin1, inPin2, enAPin, enBPin):        

        GPIO.setmode(GPIO.BCM)

        self.motorPin = [[enAPin, inPin1, inPin2]]

        self.pwm = []

        self.pwm.append(self.setPinConfig(CH1))
        
    def setPinConfig(self, ch):
        EN = self.motorPin[ch][0]
        INA = self.motorPin[ch][1]
        INB = self.motorPin[ch][2]

        GPIO.setup(EN, GPIO.OUT)
        GPIO.setup(INA, GPIO.OUT)
        GPIO.setup(INB, GPIO.OUT)
     
        pwm = GPIO.PWM(EN, 100) 
      
        pwm.start(0) 
        return pwm   
    def setMotorControl(self, ch, speed, stat):        
        EN = self.motorPin[ch][0]
        INA = self.motorPin[ch][2]
        INB = self.motorPin[ch][1] #motor backward 2

        
        self.pwm[ch].ChangeDutyCycle(speed)  
        
        if stat == FORWARD:
            GPIO.output(INA, HIGH)
            GPIO.output(INB, LOW)
            
       
        elif stat == BACKWARD:
            GPIO.output(INA, LOW)
            GPIO.output(INB, HIGH)
            
 
        elif stat == STOP:
            GPIO.output(INA, LOW)
            GPIO.output(INB, LOW)
            
            
    def setMotor(self, ch, speed, stat):
        self.setMotorContorl(self.pwm[ch], self.motorPin[ch][1], speed, stat)

    def __del__(self):
       
        GPIO.cleanup()
        
class surbo:
    def __init__ (self, pin, hz):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        
        self.p = GPIO.PWM(pin, hz)
        self.p.start(0)
        
    def doAngle(self, angle):
        self.p.ChangeDutyCycle(angle)  
        
mt = motor(21, 20, 5, 27)
s = surbo(18, 50)


def callback(data):
    yaho = round((data.data),3)
    rospy.loginfo(yaho)
    motors.forward(speed=1)
    s.doAngle(7.5-yaho)
def listener():
    rospy.init_node('sub_steers', anonymous=True)
    rospy.Subscriber("/steers", Float32, callback)
    rospy.spin()
if __name__ == '__main__':
    listener()
