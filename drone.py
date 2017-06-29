#!/usr/bin/python
# -*- coding: utf-8 -*-

#from lib import xbox_read
import RPi.GPIO as GPIO
import xbox_read
import time

from ABE_ADCPi import ADCPi
from ABE_helpers import ABEHelpers
import os

i2c_helper = ABEHelpers()
bus = i2c_helper.get_smbus()
adc = ADCPi(bus, 0x68, 0x69, 12)

#Commande Moteur
#Initialisation des GPIO



GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM) 

GPIO.setup(12, GPIO.OUT)

GPIO.setup(13, GPIO.OUT) 

GPIO.setup(18, GPIO.OUT) 

GPIO.setup(19, GPIO.OUT)

avg = GPIO.PWM(12, 50)
avg.start(0)

avd = GPIO.PWM(13, 50)
avd.start(0)

arg = GPIO.PWM(18, 50)
arg.start(0)

ard = GPIO.PWM(19, 50)
ard.start(0)

#Initialisations des consignes

consigne_throttle=0
consigne_pitch=0
consigne_roll=0
consigne_yaw=0

init=True


while(init==True) :
    
    
    for event in xbox_read.event_stream(deadzone=12000):
        print event

        mesure_pitch=adc.read_voltage(1)
        print('Mesure Pitch = ', adc.read_voltage(1))
        mesure_roll=adc.read_voltage(2)
        print('Mesure Roll = ', adc.read_voltage(2))
        mesure_yaw=adc.read_voltage(3)
        print('Mesure Yaw = ', adc.read_voltage(3))


#THROTTLE

    
        if event.key == 'RT':
            consigne_throttle=event.value
            avg.ChangeDutyCycle((consigne_throttle*80/254)/20 + 5)
            print('Moteur AV.G : Throttle = ', consigne_throttle*80/254)
            avd.ChangeDutyCycle((consigne_throttle*80/254)/20 + 5)
            print('Moteur AV.D : Throttle = ', consigne_throttle*80/254)
            arg.ChangeDutyCycle((consigne_throttle*80/254)/20 + 5)
            print('Moteur AR.G : Throttle = ', consigne_throttle*80/254)
            ard.ChangeDutyCycle((consigne_throttle*80/254)/20 + 5)
            print('Moteur AR.D : Throttle = ', consigne_throttle*80/254)

#PITCH

  
        if event.key == 'Y1':
            consigne_pitch=event.value
            avg.ChangeDutyCycle((consigne_throttle*80/254 - consigne_pitch*20/32767)/20 + 5)
            print('Moteur AV.G : Pitch = ', consigne_throttle*80/254 - consigne_pitch*20/32767)
            avd.ChangeDutyCycle((consigne_throttle*80/254 - consigne_pitch*20/32767)/20 + 5)
            print('Moteur AV.D : Pitch = ', consigne_throttle*80/254 - consigne_pitch*20/32767)
            arg.ChangeDutyCycle((consigne_throttle*80/254 + consigne_pitch*20/32767 + 1)/20 + 5)
            print('Moteur AR.G : Pitch = ', consigne_throttle*80/254 + consigne_pitch*20/32767)
            ard.ChangeDutyCycle((consigne_throttle*80/254 + consigne_pitch*20/32767 + 1)/20 + 5)
            print('Moteur AR.D : Pitch = ', consigne_throttle*80/254 + consigne_pitch*20/32767)

#ROLL

   
        if event.key == 'X2':
            consigne_roll=event.value
            avg.ChangeDutyCycle((consigne_throttle*80/254 + consigne_roll*20/32767 + 1)/20 + 5)
            print('Moteur AV.G : Roll = ', consigne_throttle*80/254 + consigne_roll*20/32767)
            avd.ChangeDutyCycle((consigne_throttle*80/254 - consigne_roll*20/32767)/20 + 5)
            print('Moteur AV.D : Roll = ', consigne_throttle*80/254 - consigne_roll*20/32767)
            arg.ChangeDutyCycle((consigne_throttle*80/254 + consigne_roll*20/32767 + 1)/20 + 5)
            print('Moteur AR.G : Roll = ', consigne_throttle*80/254 + consigne_roll*20/32767)
            ard.ChangeDutyCycle((consigne_throttle*80/254 - consigne_roll*20/32767)/20 + 5)
            print('Moteur AR.D : Roll = ', consigne_throttle*80/254 - consigne_roll*20/32767)
           

#YAW

   
        if event.key == 'X1':
            consigne_yaw=event.value
            avg.ChangeDutyCycle((consigne_throttle*80/254 + consigne_yaw*20/32767 + 1)/20 + 5)
            print('Moteur AV.G : Yaw = ', consigne_throttle*80/254 + consigne_yaw*20/32767)
            avd.ChangeDutyCycle((consigne_throttle*80/254 - consigne_yaw*20/32767)/20 + 5)
            print('Moteur AV.D : Yaw = ', consigne_throttle*80/254 - consigne_yaw*20/32767)
            arg.ChangeDutyCycle((consigne_throttle*80/254 - consigne_yaw*20/32767)/20 + 5)
            print('Moteur AR.G : Yaw = ', consigne_throttle*80/254 - consigne_yaw*20/32767)
            ard.ChangeDutyCycle((consigne_throttle*80/254 + consigne_yaw*20/32767 + 1)/20 + 5)
            print('Moteur AR.D : Yaw = ', consigne_throttle*80/254 + consigne_yaw*20/32767)

#Mode veille

        if event.key == 'start':
            init=False
            avg.stop()
            avd.stop()
            arg.stop()
            ard.stop()
            GPIO.cleanup()
            print('Drone en veille')
            break

#Mode OFF

        if event.key == 'back':
            init=False
            avg.stop()
            avd.stop()
            arg.stop()
            ard.stop()
            GPIO.cleanup()
            print('Drone éteint')
            os.system('sudo halt')

#GPIO cleanup
        if event.key == 'A':
            GPIO.cleanup()
