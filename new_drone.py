#!/usr/bin/python
# -*- coding: utf-8 -*-

############################################################################################
#
#  Importation des librairies
#
############################################################################################

#---------------------------------------------------------------------------
# Xbox read
#---------------------------------------------------------------------------

import RPi.GPIO as GPIO
import xbox_read
import time

#---------------------------------------------------------------------------
# PID
#---------------------------------------------------------------------------

import PID
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import spline

#---------------------------------------------------------------------------
# ADC
#---------------------------------------------------------------------------

from ABE_ADCPi import ADCPi
from ABE_helpers import ABEHelpers
import os

############################################################################################
#
#  Paramétrage des composants I2Cs
#
############################################################################################

i2c_helper = ABEHelpers()
bus = i2c_helper.get_smbus()
adc = ADCPi(bus, 0x68, 0x69, 12)

############################################################################################
#
#  Initialisation des GPIOs
#
############################################################################################

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

############################################################################################
#
#  Initialisation des variables
#
############################################################################################

consigne_throttle=0
consigne_throttle_av=0

consigne_pitch=0
consigne_pitch_av=0
consigne_pitch_ar=0

consigne_roll=0
consigne_roll_av=0
consigne_roll_ar=0

consigne_yaw=0
consigne_yaw_av=0
consigne_yaw_ar=0

erreur = 0
consigne = 0
mesure = 0
somme_erreurs = 0
variateur_erreur = 0
erreur_précédente =0
commande = 0
Kp = 1.2
Ki = 1
Kd = 0.001

init=True

############################################################################################
#
#  FONCTIONNEMMENT
#
############################################################################################


while(init==True) :

	time.sleep(0.01)  
    
    for event in xbox_read.event_stream(deadzone=12000):
        print event

        #---------------------------------------------------------------------------
		# Lecture des 3 axes du gyroscope depuis l'ADC
		#---------------------------------------------------------------------------

        mesure_pitch=adc.read_voltage(1)
        #print('Mesure Pitch = ', adc.read_voltage(1))
        mesure_roll=adc.read_voltage(2)
        #print('Mesure Roll = ', adc.read_voltage(2))
        mesure_yaw=adc.read_voltage(3)
        #print('Mesure Yaw = ', adc.read_voltage(3))


############################################################################################
#
#  THROTTLE
#
############################################################################################

    
        if event.key == 'RT':

        	#---------------------------------------------------------------------------
			# Lecture de la consigne throttle depuis Xbox read
			#---------------------------------------------------------------------------

            consigne_throttle=event.value
            consigne_throttle_av = ((consigne_throttle*80/254)/20) * PID(mesure_pitch, 1.35, Kp, Ki, Kd) * PID(mesure_roll, 1.35, Kp, Ki, Kd) * PID(mesure_yaw, 1.35, Kp, Ki, Kd)

            #---------------------------------------------------------------------------
			# Pilotage des GPIOs
			#---------------------------------------------------------------------------

            avg.ChangeDutyCycle(consigne_throttle_av + 5)
            #print('Moteur AV.G : Throttle = ', consigne_throttle*80/254)
            avd.ChangeDutyCycle(consigne_throttle_av + 5)
            #print('Moteur AV.D : Throttle = ', consigne_throttle*80/254)
            arg.ChangeDutyCycle(consigne_throttle_av + 5)
            #print('Moteur AR.G : Throttle = ', consigne_throttle*80/254)
            ard.ChangeDutyCycle(consigne_throttle_av + 5)
            #print('Moteur AR.D : Throttle = ', consigne_throttle*80/254)

############################################################################################
#
#  PITCH
#
############################################################################################

  
        if event.key == 'Y1':

			#---------------------------------------------------------------------------
			# Lecture de la consigne pitch depuis Xbox read
			#---------------------------------------------------------------------------

            consigne_pitch=event.value
            consigne_pitch_av = ((consigne_throttle*80/254 + consigne_pitch*20/32767 + 1)/20) * PID(mesure_pitch, 1.35, Kp, Ki, Kd) * PID(mesure_roll, 1.35, Kp, Ki, Kd) * PID(mesure_yaw, 1.35, Kp, Ki, Kd)
            consigne_pitch_ar = ((consigne_throttle*80/254 - consigne_pitch*20/32767)/20) * PID(mesure_pitch, 1.35, Kp, Ki, Kd) * PID(mesure_roll, 1.35, Kp, Ki, Kd) * PID(mesure_yaw, 1.35, Kp, Ki, Kd)

            #---------------------------------------------------------------------------
			# Pilotage des GPIOs
			#---------------------------------------------------------------------------

            avg.ChangeDutyCycle(consigne_pitch_ar + 5)
            #print('Moteur AV.G : Pitch = ', consigne_throttle*80/254 - consigne_pitch*20/32767)
            avd.ChangeDutyCycle(consigne_pitch_ar + 5)
            #print('Moteur AV.D : Pitch = ', consigne_throttle*80/254 - consigne_pitch*20/32767)
            arg.ChangeDutyCycle(consigne_pitch_av + 5)
            #print('Moteur AR.G : Pitch = ', consigne_throttle*80/254 + consigne_pitch*20/32767)
            ard.ChangeDutyCycle(consigne_pitch_av + 5)
            #print('Moteur AR.D : Pitch = ', consigne_throttle*80/254 + consigne_pitch*20/32767)

############################################################################################
#
#  ROLL
#
############################################################################################

   
        if event.key == 'X2':

			#---------------------------------------------------------------------------
			# Lecture de la consigne roll depuis Xbox read
			#---------------------------------------------------------------------------

            consigne_roll=event.value
            consigne_roll_av = ((consigne_throttle*80/254 + consigne_roll*20/32767 + 1)/20) * PID(mesure_pitch, 1.35, Kp, Ki, Kd) * PID(mesure_roll, 1.35, Kp, Ki, Kd) * PID(mesure_yaw, 1.35, Kp, Ki, Kd)
            consigne_roll_ar = ((consigne_throttle*80/254 - consigne_roll*20/32767)/20) * PID(mesure_pitch, 1.35, Kp, Ki, Kd) * PID(mesure_roll, 1.35, Kp, Ki, Kd) * PID(mesure_yaw, 1.35, Kp, Ki, Kd)

            #---------------------------------------------------------------------------
			# Pilotage des GPIOs
			#---------------------------------------------------------------------------

            avg.ChangeDutyCycle(consigne_roll_av + 5)
            #print('Moteur AV.G : Roll = ', consigne_throttle*80/254 + consigne_roll*20/32767)
            avd.ChangeDutyCycle(consigne_roll_ar + 5)
            #print('Moteur AV.D : Roll = ', consigne_throttle*80/254 - consigne_roll*20/32767)
            arg.ChangeDutyCycle(consigne_roll_av + 5)
            #print('Moteur AR.G : Roll = ', consigne_throttle*80/254 + consigne_roll*20/32767)
            ard.ChangeDutyCycle(consigne_roll_ar + 5)
            #print('Moteur AR.D : Roll = ', consigne_throttle*80/254 - consigne_roll*20/32767)
           

############################################################################################
#
#  YAW
#
############################################################################################

   
        if event.key == 'X1':

			#---------------------------------------------------------------------------
			# Lecture de la consigne yaw depuis Xbox read
			#---------------------------------------------------------------------------

            consigne_yaw=event.value
            consigne_yaw_av = ((consigne_throttle*80/254 + consigne_yaw*20/32767 + 1)/20) * PID(mesure_pitch, 1.35, Kp, Ki, Kd) * PID(mesure_roll, 1.35, Kp, Ki, Kd) * PID(mesure_yaw, 1.35, Kp, Ki, Kd)
            consigne_yaw_ar = ((consigne_throttle*80/254 - consigne_yaw*20/32767)/20) * PID(mesure_pitch, 1.35, Kp, Ki, Kd) * PID(mesure_roll, 1.35, Kp, Ki, Kd) * PID(mesure_yaw, 1.35, Kp, Ki, Kd)

            #---------------------------------------------------------------------------
			# Pilotage des GPIOs
			#---------------------------------------------------------------------------

            avg.ChangeDutyCycle(consigne_yaw_av + 5)
            #print('Moteur AV.G : Yaw = ', consigne_throttle*80/254 + consigne_yaw*20/32767)
            avd.ChangeDutyCycle(consigne_yaw_ar + 5)
            #print('Moteur AV.D : Yaw = ', consigne_throttle*80/254 - consigne_yaw*20/32767)
            arg.ChangeDutyCycle(consigne_yaw_ar + 5)
            #print('Moteur AR.G : Yaw = ', consigne_throttle*80/254 - consigne_yaw*20/32767)
            ard.ChangeDutyCycle(consigne_yaw_av + 5)
            #print('Moteur AR.D : Yaw = ', consigne_throttle*80/254 + consigne_yaw*20/32767)

############################################################################################
#
# VEILLE
#
############################################################################################

        if event.key == 'start':
            init=False
            avg.stop()
            avd.stop()
            arg.stop()
            ard.stop()
            GPIO.cleanup()
            print('Drone en veille')
            break

############################################################################################
#
#  OFF
#
############################################################################################

        if event.key == 'back':
            init=False
            avg.stop()
            avd.stop()
            arg.stop()
            ard.stop()
            GPIO.cleanup()
            print('Drone éteint')
            os.system('sudo halt')

############################################################################################
#
#  CLEAN GPIOs
#
############################################################################################

        if event.key == 'A':
            GPIO.cleanup()

#---------------------------------------------------------------------------
# PID
#---------------------------------------------------------------------------
def PID(consigne, mesure, Kp, Ki, Kd):

	erreur = consigne - mesure
	somme_erreurs += erreur
	variation_erreur = erreur - erreur_précédente
	commande = Kp * erreur + Ki * somme_erreurs + Kd * variation_erreur
	erreur_précédente = erreur
	return commande