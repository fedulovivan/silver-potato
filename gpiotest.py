#!/usr/bin/python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time
import mpd
import pprint

BTN_PIN = 23

pp = pprint.PrettyPrinter(indent=4)

client = mpd.MPDClient(use_unicode=True)
client.connect("localhost", 6600)

GPIO.setmode(GPIO.BCM)

GPIO.setup(BTN_PIN, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

def report_button_press(chanel):
  print("Button pressed")
  client.next()

GPIO.add_event_detect(BTN_PIN, GPIO.RISING, callback=report_button_press, bouncetime=200)

while True:
  client.ping()	
  time.sleep(10)  

GPIO.cleanup()
client.close()
client.disconnect()	