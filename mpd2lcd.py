#!/usr/bin/python
# -*- coding: utf-8 -*-

import smbus
import time
import string
from time import gmtime, strftime
import mpd
import pprint
import math
import RPi.GPIO as GPIO
from threading import Thread

# gpio pin, button is connected
BTN_PIN = 23

# Define some device parameters
I2C_ADDR  = 0x27 # I2C device address
LCD_WIDTH = 16   # Maximum characters per line

# Define some device constants
LCD_CHR = 1 # Mode - Sending command
LCD_CMD = 0 # Mode - Sending data

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line

LCD_BACKLIGHT  = 0x08  # On
#LCD_BACKLIGHT = 0x00  # Off

ENABLE = 0b00000100 # Enable bit

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

#Open I2C interface
bus = smbus.SMBus(1)

# create mpd client instance
client = mpd.MPDClient(use_unicode=True)
client.connect("localhost", 6600)

# pretty printer library
pp = pprint.PrettyPrinter(indent=4)

# gpio pin setup
def gpio_setup():
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(BTN_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
  GPIO.add_event_detect(BTN_PIN, GPIO.FALLING, callback=button_handler, bouncetime=200)

def button_handler(pin_num):
  print("Button pressed")
  stat = client.status()
  state = stat.get("state")
  if state == "stop":
    client.play()
  else:  
    client.next()

def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off 
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  #time.sleep(E_DELAY)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = the data
  # mode = 1 for character
  #        0 for command

  bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
  bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT

  # High bits
  #bus.write_byte(I2C_ADDR, bits_high)
  lcd_toggle_enable(bits_high)

  # Low bits
  #bus.write_byte(I2C_ADDR, bits_low)
  lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
  # Toggle enable
  #time.sleep(E_DELAY)
  bus.write_byte(I2C_ADDR, (bits | ENABLE))
  #time.sleep(E_PULSE)
  bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
  #bus.write_byte(I2C_ADDR, bits);
  #bus.write_byte(I2C_ADDR, ENABLE);
  #bus.write_byte(I2C_ADDR, ~ENABLE);
  #time.sleep(E_DELAY)

def lcd_string(message,line):
  # Send string to display

  message = message.ljust(LCD_WIDTH, " ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

def scrolling_text():

  curr = client.currentsong()
  message = curr.get("file")

  totalsize = len(message)
  ifrom = 0
  while True:
    if(ifrom > totalsize - LCD_WIDTH/2):
      ifrom = 0
    to = ifrom + LCD_WIDTH
    lcd_string(message[ifrom:to], LCD_LINE_1)
    ifrom += 4
    time.sleep(1)    

def main():

  lcd_init()
  gpio_setup()

  while True:

    curr = client.currentsong()
    stat = client.status()

    #pp.pprint(curr)
    #pp.pprint(stat)

    state = stat.get("state")
    raw_time = stat.get("time")
    title = curr.get("title")

    if title == None:
      line1 = "No info"
    else:
      line1 = title

    if state == "play" or state == "pause":
      tt = raw_time.split(":", 2)
      csec = float(tt[0])
      tsec = float(tt[1])
      pntg = int(math.floor(csec/tsec*100))
      line2 = "{0} {1}% / {2}".format({
        'play': '>', 'pause': '||'}[state],
        pntg, 
        time.strftime('%Mm%Ss', time.gmtime(tsec))
      )   
    elif state == "stop":
      line2 = "STOPPED"
    else:
      line2 = "Error"
      
    lcd_string(line1, LCD_LINE_1)
    lcd_string(line2, LCD_LINE_2)

    #if "thread" not in locals():
    #  thread = Thread(target = scrolling_text)
    #  thread.start()
    #scrolling_text(curr.get("file"))

    time.sleep(1)

if __name__ == '__main__':

  try:
    main()
  except KeyboardInterrupt:
    pass
  finally:
    lcd_byte(0x01, LCD_CMD)
    client.close()
    client.disconnect()
