#!/usr/bin/env python

import lcddriver
lcd = lcddriver.lcd()

from subprocess import *
from time import sleep, strftime
from datetime import datetime

import threading

import RPi.GPIO as GPIO


# Use Pi board pin numbers as these as always consistant between revisions 
GPIO.setmode(GPIO.BOARD)                                 

#emonPi push button Pin 16 GPIO 23
GPIO.setup(16, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)    




print datetime.now().strftime('%b %d %H:%M')

class ButtonInput():

    def __init__(self):
        GPIO.add_event_detect(16, GPIO.RISING, callback=self.buttonPress, bouncetime=500) 
        self.press_num = 0

    def buttonPress(self,channel):
        print('Button 1 pressed!') 
        self.press_num = self.press_num + 1 

buttoninput = ButtonInput()
#Setup callback function buttonpress to appen on press of push button    



# Return local IP address for eth0 or wlan0
def local_IP():
	wlan0 = "ip addr show wlan0 | grep inet | awk '{print $2}' | cut -d/ -f1 | head -n1"
	eth0 = "ip addr show eth0 | grep inet | awk '{print $2}' | cut -d/ -f1 | head -n1"
	p = Popen(eth0, shell=True, stdout=PIPE)
	IP = p.communicate()[0]
	network = "eth0"
	if IP == "":
		p = Popen(wlan0, shell=True, stdout=PIPE)
		IP = p.communicate()[0].rstrip('\n')
		network = "wlan0"
	return {IP , network}
#IP, network = local_IP()
#print IP
#print network

#Test to see if we have working internet connection to emoncms.org 
import socket
REMOTE_SERVER = "www.google.com"
def is_connected():
  try:
    # see if we can resolve the host name -- tells us if there is
    # a DNS listening
    host = socket.gethostbyname(REMOTE_SERVER)
    # connect to the host -- tells us if the host is actually
    # reachable
    s = socket.create_connection((host, 80), 2)
    return True
  except:
     pass
  return False
#print 'Internet connected? %s' %(is_connected())

#update lcd 
def updatelcd():
    threading.Timer(3, updatelcd).start()
    lcd.lcd_clear();

 
while 1:

    ##updatelcd();

    if buttoninput.press_num == 0:   
        print "screen 0" 
        print buttoninput.press_num
        IP, network = local_IP()
        if IP == "":
            lcd.lcd_display_string('Awaiting Network',1) 
            lcd.lcd_display_string('Connection......',2)
            
        if IP != "":
            lcd.lcd_display_string('%s connected' % (IP),1)
            lcd.lcd_display_string('IP: %s' % ( network ),2)

    elif buttoninput.press_num == 1:          
        print "screen 2"
        print buttoninput.press_num
        lcd.lcd_display_string('Checking WAN    ',1) 
        lcd.lcd_display_string('Connection......',2)
        #if is_connected() == True:
        #    lcd.lcd_display_string('Internet',1)
        #    lcd.lcd_display_string('Connected',2)
        #else:
        #    lcd.lcd_display_string('Internet',1)
        #    lcd.lcd_display_string('Connection FAIL',2)

    elif buttoninput.press_num == 2: 
        print "screen 2"
        print buttoninput.press_num       
        lcd.lcd_display_string(datetime.now().strftime('%b %d %H:%M'),1)
        lcd.lcd_display_string(datetime.now().strftime('Uptime: 123 days'),2)

    else:
        print "reset"
        print buttoninput.press_num
        buttoninput.press_num = 0

#

            
GPIO.cleanup()
        



