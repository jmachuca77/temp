Python 3.4.2 (default, Oct 19 2014, 13:31:11) 
[GCC 4.9.1] on linux
Type "copyright", "credits" or "license()" for more information.
>>> 
>>> 	
>>> import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306

# Pins connected to the HC-SR04 sensor
iTriggerPin = 2
iEchoPin    = 3

GPIO.setup(iTriggerPin, GPIO.OUT)
GPIO.setup(iEchoPin, GPIO.IN)

GPIO.output(iTriggerPin, False)
time.sleep(0.5)

while True:
        GPIO.output(iTriggerPin, True)
        time.sleep(0.0001)
        GPIO.output(iTriggerPin, False)

        while GPIO.input(iEchoPin) == 0:
                pass
        fPulseStart = time.time()

        while GPIO.input(iEchoPin) == 1:
                pass
        fPulseEnd = time.time()

        fPulseDuration = fPulseEnd - fPulseStart

        fDistance = round((fPulseDuration * 17150), 2)

        print "Distance:", fDistance, "cm."

        time.sleep(0.5)

GPIO.cleanup()


