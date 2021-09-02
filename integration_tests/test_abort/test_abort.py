#!/home/pi/spotmicroai/venv/bin/python3 -u

import RPi.GPIO as GPIO
import time

from spotmicroai.utilities.log import Logger
from spotmicroai.utilities.config import Config

log = Logger().setup_logger('Test Abort')

log.info('Testing abort mechanism...')

gpio_port = Config().get('abort_controller[0].gpio_port')

log.info('Make sure you have connected your GPIO pin to the 0E port in the PCA9685 boards')
log.info('GPIO information for RaspberryPi can be found here: ')
log.info('     https://www.raspberrypi.org/documentation/usage/gpio/')
log.info('Current configuration value is: ' + str(gpio_port))
input("Press Enter to start the tests...")

try:

    GPIO.cleanup()
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(gpio_port, GPIO.OUT)

    GPIO.cleanup()

    GPIO.output(gpio_port, True)
    time.sleep(2)

finally:
    # GPIO.cleanup()
    pass
