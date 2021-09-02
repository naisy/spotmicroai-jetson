import busio
from board import SCL, SDA
#from Adafruit_PCA9685 import PCA9685  # adafruit_pca9685
from adafruit_pca9685 import PCA9685 # adafruit-circuitpython-pca9685
from adafruit_motor import servo
from pick import pick
import time
import os
import sys
import Jetson.GPIO as GPIO

from spotmicroai.utilities.log import Logger
from spotmicroai.utilities.config import Config

log = Logger().setup_logger('CALIBRATE SERVOS')

log.info('Calibrate rest position...')

config_path = os.path.abspath('spotmicroai.json')
config = Config(config_path=config_path)

#gpio_port = config.get(config.ABORT_CONTROLLER_GPIO_PORT)
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(gpio_port, GPIO.OUT)
#GPIO.output(gpio_port, False)

pca = None

i2c = busio.I2C(SCL, SDA)

while True:
    options = {
        0: 'rear_shoulder_left   - PCA[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_PCA9685)) + '] CHANNEL[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_CHANNEL)) + ']  - ANGLE[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_REST_ANGLE)) + ']',
        1: 'rear_leg_left        - PCA[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_PCA9685)) + '] CHANNEL[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_CHANNEL)) + ']  - ANGLE[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_REST_ANGLE)) + ']',
        2: 'rear_feet_left       - PCA[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_PCA9685)) + '] CHANNEL[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_CHANNEL)) + ']  - ANGLE[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_REST_ANGLE)) + ']',
        3: 'rear_shoulder_right  - PCA[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_PCA9685)) + '] CHANNEL[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_CHANNEL)) + ']  - ANGLE[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_REST_ANGLE)) + ']',
        4: 'rear_leg_right       - PCA[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_PCA9685)) + '] CHANNEL[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_CHANNEL)) + ']  - ANGLE[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_REST_ANGLE)) + ']',
        5: 'rear_feet_right      - PCA[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_PCA9685)) + '] CHANNEL[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_CHANNEL)) + ']  - ANGLE[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_REST_ANGLE)) + ']',
        6: 'front_shoulder_left  - PCA[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_PCA9685)) + '] CHANNEL[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_CHANNEL)) + ']  - ANGLE[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_REST_ANGLE)) + ']',
        7: 'front_leg_left       - PCA[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_PCA9685)) + '] CHANNEL[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_CHANNEL)) + ']  - ANGLE[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_REST_ANGLE)) + ']',
        8: 'front_feet_left      - PCA[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_PCA9685)) + '] CHANNEL[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_CHANNEL)) + ']  - ANGLE[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_REST_ANGLE)) + ']',
        9: 'front_shoulder_right - PCA[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_PCA9685)) + '] CHANNEL[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_CHANNEL)) + ']  - ANGLE[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_REST_ANGLE)) + ']',
        10: 'front_leg_right      - PCA[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_PCA9685)) + '] CHANNEL[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_CHANNEL)) + ']  - ANGLE[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_REST_ANGLE)) + ']',
        11: 'front_feet_right     - PCA[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_PCA9685)) + '] CHANNEL[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_CHANNEL)) + ']  - ANGLE[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_REST_ANGLE)) + ']',
        12: 'arm_rotation         - PCA[' + str(config.get(config.ARM_CONTROLLER_SERVOS_ARM_ROTATION_PCA9685)) + '] CHANNEL[' + str(config.get(config.ARM_CONTROLLER_SERVOS_ARM_ROTATION_CHANNEL)) + ']  - ANGLE[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_ARM_ROTATION_REST_ANGLE)) + ']',
        13: 'arm_lift             - PCA[' + str(config.get(config.ARM_CONTROLLER_SERVOS_ARM_LIFT_PCA9685)) + '] CHANNEL[' + str(config.get(config.ARM_CONTROLLER_SERVOS_ARM_LIFT_CHANNEL)) + ']  - ANGLE[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_ARM_LIFT_REST_ANGLE)) + ']',
        14: 'arm_range            - PCA[' + str(config.get(config.ARM_CONTROLLER_SERVOS_ARM_RANGE_PCA9685)) + '] CHANNEL[' + str(config.get(config.ARM_CONTROLLER_SERVOS_ARM_RANGE_CHANNEL)) + ']  - ANGLE[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_ARM_RANGE_REST_ANGLE)) + ']',
        15: 'arm_cam_tilt         - PCA[' + str(config.get(config.ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_PCA9685)) + '] CHANNEL[' + str(config.get(config.ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_CHANNEL)) + ']  - ANGLE[' + str(config.get(config.MOTION_CONTROLLER_SERVOS_ARM_CAM_TILT_REST_ANGLE)) + ']'}

    title = 'The folder integration_tests for more tests' + os.linesep + \
            '1. Use "i2cdetect -y 1" to identify your i2c address' + os.linesep + \
            '2. Write your pca9685 i2c address(es) and settings in your configuration file ~/spotmicroai.json' + os.linesep + \
            '3. if no angle is specified 90 will be the default position, for example if you just press Enter' + os.linesep + \
            '' + os.linesep + \
            'Write "menu" or "m" and press Enter to return to the list of servos' + os.linesep + \
            'Write "exit" or "e" and press Enter to exit' + os.linesep + \
            '' + os.linesep + \
            'Please choose the servo to calibrate its rest position: '

    screen_options = list(options.values())

    selected_option, selected_index = pick(screen_options, title)

    PCA9685_ADDRESS, PCA9685_REFERENCE_CLOCK_SPEED, PCA9685_FREQUENCY, CHANNEL, MIN_PULSE, MAX_PULSE, REST_ANGLE = config.get_by_section_name(selected_option.split()[0])

    while True:

        try:
            user_input = input("Write the angle and press Enter, or press Enter for 90: ")
            print(f'address: {PCA9685_ADDRESS} -> {int(PCA9685_ADDRESS, 0)}')
            print(f'clock: {PCA9685_REFERENCE_CLOCK_SPEED}')
            print(f'freq: {PCA9685_FREQUENCY}')
            print(f'channel: {CHANNEL}')
            print(f'min_pulse: {MIN_PULSE}')
            print(f'max_pulse: {MAX_PULSE}')
            print(f'rest_angle: {REST_ANGLE}')

            #pca = PCA9685(address=int(PCA9685_ADDRESS, 0), i2c=i2c)
            pca = PCA9685(i2c, address=int(PCA9685_ADDRESS, 0))
            pca.frequency = PCA9685_FREQUENCY

            active_servo = servo.Servo(pca.channels[CHANNEL])
            active_servo.set_pulse_width_range(min_pulse=MIN_PULSE, max_pulse=MAX_PULSE)

            if user_input == 'menu' or user_input == 'm':
                break
            if user_input == 'exit' or user_input == 'e':
                sys.exit(0)
            elif user_input == '':
                user_input = 90

            active_servo.angle = int(user_input)
            time.sleep(0.1)
        finally:
            #pca.deinit()
            pass

