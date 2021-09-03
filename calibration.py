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

gpio_port = Config().get(Config.ABORT_CONTROLLER_GPIO_PORT)
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)
GPIO.setup(gpio_port, GPIO.OUT)
GPIO.output(gpio_port, False)

pca = None

i2c = busio.I2C(SCL, SDA)

def update_spotmicroai_json_value(json_values, calib_value):
    new_value = {'pca9685':calib_value['pca9685'], 'channel':calib_value['channel'], 'min_pulse':calib_value['min_pulse'], 'max_pulse':calib_value['max_pulse'], 'rest_angle':calib_value['rest_angle']}
    json_values['motion_controller'][0]['servos'][0][calib_value['servo']][0].update(new_value)
    return json_values

while True:
    options = {
        0: 'rear_shoulder_left   - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_REST_ANGLE)) + ']',
        1: 'rear_leg_left        - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_REST_ANGLE)) + ']',
        2: 'rear_feet_left       - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_REST_ANGLE)) + ']',
        3: 'rear_shoulder_right  - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_REST_ANGLE)) + ']',
        4: 'rear_leg_right       - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_REST_ANGLE)) + ']',
        5: 'rear_feet_right      - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_REST_ANGLE)) + ']',
        6: 'front_shoulder_left  - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_REST_ANGLE)) + ']',
        7: 'front_leg_left       - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_REST_ANGLE)) + ']',
        8: 'front_feet_left      - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_REST_ANGLE)) + ']',
        9: 'front_shoulder_right - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_REST_ANGLE)) + ']',
        10: 'front_leg_right      - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_REST_ANGLE)) + ']',
        11: 'front_feet_right     - PCA[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_REST_ANGLE)) + ']',
        12: 'arm_rotation         - PCA[' + str(Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_ROTATION_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_ROTATION_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_ROTATION_REST_ANGLE)) + ']',
        13: 'arm_lift             - PCA[' + str(Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_LIFT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_LIFT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_LIFT_REST_ANGLE)) + ']',
        14: 'arm_range            - PCA[' + str(Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_RANGE_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_RANGE_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_RANGE_REST_ANGLE)) + ']',
        15: 'arm_cam_tilt         - PCA[' + str(Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_PCA9685)) + '] CHANNEL[' + str(Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_CHANNEL)) + ']  - ANGLE[' + str(Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_CAM_TILT_REST_ANGLE)) + ']'}

    title = 'The folder integration_tests for more tests' + os.linesep + \
            '1. Use "i2cdetect -y 1" to identify your i2c address' + os.linesep + \
            '2. Write your pca9685 i2c address(es) and settings in your configuration file ~/spotmicroai.json' + os.linesep + \
            '3. if no angle is specified 90 will be the default position, for example if you just press Enter' + os.linesep + \
            '' + os.linesep + \
            'Write "menu" or "m" and press Enter to return to the list of servos' + os.linesep + \
            'Write "save" or "s" and press Enter to save configuration' + os.linesep + \
            'Write "exit" or "e" and press Enter to exit' + os.linesep + \
            '' + os.linesep + \
            'Please choose the servo to calibrate its rest position: '

    screen_options = list(options.values())

    selected_option, selected_index = pick(screen_options, title)

    PCA9685_ADDRESS, PCA9685_REFERENCE_CLOCK_SPEED, PCA9685_FREQUENCY, CHANNEL, MIN_PULSE, MAX_PULSE, REST_ANGLE = Config().get_by_section_name(selected_option.split()[0])

    # Define calibration part of json file
    calib_dict = []
    calib_dict.append({'servo':'rear_shoulder_left', 'pca9685':Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_PCA9685), 'channel':Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_CHANNEL), 'min_pulse':MIN_PULSE, 'max_pulse':MAX_PULSE, 'rest_angle':Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_REST_ANGLE)})
    calib_dict.append({'servo':'rear_leg_left', 'pca9685':Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_PCA9685), 'channel':Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_CHANNEL), 'min_pulse':MIN_PULSE, 'max_pulse':MAX_PULSE, 'rest_angle':Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_REST_ANGLE)})
    calib_dict.append({'servo':'rear_feet_left', 'pca9685':Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_PCA9685), 'channel':Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_CHANNEL), 'min_pulse':MIN_PULSE, 'max_pulse':MAX_PULSE, 'rest_angle':Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_REST_ANGLE)})
    calib_dict.append({'servo':'rear_shoulder_right', 'pca9685':Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_PCA9685), 'channel':Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_CHANNEL), 'min_pulse':MIN_PULSE, 'max_pulse':MAX_PULSE, 'rest_angle':Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_REST_ANGLE)})
    calib_dict.append({'servo':'rear_leg_right', 'pca9685':Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_PCA9685), 'channel':Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_CHANNEL), 'min_pulse':MIN_PULSE, 'max_pulse':MAX_PULSE, 'rest_angle':Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_REST_ANGLE)})
    calib_dict.append({'servo':'rear_feet_right', 'pca9685':Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_PCA9685), 'channel':Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_CHANNEL), 'min_pulse':MIN_PULSE, 'max_pulse':MAX_PULSE, 'rest_angle':Config().get(Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_REST_ANGLE)})
    calib_dict.append({'servo':'front_shoulder_left', 'pca9685':Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_PCA9685), 'channel':Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_CHANNEL), 'min_pulse':MIN_PULSE, 'max_pulse':MAX_PULSE, 'rest_angle':Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_REST_ANGLE)})
    calib_dict.append({'servo':'front_leg_left', 'pca9685':Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_PCA9685), 'channel':Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_CHANNEL), 'min_pulse':MIN_PULSE, 'max_pulse':MAX_PULSE, 'rest_angle':Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_REST_ANGLE)})
    calib_dict.append({'servo':'front_feet_left', 'pca9685':Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_PCA9685), 'channel':Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_CHANNEL), 'min_pulse':MIN_PULSE, 'max_pulse':MAX_PULSE, 'rest_angle':Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_REST_ANGLE)})
    calib_dict.append({'servo':'front_shoulder_right', 'pca9685':Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_PCA9685), 'channel':Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_CHANNEL), 'min_pulse':MIN_PULSE, 'max_pulse':MAX_PULSE, 'rest_angle':Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_REST_ANGLE)})
    calib_dict.append({'servo':'front_leg_right', 'pca9685':Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_PCA9685), 'channel':Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_CHANNEL), 'min_pulse':MIN_PULSE, 'max_pulse':MAX_PULSE, 'rest_angle':Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_REST_ANGLE)})
    calib_dict.append({'servo':'front_feet_right', 'pca9685':Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_PCA9685), 'channel':Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_CHANNEL), 'min_pulse':MIN_PULSE, 'max_pulse':MAX_PULSE, 'rest_angle':Config().get(Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_REST_ANGLE)})
    calib_dict.append({'servo':'arm_rotation', 'pca9685':Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_ROTATION_PCA9685), 'channel':Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_ROTATION_CHANNEL), 'min_pulse':MIN_PULSE, 'max_pulse':MAX_PULSE, 'rest_angle':Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_ROTATION_REST_ANGLE)})
    calib_dict.append({'servo':'arm_lift', 'pca9685':Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_LIFT_PCA9685), 'channel':Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_LIFT_CHANNEL), 'min_pulse':MIN_PULSE, 'max_pulse':MAX_PULSE, 'rest_angle':Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_LIFT_REST_ANGLE)})
    calib_dict.append({'servo':'arm_range', 'pca9685':Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_RANGE_PCA9685), 'channel':Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_RANGE_CHANNEL), 'min_pulse':MIN_PULSE, 'max_pulse':MAX_PULSE, 'rest_angle':Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_RANGE_REST_ANGLE)})
    calib_dict.append({'servo':'arm_cam_tilt', 'pca9685':Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_PCA9685), 'channel':Config().get(Config.ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_CHANNEL), 'min_pulse':MIN_PULSE, 'max_pulse':MAX_PULSE, 'rest_angle':Config().get(Config.MOTION_CONTROLLER_SERVOS_ARM_CAM_TILT_REST_ANGLE)})

    # prepare selected PCA9685
    print("Preparing the pca9685 board...")
    pca = PCA9685(i2c, address=int(PCA9685_ADDRESS, 0))  # super slow. 10s
    pca.frequency = PCA9685_FREQUENCY


    json_values = Config().load_config()
    while True:
        try:
            user_input = input("Write the angle 0-180 and press Enter. 'm' back to menue, 's' save, 'q' exit: ")
            """
            print(f'address: int({PCA9685_ADDRESS}): {int(PCA9685_ADDRESS, 0)}')
            print(f'clock: {PCA9685_REFERENCE_CLOCK_SPEED}')
            print(f'freq: {PCA9685_FREQUENCY}')
            print(f'channel: {CHANNEL}')
            print(f'min_pulse: {MIN_PULSE}')
            print(f'max_pulse: {MAX_PULSE}')
            print(f'rest_angle: {REST_ANGLE}')
            """

            if user_input == 'menu' or user_input == 'm':
                break
            elif user_input == 'save' or user_input == 's':
                print('saving...')
                Config().save_config(json_values)
            elif user_input == 'exit' or user_input == 'e':
                sys.exit(0)
            elif user_input == '':
                # do nothing
                continue
            elif user_input.isdecimal:
                #print(f'int: {user_input}')
                angle = int(user_input)
                if 0 <= angle and angle <= 180:
                    active_servo = servo.Servo(pca.channels[CHANNEL])
                    active_servo.set_pulse_width_range(min_pulse=MIN_PULSE, max_pulse=MAX_PULSE)
                    active_servo.angle = int(angle)
                    calib_dict[selected_index].update({'rest_angle':angle})
                    json_values = update_spotmicroai_json_value(json_values, calib_dict[selected_index])
                    time.sleep(0.1)
                else:
                    print('Out of range. 0-180.')
            else:
                print(f'Unknown input: {user_input}')

        finally:
            #pca.deinit()
            pass

