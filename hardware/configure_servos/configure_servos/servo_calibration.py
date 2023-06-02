import RPi_LCD_16x2_I2C_driver
import RPi.GPIO as GPIO
import time

servo_pin_a = 17
servo_pin_b = 27

GPIO.setmode(GPIO.BCM)

GPIO.setup(servo_pin_a, GPIO.OUT)
GPIO.setup(servo_pin_b, GPIO.OUT)

left_leg = GPIO.PWM(servo_pin_a, 50) # GPIO 17 for PWM with 50Hz
right_leg = GPIO.PWM(servo_pin_b, 50) # GPIO 27 for PWM with 50Hz

mylcd = RPi_LCD_16x2_I2C_driver.lcd()

try:
    dc_left = 2.5
    left_leg.start(dc_left)
    dc_right = 12.5
    right_leg.start(dc_right)

    mylcd.lcd_clear()
    mylcd.lcd_display_string("Left 180 degree", 1)
    mylcd.lcd_display_string("Right 0 degree", 2)
    dc_left = 12.5
    left_leg.ChangeDutyCycle(dc_left)
    dc_right = 2.5
    right_leg.ChangeDutyCycle(dc_right)
    time.sleep(3)

    mylcd.lcd_clear()
    mylcd.lcd_display_string("Left 180 degree", 1)
    mylcd.lcd_display_string("Right 0 degree", 2)
    dc_left = 12.5
    left_leg.ChangeDutyCycle(dc_left)
    dc_right = 2.5
    right_leg.ChangeDutyCycle(dc_right)
    time.sleep(3)

    mylcd.lcd_clear()
    mylcd.lcd_display_string("Left 180 degree", 1)
    mylcd.lcd_display_string("Right 0 degree", 2)
    dc_left = 12.5
    left_leg.ChangeDutyCycle(dc_left)
    dc_right = 2.5
    right_leg.ChangeDutyCycle(dc_right)
    time.sleep(3)

    mylcd.lcd_clear()
    mylcd.lcd_display_string("Left 0 degree", 1)
    mylcd.lcd_display_string("Right 180 degree", 2)
    dc_left = 2.5
    left_leg.ChangeDutyCycle(dc_left)
    dc_right = 12.5
    right_leg.ChangeDutyCycle(dc_right)
    time.sleep(3)

    mylcd.lcd_clear()
    mylcd.lcd_display_string("Left 0 degree", 1)
    mylcd.lcd_display_string("Right 180 degree", 2)
    dc_left = 2.5
    left_leg.ChangeDutyCycle(dc_left)
    dc_right = 12.5
    right_leg.ChangeDutyCycle(dc_right)
    time.sleep(3)

    mylcd.lcd_clear()
    mylcd.lcd_display_string("Left 0 degree", 1)
    mylcd.lcd_display_string("Right 180 degree", 2)
    dc_left = 2.5
    left_leg.ChangeDutyCycle(dc_left)
    dc_right = 12.5
    right_leg.ChangeDutyCycle(dc_right)
    time.sleep(3)


    mylcd.lcd_clear()
    mylcd.lcd_display_string("Left 90 degree", 1)
    mylcd.lcd_display_string("Right 90 degree", 2)
    dc_left = 7.5
    left_leg.ChangeDutyCycle(dc_left)
    dc_right = 7.5
    right_leg.ChangeDutyCycle(dc_right)
    time.sleep(3)

    mylcd.lcd_clear()
    mylcd.lcd_display_string("Left 90 degree", 1)
    mylcd.lcd_display_string("Right 90 degree", 2)
    dc_left = 7.5
    left_leg.ChangeDutyCycle(dc_left)
    dc_right = 7.5
    right_leg.ChangeDutyCycle(dc_right)
    time.sleep(3)


    mylcd.lcd_clear()
    mylcd.lcd_display_string("Left 90 degree", 1)
    mylcd.lcd_display_string("Right 90 degree", 2)
    dc_left = 7.5
    left_leg.ChangeDutyCycle(dc_left)
    dc_right = 7.5
    right_leg.ChangeDutyCycle(dc_right)
    time.sleep(3)

finally:
    left_leg.stop()
    right_leg.stop()
    GPIO.cleanup()

