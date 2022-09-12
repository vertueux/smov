from lcd_i2c_monitor import lcd_i2c_driver
from time import *

def main():
    lcd = lcd_i2c_driver.LCD()
    lcd.lcd_display_string("Hello world", 1)

if __name__ == '__main__':
    main()