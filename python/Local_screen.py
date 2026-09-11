from RPLCD.i2c import CharLCD
from time import sleep

lcd = CharLCD(
    i2c_expander='PCF8574',
    address=0x27,      # change if needed
    port=1,
    cols=16,
    rows=2
)

lcd.clear()
lcd.write_string('Hello JJ!')
lcd.cursor_pos = (1, 0)
lcd.write_string('GPS Starting...')
