# components/lcd.py
# 16x2 I2C LCD (PCF8574 backpack, address 0x27). Only rewrites a row when its
# text changes, because I2C writes are slow. (Written fresh: lcd_safe.py from
# the coffee project was never reviewed.)

from RPLCD.i2c import CharLCD

LCD_ADDRESS = 0x27
COLS = 16

_lcd = None
_rows = ["", ""]


def setup_lcd():
    global _lcd
    _lcd = CharLCD("PCF8574", LCD_ADDRESS, cols=COLS, rows=2, charmap="A00")
    _lcd.clear()
    _rows[0] = _rows[1] = ""


def lcd_show(line1="", line2=""):
    if _lcd is None:
        return
    for i, text in enumerate((line1, line2)):
        text = str(text)[:COLS].ljust(COLS)
        if text != _rows[i]:
            try:
                _lcd.cursor_pos = (i, 0)
                _lcd.write_string(text)
                _rows[i] = text
            except Exception:
                pass    # a failed LCD write must never stop the machine


def lcd_clear():
    global _rows
    if _lcd is None:
        return
    try:
        _lcd.clear()
    except Exception:
        pass
    _rows = ["", ""]
