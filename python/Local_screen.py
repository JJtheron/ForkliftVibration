import os


def open_display():
    if os.environ.get("LCD_ENABLED", "1").lower() in ("0", "false", "no"):
        return None
    try:
        from RPLCD.i2c import CharLCD
        return CharLCD(
            i2c_expander="PCF8574",
            address=int(os.environ.get("LCD_ADDRESS", "0x27"), 0),
            port=1,
            cols=16,
            rows=2,
        )
    except Exception as exc:
        print(f"LCD unavailable: {exc}")
        return None


def show(display, line_one, line_two):
    if display is None:
        return
    try:
        display.clear()
        display.write_string(line_one[:16].ljust(16))
        display.cursor_pos = (1, 0)
        display.write_string(line_two[:16].ljust(16))
    except Exception as exc:
        print(f"LCD update failed: {exc}")
