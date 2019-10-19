def switch(pin):
    """return if switch is on or off"""
    import RPi.GPIO as GPIO
    import time

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    input_state = GPIO.input(pin)
    if not input_state:
        return True
    return False
