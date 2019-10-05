def switch(pin):
    import RPi.GPIO as GPIO
    import time

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    input_state = GPIO.input(pin)
    if not input_state:
        # print('switch is on')
        # time.sleep(0.2)
        return True
    return False
