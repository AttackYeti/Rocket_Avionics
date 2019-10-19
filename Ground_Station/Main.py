from DataHandling import *
from HardwareInput import *
import curses
terminal = curses.initscr()
terminal.nodelay(True)#nonblocking terminal input
brain = GroundBrain()
COMMANDS = {'E': brain.stopRunning,
            'L': brain.stopLogging}
while brain.RUNNING:
    try:
        keyInput = terminal.getkey()
    except:
        keyInput = None
    COMMANDS.get(keyInput, lambda: None)() #run command
    brain.parseData() #receive and parse serial data
    brain.logData()

    terminal.refresh()
