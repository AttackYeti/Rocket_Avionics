import sys
sys.path.append('usr/bin/python3')

import bluetooth
import serial
import time
import os
import warnings
import serial.tools.list_ports

class RocketBrain:
    #def __init__(self, DATA_FILE_NAME, SLAVE_SERIAL_NUMBER, ARMING_DELAY, SHUTDOWN_TIMER):
    def __init__(self, DATA_FILE_NAME, SLAVE_SERIAL_NUMBER, SHUTDOWN_TIMER):
        self.LAUNCH_ARMED = False
        self.DATALOGGING_ENABLED = False
        self.SLAVE_CONNECTED = False
        self.BLUETOOTH_CONNECTED = False
        self.PARACHUTE_DEPLOYED = False
        self.ALTIMETER_ENABLED = False
        self.ALTITUDE = 0
        self.TEMPERATURE = 0
        self.ARM_TIME = 0
        self.SLAVE_SERIAL_SPEED = 115200
        self.BLUETOOTH_PORT = 1
        self.STARTUP_STATUS = 0
        self.SERVER = 0
        self.CLIENT = 0
        self.DATA_FILE_OBJECT = 0
        self.SLAVE_SERIAL_CONNECTION = 0
        self.DATA_FILE_NAME = DATA_FILE_NAME
        self.SLAVE_SERIAL_NUMBER = SLAVE_SERIAL_NUMBER
        #self.ARMING_DELAY = ARMING_DELAY
        self.SHUTDOWN_TIMER = SHUTDOWN_TIMER
        self.openDataLog()
        self.connectSlave()

    def findSlave(self):
        for connection in serial.tools.list_ports.comports():
            if connection.serial_number == self.SLAVE_SERIAL_NUMBER:
                return serial.Serial(connection.device, self.SLAVE_SERIAL_SPEED)
        raise IOError("Could not find Slave...")

    def connectSlave(self):
        if not self.SLAVE_CONNECTED:
            try:
                self.SLAVE_SERIAL_CONNECTION = self.findSlave()
                self.SLAVE_CONNECTED = True
                print("Slave Connected")
            except:
                self.handleError('002')
        else:
            print('Slave already connected')

    def openDataLog(self):
        try:
            self.DATA_FILE_OBJECT = open(self.DATA_FILE_NAME,"a")
            self.DATALOGGING_ENABLED = True
            self.logData(time.strftime("%c"))
            print("Datalog Opened")
        except:
            self.handleError('001')
            self.DATALOGGING_ENABLED = False

    def getArduinoStatus(self):
        if self.SLAVE_CONNECTED:
            self.SLAVE_SERIAL_CONNECTION.write('s')
            delay(0.1)
            self.STARTUP_STATUS = self.SLAVE_SERIAL_CONNECTION.readline()

    def connectBluetooth(self):
        try:
            self.SERVER=bluetooth.BluetoothSocket(bluetooth.RFCOMM)

            # Bind port and listen for incoming connection
            self.SERVER.bind(("",self.BLUETOOTH_PORT))
            print('Socket bound...')
            self.SERVER.listen(1)
            print('Listening on socket 1...')
            self.CLIENT, address = self.SERVER.accept()
            print('Accepted connection from ',address)
            self.CLIENT.send("CONNECTED")
            self.BLUETOOTH_CONNECTED = True
        except:
            print('Exception raised')
            self.BLUETOOTH_CONNECTED = False

    def closeBluetooth(self):
        print("Closing Bluetooth connection...")
        self.CLIENT.close()
        self.SERVER.close()
        self.BLUETOOTH_CONNECTED = False

    def recieveBluetooth(self):
        if self.BLUETOOTH_CONNECTED:
            try:
                DATA = self.CLIENT.recv(1024)
                return DATA.decode()
            except:
                return 0
        else:
            self.handleError('003')
            return 0

    def sendBluetooth(self, DATA):
        self.CLIENT.send(DATA)

    def logData(self, DATA):
        if self.DATALOGGING_ENABLED:
            self.DATA_FILE_OBJECT.write(DATA)
            print(DATA)
        else:
            print("Datalogging Disabled. Unable to write to file.")

    def getSlaveData(self):
        if self.SLAVE_CONNECTED:
            DATA = self.SLAVE_SERIAL_CONNECTION.readline()
            return DATA
        else:
            self.handleError('002')
            return 0

    def sendSlaveData(self, DATA):
        if self.SLAVE_CONNECTED:
            self.SLAVE_SERIAL_CONNECTION.write(DATA.encode())
        else:
            self.handleError('002')

    def handleError(self, errorNumber):
        if (errorNumber is '000'):
            message = 'Error 000, Something Broke...'

        elif (errorNumber is '001'):
            message = 'Error 001, Unable to open file...'

        elif (errorNumber is '002'):
            message = 'Error 002, Unable to connect to slave...'

        elif (errorNumber is '003'):
            message = 'Error 003, Unable to Establish Bluetooth Connection...'

        elif (errorNumber is '004'):
            message = 'Error 004, Unable to parse input...'

        elif (errorNumber is '005'):
            message = 'Error 005, Communication with slave is corrupt...'

        else:
            message = 'Error Unknown, Something Broke...'

        print(message)
        if (self.BLUETOOTH_CONNECTED):
            self.sendBluetooth(message)
        if (self.DATALOGGING_ENABLED):
            self.logData(message)

    def shutDown(self):
        self.SLAVE_SERIAL_CONNECTION.close()
        self.DATA_FILE_OBJECT.close()

    def handleInput(self, INPUT):
        if INPUT is [0]:
            return 0
        if INPUT is 0:
            return 0
        if len(INPUT) > 0:
            self.logData(INPUT)

            if INPUT == "LAUNCH_READY":
                # Put arming sequence here
                self.LAUNCH_ARMED = True
                message = "Rocket armed..."
                print("LAUNCH_ARMED: {}".format(self.LAUNCH_ARMED))

            elif INPUT == "DISARM":
                self.LAUNCH_ARMED = False
                message = "Disarmed..."
                print("LAUNCH_ARMED: {}".format(self.LAUNCH_ARMED))

            elif INPUT == "FORCE_DEPLOY":
                self.sendSlaveData('f')
                self.PARACHUTE_DEPLOYED = True
                message = "Forcing Parachute Deployment..."

            elif INPUT == "CLOSE_BLUETOOTH":
                self.ARM_TIME = time.time()
                self.BLUETOOTH_CONNECTED = False
                print("Closing current bluetooth connection...")
                self.closeBluetooth()


            elif INPUT == "CONNECT_ARDUINO":
                if self.SLAVE_CONNECTED:
                    message = "Pi-Arduino Connection Already Established..."
                else:
                    self.connectSlave()
                    if self.SLAVE_CONNECTED:
                        message = "Successfully Connected to Arduino..."
                    else:
                        message = "Unable to establish Pi-Arduino Connection..."

            elif INPUT == "BEGIN_DATALOG":
                if self.DATALOGGING_ENABLED:
                    message = "Pi datalogging already enabled and open..."
                else:
                    self.openDataLog()
                    if self.DATALOGGING_ENABLED:
                        message = "Successfully began datalogging..."
                    else:
                        message = "Unable to begin datalogging..."

            else:
                # Put catch all functionality here
                if self.LAUNCH_ARMED == True:
                    self.LAUNCH_ARMED = False
                    message = "Disarmed..."
                else:
                    message = "ENTER A VALID COMMAND"

            return message
        else:
            return 0

    def isNotLaunching(self):
        return (not self.LAUNCH_ARMED or self.BLUETOOTH_CONNECTED)
        #return (not self.LAUNCH_ARMED) or (self.LAUNCH_ARMED and ((time.time() - self.ARM_TIME) < self.ARMING_DELAY))

    def isLoggingData(self):
        return (self.LAUNCH_ARMED and (time.time()-self.ARM_TIME) < self.SHUTDOWN_TIMER)

    def parseData(self, DATA):
        parsedData = [DATA.split(b'~')]
        print(parsedData)
        #try:
        for n in len(parsedData):
            datum = parsedData[n]
            parsedData[n] = datum.decode()
            print(parsedData)

        if parsedData[0] is 'LOG':
            self.ALTITUDE = parsedData[1]
            self.PRESSURE = parsedData[2]
            self.TEMPERATURE = parsedData[3]
            self.PARACHUTE_DEPLOYED = parsedData[4]
            self.ALTIMETER_ENABLED = parsedData[5]
        else:
            self.handleError('005')

        return parsedData

#        except TypeError:
#            pass
#            return 0
#        except:
#            self.handleError('005')
#            return 0
