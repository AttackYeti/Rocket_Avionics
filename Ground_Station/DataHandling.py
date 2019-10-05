from HardwareInput import switch
import serial
#import matplotlib.pyplot as plt
#import matplotlib.animation as anim
import csv
import time
import os.path

class GroundBrain:
    def __init__(self, SERIAL_ADDRESS='/dev/ttyS0'):

        self.DELIMITER = b'~'
        self.SERIAL_ADDRESS = SERIAL_ADDRESS
        self.DATALOGGING_ENABLED = True
        self.DATA_FILE_OBJECT = open(findOutputFile(), "w")
        self.CSV_WRITER = csv.writer(self.DATA_FILE_OBJECT)
        self.SERIAL_STREAM = serial.Serial(
            port=self.SERIAL_ADDRESS,
            baudrate = 57600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1)

        self.dataCurrent = {"ALTITUDE": 1.0, #meter? feet?
                            "ATMOSPHERE_PRESSURE": 2.0, #atm
                            "ACCEL": 3.0, #Gs
                            "TIME": 4.0, #seconds
                            "FUEL_PRESSURE": 5.0, #psi
                            "LOX_PRESSURE": 6.0, #psi
                            "Bin0": False,
                            "Bin1": True,
                            "Bin2": False,
                            "Bin3": True
                            }
        self.dataPast = {"ALTITUDE": [], #meter? feet?
                            "ATMOSPHERE_PRESSURE": [], #atm
                            "ACCEL": [], #Gs
                            "TIME": [], #seconds
                            "FUEL_PRESSURE": [], #psi
                            "LOX_PRESSURE": [], #psi
                            "Bin0": [],
                            "Bin1": [],
                            "Bin2": [],
                            "Bin3": []}
        self.initFileHeaders()

    def findOutputFile(self):
        for i in range(100):
            if not os.path.exists("DataLog{}.csv".format(i))
                break
        return i

    def initFileHeaders(self):
        self.CSV_WRITER.writerow(self.dataCurrent.keys())

    def logData(self, DATA = "DEFAULT"):
        """Logs data to a csv file"""
        if self.DATALOGGING_ENABLED:
            if DATA == "DEFAULT":
                self.CSV_WRITER.writerow(self.dataCurrent.values())# order guaranteed to line up as long as no new keys are added/removed
            else:
                self.CSV_WRITER.writerow(DATA)
            print(DATA)
        else:
            print("Datalogging Disabled. Unable to write to file.")

    def sendSlaveData(self, DATA):
        if self.SLAVE_CONNECTED:
            self.SLAVE_SERIAL_CONNECTION.write(DATA.encode())
        else:
            self.handleError('002')

    def getSerialData(self):
        message = readline()
        return message

    def parseData(self, DATA):
        parsedData = [DATA.split(self.DELIMITER)]
        print(parsedData)
        #try:
        for n in len(parsedData):
            datum = parsedData[n]
            parsedData[n] = datum.decode()
            print(parsedData)

        if parsedData[0] is 'LOG':
            self.dataCurrent["ALTITUDE"] = parsedData[1]
            self.dataCurrent["ATMOSPHERE_PRESSURE"] = parsedData[2]
            self.dataCurrent["ACCEL"] = parsedData[3]
            self.dataCurrent["TIME"] = parsedData[4]
            self.dataCurrent["FUEL_PRESSURE"] = parsedData[5]
            self.dataCurrent["LOX_PRESSURE"] = parsedData[6]
            self.dataCurrent["Bin0"] = parsedData[7]
            self.dataCurrent["Bin1"] = parsedData[8]
            self.dataCurrent["Bin2"] = parsedData[9]
            self.dataCurrent["Bin3"] = parsedData[10]
            for key, val in self.dataCurrent.items():
                self.dataPast[key].append(val)
        else:
            self.handleError('005')
        return parsedData

    def updateDummy(self):
        """Produces random dummy data"""
        for key in self.dataCurrent.keys():
            import random
            self.dataCurrent[key] = random.random()
            self.dataPast[key].append(self.dataCurrent[key])

    def testSwitch():
        while True:
            if switch(3):
                print('yeet')
                time.sleep(.2)

    def testLogData():
        x = GroundBrain()
        x.logData()
