import serial


class GroundBrain(self):

    def init(self, SERIAL_ADDRESS='/dev/ttyS0'):

        self.DELIMITER = b'~'
        self.SERIAL_ADDRESS = SERIAL_ADDRESS
        self.DATALOGGING_ENABLED = False
        self.SERIAL_STREAM = serial.Serial(
            port=self.SERIAL_ADDRESS,
            baudrate = 57600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1)

        self.dataCurrent = {"ALTITUDE": 0.0, #meter? feet?
                            "ATMOSPHERE_PRESSURE": 0.0, #atm
                            "ACCEL": 0.0, #Gs
                            "TIME": 0.0, #seconds
                            "FUEL_PRESSURE": 0.0, #psi
                            "LOX_PRESSURE": 0.0, #psi
                            "Bin0": False,
                            "Bin1": False,
                            "Bin2": False,
                            "Bin3": False
                            } 
    return

    def logData(self, DATA):
        if self.DATALOGGING_ENABLED:
            self.DATA_FILE_OBJECT.write(DATA)
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
                           
                
            else:
                self.handleError('005')

            return parsedData