
class GroundBrain(self):

    def init(self):

        self.DATALOGGING_ENABLED = False

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

    def parseData(self, DATA):
            parsedData = [DATA.split(b'~')]
            print(parsedData)
            #try:
            for n in len(parsedData):
                datum = parsedData[n]
                parsedData[n] = datum.decode()
                print(parsedData)

            if parsedData[0] is 'LOG':
                
                """
                self.ALTITUDE = parsedData[1]
                self.PRESSURE = parsedData[2]
                self.TEMPERATURE = parsedData[3]
                self.PARACHUTE_DEPLOYED = parsedData[4]
                self.ALTIMETER_ENABLED = parsedData[5]
                """
            else:
                self.handleError('005')

            return parsedData