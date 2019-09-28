import bluetooth
import serial
import time
import os
import warnings
import serial.tools.list_ports

# Important Flags
LAUNCH_ARMED = False
DATALOGGING_ENABLED = False
ARDUINO_CONNECTED = False
BLUETOOTH_CONNECTED = False
PARACHUTE_DEPLOYED = False
MPU_CALIBRATED = False
ALTIMETER_ENABLED = False
GPS_ENABLED = False

# Variables to control state flow
arming_delay = 120
arm_time = 0

def findArduino(SerialNumber, SerialSpeed):
    for p in serial.tools.list_ports.comports():
        if p.serial_number == SerialNumber:
            return serial.Serial(p.device, SerialSpeed)
    raise IOError("Could not find Arduino...")

def connectArduino():
    try:
        ser = findArduino(Arduino_Serial_Number, SerialSpeed)
        ARDUINO_CONNECTED = True
        return ser
    except:
        print("Error 002 - Arduino Not Found")
        ARDUINO_CONNECTED =  False
        return 0

def openDataLog():
    try:
        file = open(fileName,"a")
        file.write(time.strftime("%c"))
        DATALOGGING_ENABLED = True
        return file
    except:
        print("Error 001 - Unable to open file: %s" %(fileName))
        DATALOGGING_ENABLED = False
        return 0

def getArduinoStatus(ser):
    if ARDUINO_CONNECTED:
        ser.write('STATUS')
        delay(2)
        line = ser.readline()


# Handle Datalogging file
fileName = "AvionicsDataLog.txt"
file = openDataLog()

# Handle Arduino Serial connection
Arduino_Serial_Number = '75335313437351B0E192'
SerialSpeed = 115200;
ser = connectArduino()

# Handle Bluetooth socket
port = 1
try:
    server_sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM)

    # Bind port and listen for incoming connection
    server_sock.bind(("",port))
    print('Socket bound...')
    server_sock.listen(1)
    print('Listening on socket 1...')
    client_sock, address = server_sock.accept()
    print("Accepted connection from ",address)
    client_sock.send("CONNECTED")
    BLUETOOTH_CONNECTED = True
except:
    print('Error 003 - Unable to Establish Bluetooth Connection')

while (not LAUNCH_ARMED) or (LAUNCH_ARMED and ((time.time() - arm_time) < arming_delay)):
    try:
        data = client_sock.recv(1024)
        data = data.decode()
        if len(data) > 0:
            print("received [%s]" % data)
            if DATALOGGING_ENABLED:
                file.write("recieved" + data)

            '''
                Everything above here is just boilerplate to get the two devices
                to connect to one another and set up the connection with the Arduino.
                DO NOT CHANGE ANYTHING... Unless you switch out the arduino board
                and need to change the very obvious Arduino Serial Number. Or if your name is Eddie.

                Any functionality should be added below as an additional elif statement,
                You can also find at the bottom an area for any functions that
                should happen after the connection has ended and the rocket is operating
                on its own.

            '''

            if data == "STATUS":
                # Put status output here
                message =   """
                                Connected to: %s            \n
                                DATALOGGING_ENABLED = %b    \n
                                LAUNCH_ARMED = %b           \n
                                ARDUINO_CONNECTED = %b      \n
                                PARACHUTE_DEPLOYED = %b     \n
                                MPU Calibrated = %b         \n
                                Altimeter enabled = %b      \n
                                Current Altitude = %s       \n
                                Current GPS Reading = %s    \n


                            """ %(ser.portstr, DATALOGGING_ENABLED, LAUNCH_ARMED, ARDUINO_CONNECTED, PARACHUTE_DEPLOYED, MPU_CALIBRATED, ALTIMETER_ENABLED, ~, ~)
                client_sock.send(message)

            elif data == "LAUNCH_READY":
                # Put arming sequence here
                client_sock.send("Rocket armed...")
                LAUNCH_ARMED = True
                arm_time = time.time()

            elif data == "DEBUG":
                # Put Debugging info here
                message =   """
                                Debug...\n
                                Connected to: %s            \n
                                DATALOGGING_ENABLED = %b    \n
                                LAUNCH_ARMED = %b           \n
                                ARDUINO_CONNECTED = %b      \n
                                PARACHUTE_DEPLOYED = %b     \n
                            """ %(ser.portstr, DATALOGGING_ENABLED, LAUNCH_ARMED, ARDUINO_CONNECTED, PARACHUTE_DEPLOYED)
                client_sock.send(message)

            elif data == "DISARM":
                LAUNCH_ARMED = False
                client_sock.send("Disarmed...")

            elif data == "FORCE_DEPLOY":
                client_sock.send("Forcing Parachute Deployment")
                delay(3)
                ser.write('FORCE_DEPLOY')
                PARACHUTE_DEPLOYED = True

            elif data == "CONNECT_ARDUINO":
                if ARDUINO_CONNECTED is True:
                    client_sock.send("Pi-Arduino Connection Already Established...")
                else:
                    ser = connectArduino()
                    if ARDUINO_CONNECTED:
                        client_sock.send("Successfully Connected to Arduino...")
                    else:
                        client_sock.send("Unable to establish Pi-Arduino Connection...")

            elif data == "BEGIN_DATALOG":
                if DATALOGGING_ENABLED is True:
                    client_sock.send("Pi datalogging already enabled and open...")
                else:
                    file = openDataLog()
                    if DATALOGGING_ENABLED:
                        client_sock.send("Successfully began datalogging...")
                    else:
                        client_sock.send("Unable to begin datalogging...")

            else:
                # Put catch all functionality here
                if LAUNCH_ARMED == True:
                    LAUNCH_ARMED = False
                    client_sock.send("Disarmed...")

                else:
                    client_sock.send("ENTER A VALID COMMAND")

    except:
        print("Error 000 - Something Broke")

print("Closing Bluetooth connection...")
client_sock.close()
server_sock.close()

if DATALOGGING_ENABLED:
    file.write("--BEGIN_LAUNCH_RECORD--")

i = 0
# Might be a good idea to have some conditional that checks for some sort of end state.
while True:
    '''
        This section will run once the system is armed. Currently the loop
        will run indefinitely. However an obvious downside of this is that
        there is a possibility of data corruption if power is lost during
        a write. A goal is to find a way to avoid this.

        Idea: write data in 'chunks' where each chunk is a new file so that
        in the event of data corruption, the loss is compartmentalized and only a
        small chunk is lost.
    '''
    try:
        i += 1

        # Handles reading from Arduino and writing to file
        if DATALOGGING_ENABLED and ARDUINO_CONNECTED:
            line = ser.readline()
            file.write(line.decode() + "\n")

            # This limiter can be raised in order to speed up execution
            if i > 250:
                file.flush()
                os.fsync(file.fileno())

        # Put additional code below here

        # Put additional code above here

    except:
        # This could use some better error handling
        print("Error 000")
        print("Something broke")
        ser.close()
        file.close()
