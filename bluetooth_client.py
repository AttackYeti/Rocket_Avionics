import bluetooth
import sys

port = 1
connectionEnabled = True
autoconnect = True

Arduino_Serial_Number = '75335313437351B0E192'
Raspberry_Pi_Bluetooth_Address = "B8:27:EB:A6:FC:F2"

if True:
    print("Scanning for nearby bluetooth devices...")
    nearby_devices = bluetooth.discover_devices()
    print('Address : Lookup Name')
    for bdaddr in nearby_devices:
        print(bdaddr + ' : ' + bluetooth.lookup_name(bdaddr))
        if autoconnect and (Raspberry_Pi_Bluetooth_Address in nearby_devices):
            address = Raspberry_Pi_Bluetooth_Address
        else:
            address = input("Which address would you like to join?\n")

    server = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    server.connect((address, port))

else:
    # Handle an exception...
    print("Error 000 - Incomplete Exception Handling")
    print("Unable to connect...")
SKIP = False
while connectionEnabled:
    if not SKIP:
        data = server.recv(1024)
        print(data.decode())
    user_input = input(">>")
    user_input.upper()

    if len(user_input) == 0:
        print('To exit type EXIT...')
        SKIP = True

    elif user_input == "EXIT":
        print('Exiting interactive connection...')
        connectionEnabled = False

    else:
        server.sendall(user_input.encode())

server.close()
