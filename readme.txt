~~~~~~~~~~~~~~~~~~~~~~~~~~
# Avionics Developer Notes
~~~~~~~~~~~~~~~~~~~~~~~~~~
~ Need a GUI
    - maybe
~ Need to add parachute ejection
    - Arduino side
~ Need to add "Forced Parachute Ejection" via Bluetooth for testing
    x Pi side
    - Arduino side
~ Need to add exception handling
    - Bluetooth_Client.connectToServer()

~ Need more robust STATUS report
    - Could use a little more
~ Need more robust communication between Arduino and Pi
X Add functionality to try and reconnect to services via Bluetooth Interface
    x Refactor connection startups into functions
~ Make better documentation
~ System only tested using Linux so far (Client and Server)
~ Need nondestructive file close for Datalogging
~ Once working, refactor into OOP

~~~~~~~~~~~~~~~~~~~~~~~~~~
# Avionics Developer Docs
~~~~~~~~~~~~~~~~~~~~~~~~~~
The avionics code is built in three parts. One for each device involved in the system.
For the current system, the code is built from two languages. These being C and Python.
C is used for code on the Arduino which handles the actual hardware and sensor interfacing.
Python is used for code on the Raspberry Pi (Rocket) and the client device which arms
and checks status of the rocket. The goal is to have a base station which talks mainly to the
Raspberry Pi. The Arduino and hardware interfacing should be abstracted away from the
user/operator in order to make the process simple yet still provide enough detail that
if errors occur, they can be quickly fixed. Communication between the Pi and the Arduino
is supported by the Serial/USB interface. Arduino broadcasts data over the Serial and the Pi
then reads this data and uses it for data logging and system evaluation. The Pi's main responsibilities
in this current design are for data logging, and pre flight systems check. In the future, the Pi will
play a more active role, but it is preferable to have each system as self contained as possible.
ie. If the pi is not functioning, the only flight feature that should be lost is data logging,
recovery systems should be the most stable and independent system such that only a hardware failure
of the system itself would result in an inability to deploy.

HARDWARE RESPONSIBILITIES
~~~~~~~~~~~~~~~~~~~~~~~~~
RPi (Python)
  - Data logging,
  - Mild Data Processing,
  - Bluetooth communications for systems check and launch arming.
  - Secondary recovery systems
Arduino (C)
  - Sensor interfacing,
  - data formatting,
  - hardware interface,
  - main recovery systems
Client (Python)
  - Ability to send text strings over Bluetooth to RPi

~~~~~~~~~~~~~~~~~~~~~~~~~

Some convenience features have been implemented so far but they are fairly limited.
  - Client and server will auto connect if server is started first and then the client.

Standard operating procedures will be included below once development has progressed to
a more mature state.

An important note is that unless the client receives something back from the server,
it will become unresponsive.
THE SERVER MUST SEND SOMETHING BACK ANY TIME IT RECEIVES FROM THE CLIENT.

~~~~~~~~~~~~~~~~~~~~~~~~~~
# Avionics Error Codes
~~~~~~~~~~~~~~~~~~~~~~~~~~
Error 000 - We have no idea what went wrong. Incomplete exception handling.
Error 001 - The Pi was unable to open the data logging file
Error 002 - The Pi was unable to connect to the Arduino over Serial
Error 003 - The Pi was unable to establish a Bluetooth connection
