import RocketController

altitude_filter = [0]*50

SHUTDOWN_TIMER = 1800
rocket = RocketBrain("AvionicsDataLog.txt", '75335313437351B0E192', 120, SHUTDOWN_TIMER)

# Handle Prelaunch
while rocket.isNotLaunching():
    try:
        data = rocket.recieveBluetooth()
        print("received [%s]" %(data))
        message = rocket.handleInput(data)

        if (message is not 0):
            rocket.sendBluetooth(message)
        else:
            rocket.handleError('004')
    except:
        rocket.handleError('000')

# Transition from Prelaunch to Launch
rocket.closeBluetooth()
rocket.logData("--BEGIN_LAUNCH_RECORD--")
rocket.sendSlaveData("BEGIN_LAUNCH_RECORD")

# Handle Launch
# isLoggingData() will return true for however long the logging timer
# is set to run after the rocket closes bluetooth
while rocket.isLoggingData():
    try:
        # Handles reading from Arduino and writing to file
        data = rocket.getSlaveData()
        rocket.parseData()
        altitude_filter = [altitude_filter[2:length(altitude_filter)-1], rocket.ALTITUDE]
	altitude = sum(altitude_filter)/length(altitude_filter)
        # Wait for max altitude
        if altitude >= max_altitude:
            max_altitude = altitude
            max_altitude_time = time.time()
        # Wait for ejection_timing before ejecting parachute
        if (time.time() - max_altitude_time >= ejection_timing):
            rocket.sendSlaveData('f')
        rocket.logData(data)

    except:
        rocket.handleError('000')

# Shutdown after a launch period defined by SHUTDOWN_TIMER
rocket.shutDown()
