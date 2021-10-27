import time
import serial
import csv
import math
import paho.mqtt.client as mqtt

# Data Storage
profileLocation = ""
profileSEA = 0.00
profileSE = 0.00
profileBR = 0.00
profileMO = 0.00
profileCE = 0.00
profileCoordinates = []
profileEnvelopeOriginal = []
profileEnvelopeAdjusted = []

# Calibration
CoordinateOffsetX = 0.00 # Origin Offset of Scanner
CooridanteOffsetY = 0.00 # Origin Offset of Scanner


# Serial Setup
encoderSerial = serial.Serial('/dev/ttyACM0', 115200, timeout=1) # TODO - Change Driver Name
#scannerSerial = serial.Serial('COM4', 115200, timeout=1) # TODO - Change Driver Name
encoderSerial.flush()
#scannerSerial.flush()

# MQTT Setup
mqttBroker = "192.168.5.1"
mqttClient = mqtt.Client("RPi1")
def onMessage(client, userdata, message):
    topic = message.topic
    command = message.payload.decode("utf-8")
    print("MQTT: ", message.topic, " - ", message.payload.decode("utf-8"))

    
mqttClient.on_message=onMessage
mqttClient.connect(mqttBroker)
mqttClient.loop_start()
mqttClient.subscribe("cvs/external/command") #Commands from tablet

def EncoderSerialWrite(message):
    # Function used to push messages to Encoder Assembly Arduino
    encoderSerial.write(bytes(message, 'utf-8'))
    print("Message Pushed:", message)

def ScannerSerialWrite(message):
    # Function used to push messages to Sensor Head Arduino
    scannerSerial.write(bytes(message, 'utf-8'))
    print("Message Pushed: ", message)

#################### Execute Task Functions #####################

def ExecuteTaskHomeMotor():
    # Moves motor until in home position
    ScannerSerialWrite("ETHM")
    while not (scannerSerial.in_waiting > 0):
        time.sleep(0.1)
    response = scannerSerial.readline().decode('utf-8').rstrip()

def ExecuteTaskScanProfile(resolution, range):
    # Performs a 360* scan of tunnel profile
    global profileCoordinates
    message = "ETSP:" + resolution + "|" + range
    points = range / resolution
    profileCoordinates = []
    if (int)(range/resolution) < (float)(range/resolution):
        points += 1
    ScannerSerialWrite(message)
    for x in range(1, points):
        while not (scannerSerial.in_waiting > 0):
            time.sleep(0.05)
        response = scannerSerial.readline().decode('utf-8').rstrip()
        break1 = response.index(":")
        break2 = response.index(":", break1+1)
        pt = response[0:break1]
        x = response[break1:break2]
        y = response[break2:]
        profileCoordinates.append([pt, x, y])
    

def ExecuteReadSuperElevation():
    global profileSE
    global profileSEA
    EncoderSerialWrite("ERSE")
    while not (encoderSerial.in_waiting > 0):
        time.sleep(0.1)
    response = encoderSerial.readline().decode('utf-8').rstrip()
    #print(response)
    value = response[5:] #DPSE:##.##
    #print(value)
    profileSEA = float(value)
    profileSE = 56.5 * (math.sin(profileSEA * (2.00 * math.pi / 360.00)))


def ExecuteTaskBendRadius():
    global profileBR
    global profileMO
    global profileCE
    leResponse = 0.00
    reResponse = 0.00
    # Get Left
    EncoderSerialWrite('ERLE')
    while not (encoderSerial.in_waiting > 0):
        time.sleep(0.1)
    response = encoderSerial.readline().decode('utf-8').rstrip()
    #print(response)
    leResponse = float(response[5:])
    # Get Right
    EncoderSerialWrite("ERRE")
    while not (encoderSerial.in_waiting > 0):
        time.sleep(0.1)
    response = encoderSerial.readline().decode("utf-8").rstrip()
    reResponse = float(response[5:])
    # Calc Radius and Middle Ordinate
    lx = -(25*12) * math.cos(leResponse * (2*math.pi/360.00))
    ly = (25*12) * math.sin(leResponse * (2*math.pi/360.00))
    rx = (25*12) * math.cos(reResponse * (2*math.pi/360.00))
    ry = (25*12) * math.sin(reResponse * (2*math.pi/360.00))
    slope = (ry - ly) / (rx - lx)
    yint = ry - (slope * rx)
    radius = -1
    ce = -1
    if (yint != 0):
        radius = (1.5 * 50 * 50) / yint
        ce = 4374/radius
    profileMO = yint
    profileBR = radius
    profileCE = ce
    # Update Violation Profile
    InternalTaskUpdateEnvelope()
    # Push result to user
    #print("Left: ", leResponse)
    #print("Right: ", reResponse)

#################### Execute Read Functions #####################

def updateExternal():
    # Notify clients of inbound data update
    mqttClient.publish("cvs/external/DATA", "DATA_REFRESH")
    # Super Elevation
    global profileSEA, profileSE
    mqttClient.publish("cvs/external/SE", str(profileSE))
    mqttClient.publish("cvs/external/SEA", str(profileSEA))
    # Bend Radius
    global profileBR, profileMO
    mqttClient.publish("cvs/external/BR", str(profileBR))
    mqttClient.publish("cvs/external/MO", str(profileMO))
    # Scan Data
    global profileCoordinates, profileEnvelopeAdjusted
    for coord in profileCoordinates:
        payload = str(coord[0]) + "," + str(coord[1]) + "," + str(coord[2])
        mqttClient.publish("cvs/external/SP", payload)
    for coord in profileEnvelopeAdjusted:
        payload = str(coord[0]) + "," + str(coord[1]) + "," + str(coord[2])
        mqttClient.publish("cvs/external/EP", payload)
    #HM - Motor Position, Home State

def InternalTaskUpdateEnvelope():
    global profileEnvelopeAdjusted
    global profileEnvelopeOriginal
    global profileSEA
    global profileMO


    profileEnvelopeAdjusted = []
    for coord in profileEnvelopeOriginal:
        ID = coord[0]
        X = coord[1]
        Y = coord[2]
        radiusPol = math.sqrt(math.pow(X, 2) + math.pow(Y, 2))
        anglePolD = math.atan2(Y,X) * (360.00 / (2 * math.pi))
        anglePolD = anglePolD + profileSEA
        newX = radiusPol * math.cos(anglePolD * (2 * math.pi / 360.00))
        newY = radiusPol * math.sin(anglePolD * (2 * math.pi / 360.00))
        newX = newX + profileMO
        profileEnvelopeAdjusted.append([ID, newX, newY])



# Command Dictionary
cvsCommands = {"ETHM": ExecuteTaskHomeMotor,
               "ETSP": ExecuteTaskScanProfile,
               "ERSE": ExecuteReadSuperElevation,
               "ETBR": ExecuteTaskBendRadius,
               "ITUE": InternalTaskUpdateEnvelope}


if __name__ == "__main__":
    # Main Function

    # Upload all background data
    with open('CVS_ENVELOPE_COORDS.csv') as csv_file:
        csvReader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csvReader:
            if line_count == 0:
                columns = row
                line_count += 1
            else:
                ID = row[0]
                X = float(row[1])
                Y = float(row[2])
                profileEnvelopeOriginal.append([ID, X, Y])
                line_count += 1

    # Greet User
    print("CVS - RPi Script")
    print("Version 0.1")

    while True:
        print("\nETHM - Execute Task Home Motor")
        print("ERSE - Execute Read Super Elevation")
        print("ETBR - Execute Task Bend Radius")
        print("ETSP - Execute Task Scan Profile\n")

        userIn = input("Enter CVS Command:")
        #try:
        cmd = userIn[0:3]
        range = 0.00
        resolution = 0.00
        if len(userIn) > 4:
            br = userIn.index(":", 5)
            range = userIn[5:br]
            resolution = userIn[br+1:]
        function = cvsCommands[userIn]
        function()
        InternalTaskUpdateEnvelope()
        updateExternal()
        #except:
            #print("Invalid Command")