import serial
import math 
import open3d as o3d 
import numpy as np 
import os 

MYPORT = "5" # COM Port Number
OFFSET = -2000 # your offset in millimeters. '100' is 10cm

# Initialized serial communication via port
serial_comm = serial.Serial(port = "COM"+ MYPORT, baudrate = 115200, bytesize = 8, timeout = 10, stopbits = serial.STOPBITS_ONE)

# Checks if a string contains valid data by converting to a float
def verify_data(string):
    try: # If successful
        list(map(float, string.split(", ")))
        return 1
    except: # If fail
        return 0

# Reads data from the port until it receives data of at least 360 degrees
def getData(invalid_data = 1):
    data_collection = [] # Empty list to store data
    run_flag = True

    # Opens serial communication
    if not serial_comm.is_open:
        serial_comm.open()

    print("Serial port is active!")

    while run_flag:
        info = serial_comm.readline() # Read line of data from serial
        info = info.decode() # Decodes the bytes to strings
        info = info.strip("\r\n") # Removes return and newline characters

        # Checks if info received is valid
        if not verify_data(info):
            print("Invalid Info:", info)
            continue
        
        # Converts each element into floats and then into a list
        data = list(map(float, info.split(", ")))

        # Elements of data as separate variables
        rangeStatus, distance, signalRate, ambientRate, spadNb, degrees = data 

        # Checks if data is valid
        if (invalid_data) and (round(rangeStatus) != 0):
            print("Invalid data:", data)
            continue

        print("Data:", data)

        # If over 360 degrees, sets last element of data to 360 and ends loop
        if degrees >= 360:
            data[-1] = 360.0
            run_flag = False

        # Adds the data to the data collection list
        data_collection.append(data)
    
    serial_comm.close()
    print("Scan complete. Return:", data_collection)
    return data_collection

# Obtains the [x, y, z] coordinates
def get_coordinates(degrees, raw_distance, number_of_offsets):
    x = raw_distance * math.cos(math.radians(degrees))
    y = number_of_offsets * OFFSET
    z = raw_distance * math.sin(math.radians(degrees))

    return [x, y, z]

def clearExistingFile(fileName):
    if os.path.exists(fileName):
        os.remove(fileName)


# fileName = findFileName("data", "xyz") # find an untaken file name to write to
fileName = "data_hallway.xyz"
print(fileName)

coordinates = []
data = getData()

for j in range(len(data)):
    x, y, z = get_coordinates(data[j][-1], data[j][1])
    print("Coordinates [",x, y, z,"] at", data[j][-1], "degrees")

    coordinates.append([x, y, z])

# Writing the coordinates to file
with open(fileName, "a") as file:
    for j in range(len(coordinates)):
        file.write(str(coordinates[j][0])) # x coordinate
        file.write(" ")
        file.write(str(coordinates[j][1])) # y coordinate 
        file.write(" ")
        file.write(str(coordinates[j][2])) # z coordinate
        file.write("\n")