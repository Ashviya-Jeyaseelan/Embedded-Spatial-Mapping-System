import serial # UART communication library
import math 
import open3d as o3d
import numpy as np 
import os 

MYPORT = "5" # Port number for UART communication
SCANS = 3 # Number of scans 
OFFSET = 100 # Distance moved between each scan in milimeters (in y-axis)

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

# File name
file_name = "data_demo.xyz"
print(file_name)

# Iterates for the number of scans required
for i in range (SCANS):
    coordinates = []
    data = getData()

    # The data is converted to xyz coordinates
    for j in range(len(data)):
        x, y, z = get_coordinates(data[j][-1], data[j][1], i)
        print("Coordinates [",x, y, z,"] at", data[j][-1], "degrees")
        coordinates.append([x, y, z])

    # Writing the coordinates to file
    with open(file_name, "a") as file:
        for j in range(len(coordinates)):
            file.write(str(coordinates[j][0])) # x coordinate
            file.write(" ")
            file.write(str(coordinates[j][1])) # y coordinate
            file.write(" ")
            file.write(str(coordinates[j][2])) # z coordinate
            file.write("\n")