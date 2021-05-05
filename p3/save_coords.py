import serial
import json
import time
import numpy as np
from numpy.linalg import inv
from sklearn import neighbors

import sys


if len(sys.argv) == 3:
    # true values of x and y for mobile node
    true_x = sys.argv[1]
    true_y = sys.argv[2]


with open('coords.json', 'r') as f:
    data = json.load(f)

# # test
# true_x = 2
# true_y = 3

# rssi1 = 1
# rssi2 = 1
# rssi3 = 1
# rssi4 = 1

# data.append({
#     "true_coords": [true_x, true_y], 
#     "rssi_values": [ rssi1, rssi2, rssi3, rssi4 ]
# })

# print
# print(json.dumps(data))


# write
# with open('data.txt', 'w') as outfile:
#     json.dump(data, outfile)


serial_port = '/dev/ttyACM0'
print(sys.argv)
if len(sys.argv) == 2:
   serial_port = sys.argv[1]

ser = serial.Serial(serial_port)
ser.flushInput()

while True:

    ser_bytes = ser.readline()
    line = ser_bytes.decode("utf-8")
    print(line)
    try:
        j = json.loads(line)
        print(json.dumps(j))            
    except:
        continue
        
    # [[rssi1, distance1],[rssi2, distance2 ],[rssi3, _ ],[rssi4, _ ]
    # This is read line by line by our python script which then converts the rssi information into distance information using the following equation.

    rssi1 = j[0][0]
    rssi2 = j[1][0]
    rssi3 = j[2][0]
    rssi4 = j[3][0]


    new_data = {
        "true_coords": [true_x, true_y], 
        "rssi_values": [ rssi1, rssi2, rssi3, rssi4 ]
    }

    print(new_data)
    data.append(new_data)

    with open('coords.json', 'w') as f:
        json.dump(data, f)

    time.sleep(0.1)


