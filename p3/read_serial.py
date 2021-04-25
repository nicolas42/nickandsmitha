import serial
import json

ser = serial.Serial('/dev/ttyACM0')
ser.flushInput()

while True:
    try:
        ser_bytes = ser.readline()
        line = ser_bytes.decode("utf-8")
        # print(line)
        try:
            j = json.loads(line)
            # print(json.dumps(j, sort_keys=True, indent=4))
            print(json.dumps(j))
        except:
            pass
            # print("bad json", line, end="")

    except:
        print("Keyboard Interrupt")
        break
    


# # Approximate the position of the mobile device
# # using four static devices which are each 4 meters apart
# # Each device broadcasts a received signal strength indicator
# # and two of the devices will also broadcast distance values 
# # derived from an ultrasound sensor.

# import numpy as np
# # k = 4

# r1 = 2.0
# r2 = 2.0
# r3 = 2.0
# r4 = 2.0

# x1,y1 = 0.0,0.0
# x2,y2 = 4.0,0.0
# x3,y3 = 4.0,4.0
# x4,y4 = 0.0,4.0

# b = np.array([
# [ r1**2 -r4**2 -x1**2 -y1**2 +x4**2 +y4**2 ], 
# [ r2**2 -r4**2 -x2**2 -y2**2 +x4**2 +y4**2 ], 
# [ r3**2 -r4**2 -x3**2 -y3**2 +x4**2 +y4**2 ]
# ])

# A = np.array([
# [ 2*(x4-x1), 2*(y4-y1)],
# [ 2*(x4-x2), 2*(y4-y2)],
# [ 2*(x4-x3), 2*(y4-y3)]
# ])

# # Ax = b
# # estimate for x is 

# A1 = np.linalg.inv(np.matmul(A.transpose(), A))
# A2 = np.matmul(A1, A.transpose())
# A3 = np.matmul(A2, b)
# print(A3)
