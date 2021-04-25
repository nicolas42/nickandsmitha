import serial
import json

import matplotlib.pyplot as plt
plt.axis([0, 4, 0, 4])

show_gui = False

def do_least_squares_thing_please(r1,r2,r3,r4):


    # # Approximate the position of the mobile device
    # # using four static devices which are each 4 meters apart
    # # Each device broadcasts a received signal strength indicator
    # # and two of the devices will also broadcast distance values 
    # # derived from an ultrasound sensor.

    import numpy as np
    # nodes 1..k where k=4

    # for testing
    # r1 = 2.0
    # r2 = 2.0
    # r3 = 2.0
    # r4 = 2.0

    x1,y1 = 0.0,0.0
    x2,y2 = 4.0,0.0
    x3,y3 = 4.0,4.0
    x4,y4 = 0.0,4.0


    b = np.array([
    [ r1**2 -r4**2 -x1**2 -y1**2 +x4**2 +y4**2 ], 
    [ r2**2 -r4**2 -x2**2 -y2**2 +x4**2 +y4**2 ], 
    [ r3**2 -r4**2 -x3**2 -y3**2 +x4**2 +y4**2 ]
    ])

    A = np.array([
    [ 2*(x4-x1), 2*(y4-y1)],
    [ 2*(x4-x2), 2*(y4-y2)],
    [ 2*(x4-x3), 2*(y4-y3)]
    ])

    # Ax = b
    # estimate for x is 

    A1 = np.linalg.inv(np.matmul(A.transpose(), A))
    A2 = np.matmul(A1, A.transpose())
    A3 = np.matmul(A2, b)

    return A3
    # print(A3)



def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


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
            # print("bad json", line, end="")
            pass



        try:

            # [[rssi1, distance1],[rssi2, distance2 ],[rssi3, _ ],[rssi4, _ ]
            # This is read line by line by our python script which then converts the rssi information into distance information using the following equation.

            rssi1 = j[0][0]
            rssi2 = j[1][0]
            rssi3 = j[2][0]
            rssi4 = j[3][0]

            # dist = 10**((ref - rssi) /10*N))
            # As of writing this we assuming N=2 and ref=-65

            N=2
            ref=-65
            r1 = 10**( (ref - rssi1) / (10*N) )
            r2 = 10**( (ref - rssi2) / (10*N) )
            r3 = 10**( (ref - rssi3) / (10*N) )
            r4 = 10**( (ref - rssi4) / (10*N) )


            # # US Distance sensor measurements take priority within 1m range
            # distance1 = j[0][1]
            # distance2 = j[1][1]
            # if distance1 < 100:
            #     r1 = float(distance1) / 100
            # if distance2 < 100:
            #     r2 = float(distance2) / 100





            location = do_least_squares_thing_please(r1,r2,r3,r4)
            # print(location)
            # location should be within (0..4,0..4) so we can bound it at the edges
            # example format
            # [[ 2.00524551]
            # [-5.87647575]]

            x0 = location[0][0]
            y0 = location[1][0]

            x0 = clamp( x0, 0, 4)
            y0 = clamp( y0, 0, 4)

            print(x0,y0)



            if show_gui:
                plt.scatter(x0, y0)
                plt.pause(0.05)
                # Call plt.pause(0.05) to both draw the new data and it runs the GUI's event loop (allowing for mouse interaction).
                # plt.show()



            # then we can use the distance values from the US sensors in preference to the rssi values if the US distance values
            # are below 100 (cm).

        except:
            pass

    except:
        print("Keyboard Interrupt")
        break


