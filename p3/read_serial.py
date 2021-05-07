import serial
import json
import time
import numpy as np
from numpy.linalg import inv


nsamples = 50
show_gui = False 

def do_least_squares_approximation(r1,r2,r3,r4):


    # # Approximate the position of the mobile device
    # # using four static devices which are each 4 meters apart
    # # Each device broadcasts a received signal strength indicator
    # # and two of the devices will also broadcast distance values 
    # # derived from an ultrasound sensor.

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

def average(lst):
    return sum(lst) / len(lst)


def covariance2d(sigma1, sigma2):
    cov1_2 = sigma1 * sigma2
    cov2_1 = sigma2 * sigma1
    cov_matrix = np.array([[sigma1 ** 2, cov1_2], [cov2_1, sigma2 ** 2]])
    return np.diag(np.diag(cov_matrix))

def prediction(x, p, q, f):
    X_prime = f.dot(x) 
    P_prime = f.dot(p).dot(f.T) + q
    return X_prime, P_prime

def update(X, P, y, R, H):
    Inn = y - H.dot(X)
    S =  H.dot(P).dot(H.T) + R
    K = P.dot(H.T).dot(inv(S))
    X = X + K.dot(Inn)
    P = P - K.dot(H).dot(P)
    return X, P

def kalman_calc(s1,s2,r1,r2,r3,r4):
    t = 0.01  # Difference in time

    F = np.array([[1, t], 
                  [0, 1]])

    # observation matrix 
    H = np.array([[1, 0]]);
    
    # system noise
    Q = np.array([[0.05, 0],
                  [0, 1]])

    # Process / Estimation Errors
    error_est_x = 10
    error_est_xdot = 1 
    
    # Observation Errors
    error_obs_x = 10  # Uncertainty in the measurement
    error_obs_xdot = 1

    # initial state and covariance matrix
    X = np.array([[s1],[0]])
    P = covariance2d(error_est_xdot, error_est_x) 

    X, P = prediction(X, P, Q, F)
    X, P = update(X, P, s1, error_obs_xdot, H)
    X, P = update(X, P, s2, error_obs_xdot, H)
    X, P = update(X, P, r1, error_obs_x, H)
    X, P = update(X, P, r2, error_obs_x, H)
    X, P = update(X, P, r3, error_obs_x, H)
    X, P = update(X, P, r4, error_obs_x, H)
    return X
                 

import sys
serial_port = '/dev/ttyACM0'
print(sys.argv)
if len(sys.argv) == 2:
    serial_port = sys.argv[1]
 
ser = serial.Serial(serial_port)
ser.flushInput()

x0s = []
y0s = []
r1s = []
r2s = []
r3s = []
r4s = []
r1 = 0
st_matrix = np.array([[0],[0]])

if show_gui:
    import matplotlib.pyplot as plt
    import numpy as np

    plt.ion()
    fig, ax = plt.subplots(2)
    sc = ax[0].scatter(x0s,y0s)
    sc1 = ax[1].scatter(st_matrix[0][0], r1, c='red')
    ax[0].set_xlim(0,10)
    ax[0].set_ylim(0,10)
    ax[0].set_title("Multilateration coordinates")
    ax[1].set_xlim(0,200)
    ax[1].set_ylim(0,200)
    ax[1].set_title("Kalman output distance")
#    plt.xlim(0,200)
#    plt.ylim(0,200)
    plt.draw()


while True:
    # ser.flushInput()

    try:
        ser_bytes = ser.readline()
        line = ser_bytes.decode("utf-8")
        print(line)
        try:
            j = json.loads(line)
          #  print(json.dumps(j))            
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
            # As of writing this we assuming N=2 and ref_rssi=-65

            N=2
            ref_rssi=-61
            r1 = 10**( (ref_rssi - rssi1) / (10*N) )
            r2 = 10**( (ref_rssi - rssi2) / (10*N) )
            r3 = 10**( (ref_rssi - rssi3) / (10*N) )
            r4 = 10**( (ref_rssi - rssi4) / (10*N) )

            # Ultrasound distance sensor measurements take priority within 1m range
            distance1 = j[0][1]
            distance2 = j[1][1]

            d_rssi = ([r1, r2, r3, r4])
            d_sensor = ([distance1, distance2, 0, 0])

            # fusion
            if (0 < distance1) and (distance1 < 50):
                r1 = float(distance1) / 100
            if (0 < distance2) and (distance2 < 50):
                r2 = float(distance2) / 100


            r1s.append(r1)
            r2s.append(r2)
            r3s.append(r3)
            r4s.append(r4)

           # print(r1,r2,r3,r4)
            print(round(average(r1s[-nsamples:]),2), round(average(r2s[-nsamples:]),2), round(average(r3s[-nsamples:]),2), round(average(r4s[-nsamples:]),2))

            # location = do_least_squares_approximation(r1,r2,r3,r4)
            location = do_least_squares_approximation(average(r1s[-nsamples:]),average(r2s[-nsamples:]),average(r3s[-nsamples:]),average(r4s[-nsamples:]))

            x0 = location[0][0]
            y0 = location[1][0]

            x0s.append(x0)
            y0s.append(y0)

            # x0 = clamp( x0, 0, 4)
            # y0 = clamp( y0, 0, 4)

           # print(x0,y0)
            st_matrix = kalman_calc(d_sensor[0],d_sensor[1],d_rssi[0], d_rssi[1], d_rssi[2], d_rssi[3])
            print (st_matrix)

            if show_gui:
                ax[0].set_title("Multilateration coordinates: " + str(round(x0,2)) + " " + str(round(y0,2)) )
                ax[1].set_title("Kalman output distance: " + str(round(st_matrix[0][0],2)))

                sc.set_offsets(np.c_[x0s,y0s])
                sc1.set_offsets(np.c_[st_matrix[0][0],r1])
                fig.canvas.draw_idle()
                plt.pause(0.01)


            # if show_gui:
            #     plt.scatter(x0, y0)
            #     plt.pause(0.05)
            #     # Call plt.pause(0.05) to both draw the new data and it runs the GUI's event loop (allowing for mouse interaction).
            #     # plt.show()



            # then we can use the distance values from the US sensors in preference to the rssi values if the US distance values
            # are below 100 (cm).


            # "garbage collection"
            x0s = x0s[-nsamples:]
            y0s = y0s[-nsamples:]
            r1s = r1s[-nsamples:]
            r2s = r2s[-nsamples:]
            r3s = r3s[-nsamples:]
            r4s = r4s[-nsamples:]
        except:
            pass

    except:
        print("Keyboard Interrupt")
        break
    # time.sleep(0.01)


