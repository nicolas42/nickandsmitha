import serial
import json
import time
import numpy as np
from numpy.linalg import inv
from sklearn import neighbors


nsamples = 50
show_gui = False



rssi_reference_values = [
    # coordinate (x,y) [rssi values], class (box 1 or box 2)
    [ [0,0], [-56.8,-59.8,-52.18,-70.1,-91.0,-62.0,-68.68,-77.74 ], 1],
    [ [4,0], [-68.08,-55.54,-57.4,-65.02,-87.46,-62.0,-67.64,-83.02 ],1 ],
    [ [4,3.5], [-72.04,-66.7,-48.02,-70.78,-84.86,-62.0,-64.62,-79.78 ],1 ],
    [ [4,4.5], [-80.2,-68.14,-53.44,-63.98,-87.84,-62.0,-62.42,-81.5 ],2 ],
    [ [4,8], [-64.62,-71.58,-60.46,-63.98,-87.26,-62.0,-56.44,-78.04 ],2 ],
    [ [0,8], [-70.26,-61.56,-57.94,-63.48,-87.2,-62.0,-62.72,-70.18 ],2 ],
    [ [0,4.5], [-55.06,-64.52,-58.56,-49.9,-74.06,-62.0,-69.1,-73.38 ],2 ],
    [ [0,3.5], [-64.38,-60.64,-54.56,-39.82,-80.2,-62.0,-58.72,-73.74 ],1 ],
    [ [4,1.75], [-69.44,-56.02,-56.88,-67.38,-85.24,-62.0,-60.58,-78.5],1 ],
    [ [2,0], [-66.64,-69.42,-59.24,-66.26,-82.2,-62.0,-69.76,-74.88 ],1 ],
    [ [0,1.75], [-62.22,-68.84,-55.4,-60.48,-86.08,-62.0,-65.52,-68.44 ],1 ],
    [ [2,1.75], [-60.84,-50.78,-53.92,-56.52,-83.74,-62.0,-64.82,-81.64 ],1 ],
    [ [2,3.5], [-51.34,-69.7,-55.02,-61.24,-83.16,-62.0,-68.14,-70.82 ],1 ],
    [ [4,6.25], [-67.78,-65.6,-63.66,-61.42,-88.56,-62.0,-65.48,-76.56 ],2 ],
    [ [2,4.5], [-67.14,-55.16,-55.54,-62.06,-77.06,-62.0,-67.62,-76.62 ],2 ],
    [ [0,6.25], [-59.04,-63.8,-60.0,-48.88,-79.26,-62.0,-76.8,-66.74 ],2 ],
    [ [2,8], [-65.4,-64.42,-60.72,-55.58,-84.5,-62.0,-57.36,-76.18 ],2 ],
    [ [2,6.25], [-63.78,-59.12,-47.4,-61.9,-83.5,-62.0,-72.34,-76.66 ],2 ]
]


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

X= list()
y = list()

for i in range(len(rssi_reference_values)):
    X.append(rssi_reference_values[i][1])
    y.append(rssi_reference_values[i][0])
print(X,y)

knn = neighbors.KNeighborsRegressor(n_neighbors=3)
knn.fit(X, y)
                 

import tago
import random
from datetime import datetime

def get_date_time_string():
  # datetime object containing current date and time
  now = datetime.now()
  dt_string = now.strftime("%Y-%m-%d %H:%M:%S")
  return dt_string



def send_data(rssi1, rssi2, rssi3, rssi4, rssi5, rssi6, rssi7, rssi8, us1, us2, us3, us4):

  now_string = get_date_time_string()

  data_to_insert =  [

    {
      "time": now_string,
      "variable": "rssi1",
      "value": rssi1
    },
    {
      "time": now_string,
      "variable": "rssi2",
      "value": rssi2
    },
    {
      "time": now_string,
      "variable": "rssi3",
      "value": rssi3
    },
    {
      "time": now_string,
      "variable": "rssi4",
      "value": rssi4
    },
    {
      "time": now_string,
      "variable": "rssi5",
      "value": rssi5
    },
    {
      "time": now_string,
      "variable": "rssi6",
      "value": rssi6
    },
    {
      "time": now_string,
      "variable": "rssi7",
      "value": rssi7
    },
    {
      "time": now_string,
      "variable": "rssi8",
      "value": rssi8
    },
    {
      "time": now_string,
      "variable": "us1",
      "value": us1
    },
    {
      "time": now_string,
      "variable": "us2",
      "value": us2
    },
    {
      "time": now_string,
      "variable": "us3",
      "value": us4
    },
    {
      "time": now_string,
      "variable": "us4",
      "value": us4
    }
  ]

  # my_device.insert(data_to_insert)  # Without response
#   print(data_to_insert)
  result = my_device.insert(data_to_insert)  # With response
  if result['status']:
    # print(result['result'])
    return result['result']
  else:
    # print(result['message'])
    return result['message']



def receive_data():
    findData = my_device.find({'query': 'last_value'})
    if findData['status'] is True:
        # print(findData['result']) # Array with data
        a = findData['result']
        a.sort(key=lambda x: x['variable'])
        # print(a)

        variables = []
        values = []
        for item in a:
            variables.append(item['variable'])
            values.append(item['value'])
        

        # print(variables, values)
        return variables, values

    else:
        print(findData['message']) # Error (if status is False)






# =====================================================
# ==================== MAIN ===========================
# =====================================================


# initialize tago
MY_DEVICE_TOKEN = 'b2fcacd2-ff41-4158-bdbb-e5ec94f8cbf2'
my_device = tago.Device(MY_DEVICE_TOKEN)



# initialize knn stuff
r1 = 0
st_matrix = np.array([[0],[0]])

knn_mob1 = np.array([[0,0]])
knn_mob2 = np.array([[0,0]])

# initialize gui stuff
if show_gui:
    import matplotlib.pyplot as plt
    import numpy as np

    plt.ion()
    fig, ax = plt.subplots(2)
    sc = ax[0].scatter(knn_mob1[0][0], knn_mob1[0][1] , c='green')
    sc1 = ax[1].scatter(knn_mob2[0][0], knn_mob2[0][1] , c='red')
    ax[0].set_xlim(0,10)
    ax[0].set_ylim(0,10)
    ax[0].set_title("Knn location for mobile 1")
    ax[1].set_xlim(0,10)
    ax[1].set_ylim(0,10)
    ax[1].set_title("Knn location for mobile 2")
    plt.draw()




# initialize serial stuff
import multiprocessing

serial_port = '/dev/ttyACM0'
ser = serial.Serial(serial_port)
ser.flushInput()


def read_serial_process(q):
    while True:

        ser_bytes = ser.readline()
        line = ser_bytes.decode("utf-8")
        # print(line, end="")
        try:
            j = json.loads(line)
            # print(json.dumps(j))
            q.put(j)         
        except:
            ser.flushInput()
            continue



# initialize serial input value
j = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],1]

if __name__ == '__main__':

    multiprocessing.set_start_method('spawn')
    serial_queue = multiprocessing.Queue()
    serial_process = multiprocessing.Process(target=read_serial_process, args=(serial_queue,))
    serial_process.start()


    while True:

        serial_inputs = []
        while not serial_queue.empty():
            j = serial_queue.get() # remove and return an item from the queue.
            serial_inputs.append(j)

        for item in serial_inputs:
            print(item)

        
                

        # Get info from json
        # format is [[rssi1, us1],[rssi2, us2 ],[rssi3, _ ],[rssi4, _ ],[rssi5, us3],[rssi6, us4 ],[rssi7, _ ],[rssi8, _ ], id ]
        rssi1 = j[0][0]
        rssi2 = j[1][0]
        rssi3 = j[2][0]
        rssi4 = j[3][0]
        rssi5 = j[4][0]
        rssi6 = j[5][0]
        rssi7 = j[6][0]
        rssi8 = j[7][0]

        # ultrasound sensor distances from nodes 1,2,5,6
        us1 = j[0][1] 
        us2 = j[1][1]
        us3 = j[3][1]
        us4 = j[4][1]

        # the mobile device it's coming from
        mobile_id = j[8] # 1 or 2








        ## *************** for testing only *********************
        # mobile_id = 1 # 1 or 2


        # rssi1 = -100*random.random()
        # rssi2 = -100*random.random()
        # rssi3 = -100*random.random()
        # rssi4 = -100*random.random()
        # rssi5 = -100*random.random()
        # rssi6 = -100*random.random()
        # rssi7 = -100*random.random()
        # rssi8 = -100*random.random()

        # us1 = 0
        # us2 = 0
        # us3 = 0
        # us4 = 0


        # dist = 10**((ref - rssi) /10*N))
        # As of writing this we assuming N=2 and ref_rssi=-65
        N=2
        ref_rssi=-61
        r1 = 10**( (ref_rssi - rssi1) / (10*N) )
        r2 = 10**( (ref_rssi - rssi2) / (10*N) )
        r3 = 10**( (ref_rssi - rssi3) / (10*N) )
        r4 = 10**( (ref_rssi - rssi4) / (10*N) )
        r5 = 10**( (ref_rssi - rssi5) / (10*N) )
        r6 = 10**( (ref_rssi - rssi6) / (10*N) )
        r7 = 10**( (ref_rssi - rssi7) / (10*N) )
        r8 = 10**( (ref_rssi - rssi8) / (10*N) )

        # location = do_least_squares_approximation(r1,r2,r3,r4)
        # x0 = location[0][0]
        # y0 = location[1][0]


        # d_rssi = ([r1, r2, r3, r4])
        # d_sensor = ([us1, us2, 0, 0])
        # st_matrix = kalman_calc(d_sensor[0],d_sensor[1],d_rssi[0], d_rssi[1], d_rssi[2], d_rssi[3])
        # print (st_matrix)

        #knn predict
        if mobile_id == 1:
        ## *************** for testing only *********************
        ## *************** replace with j array values ***********************
            knn_mob1 = knn.predict([[rssi1, rssi2, rssi3, rssi4, rssi5, rssi6, rssi7, rssi8]]) #rssi values from s
        else:    
        ## *************** for testing only *********************
        ## *************** replace with j array values ***********************
            knn_mob2 = knn.predict([[1,2,3,4,4,5,6,7]]) #rssi values from s

        print(knn_mob1[0][0], knn_mob1[0][1])
        if show_gui:
              sc.set_offsets(np.c_[knn_mob1[0][0],knn_mob1[0][1]])
              sc1.set_offsets(np.c_[knn_mob2[0][0],knn_mob2[0][1]])

              fig.canvas.draw_idle()
              plt.pause(0.01)





        # print('sending data')
        # send_data(rssi1, rssi2, rssi3, rssi4, rssi5, rssi6, rssi7, rssi8, us1, us2, us3, us4)

        # print('receiving data')
        # variables, values = receive_data()
        # print('data received: ')
        # print(variables, values)



        time.sleep(1)


    serial_process.join()
