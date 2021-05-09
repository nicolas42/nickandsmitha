import serial
import json


import multiprocessing as mp


serial_port = '/dev/ttyACM0'
ser = serial.Serial(serial_port)
ser.flushInput()




def read_serial_into_queue(q):
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





if __name__ == '__main__':
    mp.set_start_method('spawn')
    q = mp.Queue()
    p = mp.Process(target=read_serial_into_queue, args=(q,))
    p.start()
    while True:
        print(q.get())
    p.join()