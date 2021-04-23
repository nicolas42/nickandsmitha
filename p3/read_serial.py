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
            print("that's some bad json")

    except:
        print("Keyboard Interrupt")
        break
    


