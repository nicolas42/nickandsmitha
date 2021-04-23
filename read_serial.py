import serial
ser = serial.Serial('/dev/ttyACM0')
ser.flushInput()

while True:
    try:
        ser_bytes = ser.readline()
        line = ser_bytes.decode("utf-8")
        print(line)
    except:
        print("Keyboard Interrupt")
        break


