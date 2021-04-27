Kalman_calc in the read_serial.py takes the values from the two sensors, RSSI and the ultrasonic sensor and fuses them.
The resulting output is distance because the two sensors have distance values as their outputs. 

This is as per the spec requirement. Essentially the Kalman would take a velocity parameter and track accordingly. 
Here we just fuse the data and show the best distance to consider for the mobile and fix its coordinates accordingly.

