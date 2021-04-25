To Flash the static node code for the Ultrasonic sensor:
FOR NODE 1:
-> in p3/static/src/main.c : For node 1, in the manuf_data array, after 0x63, 0x73, 0x00 change the next data byte to '0x01' if it is not already that.
-> in myoslib/src/os_hci.c : In the sensor_data array, do the same as above. The next byte will hold the distance information.

FOR NODE 2:
-> in p3/static/src/main.c : For node 2, in the manuf_data array, after 0x63, 0x73, 0x00 change the next data byte to '0x02' if it is not already that.
-> in myoslib/src/os_hci.c : In the sensor_data array, do the same as above. The next byte will hold the distance information.

The above changes are needed only for the nodes connected to the ultrasonic ranger.
Later the code can be changed to accomodate a better way of handling the static nodes connected to the sensor.
