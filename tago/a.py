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
  print(data_to_insert)
  result = my_device.insert(data_to_insert)  # With response
  if result['status']:
    print(result['result'])
  else:
    print(result['message'])



MY_DEVICE_TOKEN = 'b2fcacd2-ff41-4158-bdbb-e5ec94f8cbf2'
my_device = tago.Device(MY_DEVICE_TOKEN)

# data_to_insert = {
#   'variable': 'temperature',
#   'location': {'lat': 42.2974279, 'lng': -85.628292},
#   'time': '2014-01-20 03:43:59',
#   'unit': 'C',
#   'value': 63
# }

rssi1 = 100*random.random()
rssi2 = 100*random.random()
rssi3 = 100*random.random()
rssi4 = 100*random.random()
rssi5 = 100*random.random()
rssi6 = 100*random.random()
rssi7 = 100*random.random()
rssi8 = 100*random.random()

us1 = 100*random.random()
us2 = 100*random.random()
us3 = 100*random.random()
us4 = 100*random.random()

send_data(rssi1, rssi2, rssi3, rssi4, rssi5, rssi6, rssi7, rssi8, us1, us2, us3, us4)


