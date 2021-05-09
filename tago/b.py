import tago

MY_DEVICE_TOKEN = 'b2fcacd2-ff41-4158-bdbb-e5ec94f8cbf2'
my_device = tago.Device(MY_DEVICE_TOKEN)


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
        

        print(variables, values)

    else:
        print(findData['message']) # Error (if status is False)



receive_data()