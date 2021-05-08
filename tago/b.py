import tago

MY_DEVICE_TOKEN = 'b2fcacd2-ff41-4158-bdbb-e5ec94f8cbf2'
my_device = tago.Device(MY_DEVICE_TOKEN)

findData = my_device.find({'query': 'last_value'})
if findData['status'] is True:
    # print(findData['result']) # Array with data
    a = findData['result']
    a.sort(key=lambda x: x['variable'])
    # print(a)

    values = []
    for item in a:
        values.append({item['variable'], item['value']})
    
    print(values)

else:
    print(findData['message']) # Error (if status is False)



