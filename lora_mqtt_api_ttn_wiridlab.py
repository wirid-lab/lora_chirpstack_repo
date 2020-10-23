#!/usr/bin/python

# This shows a simple example of an MQTT subscriber.


import paho.mqtt.client as mqtt
import base64, json
import os
import datetime
import requests

#Please Complete te next information for WiridLab Platform API connection
TOKEN_API_WIRIDLAB='<YOUR_AUTHENTICATION_WIRIDLAB_TOKEN>'
NODE_NAME_WIRIDLAB= '<NODE_NAME_WIRIDLAB_PLATFORM>'

#Please complete the next information for MQTT connection
SERVER_TTN='<DNS_TTN_SERVER>'
ACCESS_KEY_TTN= '<TTN_APPLICATION_ACCESS_KEY>'
APPLICATION_ID='<APPLICATION_ID_TTN>'


def on_connect(mqttc, obj, flags, rc):
    print("rc: " + str(rc))

def on_message(mqttc, obj, msg):
    #print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
    print('Mensaje --> Topic: {} \n'.format(msg.topic))
    x = msg.payload.decode("utf-8")
    y = json.loads(x)
    gateways = y["metadata"]["gateways"][-1]
    fields = y['payload_fields']
    acele = fields["accelerometer_3"]
    temp =  fields['temperature_2']
    time = gateways['time']
    lat = gateways['latitude']
    lon = gateways['longitude']
    alt = gateways['altitude']


    print("\nAcelerometro: \n\tX : {} \n\tY: {} \n\tZ: {} ".format(acele['x'],acele['y'],acele['z']))
    print("\nTime : {} \n Temp : {} \n\tlatitud:{} \n\tlongitud: {} \n\taltitud: {}".format(time,temp,lat,lon,alt))

    #z = base64.b64decode(z).decode("utf-8")
    #print ('datos: {}'.format(z))
    print("Enviando datos a la API.....")
    jsonData = [{}]
    nodeName="test-panel-01"
    jsonData[0]["temperature"] = temp
    jsonData[0]["dateSens"] = time
    jsonData[0]["position"] = {}
    jsonData[0]["position"]["lat"] = lat
    jsonData[0]["position"]["lng"] = lon
    jsonData[0]["acelerometer"] = acele
    jsonData[0]["infGate"] = gateways

    print (jsonData)
    jsonData = json.dumps(jsonData, indent=4)
    headers = {"WIRID-LAB-AUTH-TOKEN": TOKEN_API_WIRIDLAB, "Content-Type": "application/json"}
    info = requests.post("https://api.wiridlab.site/api/iot/devices/"+ NODE_NAME_WIRIDLAB.lower(), headers=headers, data=jsonData, timeout=None)
    data = info.json()

    if (info.status_code == 200):
        print ("  Request API")
        print(json.dumps(data, indent=4, sort_keys=True))
    else:
        print ("error enviando comunicacion")
        print(json.dumps(data, indent=4, sort_keys=True))


def on_subscribe(mqttc, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))


def on_log(mqttc, obj, level, string):
    print(string)


# If you want to use a specific client id, use
# mqttc = mqtt.Client("client-id")
# but note that the client id must be unique on the broker. Leaving the client
# id parameter empty will generate a random id for you.
mqttc = mqtt.Client()
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
# Uncomment to enable debug messages
# mqttc.on_log = on_log
mqttc.username_pw_set(APPLICATION_ID, ACCESS_KEY_TTN)
mqttc.connect(SERVER_TTN, 1883, 30)
mqttc.subscribe("+/devices/+/up", 0)

mqttc.loop_forever()
