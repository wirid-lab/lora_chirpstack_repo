import paho.mqtt.client as mqtt
import json
import base64
import requests

#Please Complete te next information for API connection
TOKEN_API_WIRIDLAB='<YOUR_AUTHENTICATION_WIRIDLAB_TOKEN>'
NODE_NAME_WIRIDLAB= '<NODE_NAME_WIRIDLAB_PLATFORM>'

#Please complete the next information for MQTT connection
SERVER_CHIRPSTACK='<IP_CHIRPSTACK_SERVER>'
MQTT_CHIRPSTACK_PORT= '<MQTT_CHIRPSTACK_CONNECTION_PORT>'
APPLICATION_ID='<APPLICATION_ID_CHIRPSTACK>'

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("application/"+APPLICATION_ID+"/#")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    x=str(msg.payload)
    y = json.loads(x)
    gateways = y['rxInfo']
    node = y['txInfo']
    appid = y['applicationID']
    appname = y['applicationName']
    devname = y['deviceName']
    data = y['data']
    d = base64.b64decode(data)
    # Decoding the bytes to string
    datad = d.decode("UTF-8")

    print("Enviando datos a la API.....")
    jsonData = [{}]
    jsonData[0]["appID"] = appid
    jsonData[0]["appName"] = appname
    jsonData[0]["devName"] = devname
    jsonData[0]["dataNode"] = datad
    jsonData[0]["infNode"] = node
    jsonData[0]["infGate"] = gateways
    print (jsonData)

    jsonData = json.dumps(jsonData, indent=4)
    headers = {"WIRID-LAB-AUTH-TOKEN": TOKEN_API_WIRIDLAB, "Content-Type": "application/json"}
    info = requests.post("https://api.wiridlab.site/api/iot/devices/"+ NODE_NAME_WIRIDLAB.lower() , headers=headers, data=jsonData, timeout=None)
    dataAPI = info.json()

    if (info.status_code == 200):
        print ("  Request API")
        print(json.dumps(dataAPI, indent=4, sort_keys=True))
    else:
        print ("Error sending information")
        print(json.dumps(dataAPI, indent=4, sort_keys=True))

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(SERVER_CHIRPSTACK, MQTT_CHIRPSTACK_PORT, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_forever()
