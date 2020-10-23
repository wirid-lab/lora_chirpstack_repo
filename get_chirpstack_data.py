import requests
import json

JWT_TOKEN_CHIRPSTACK='<YOUR_API_TOKEN>'
NODE_IP= '<SERVER_API_IP>'
REQUEST_PORT='<API_PORT>'

headers = {
    'Accept': 'application/json',
    'Grpc-Metadata-Authorization': 'Bearer ' + JWT_TOKEN_CHIRPSTACK,
}

response = requests.get('http://'+NODE_IP+':'REQUEST_PORT'/api/gateways/b827ebfffe30d253', headers=headers)
print (response.json())

