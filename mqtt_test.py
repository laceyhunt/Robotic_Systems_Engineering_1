import sys
import time
from paho.mqtt import client as mqtt_client
import random
import json
from datetime import datetime

# michael_ip = '10.8.4.13' # broker
broker = '10.8.4.13'
lacey_ip = '10.8.4.13'
port = 1883

bill_topic = 'robot/bill'
dj_topic = 'robot/dj'
# test_topic = 'robot/test'
message_recieved = False

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client()
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def publish(client, msg):
   result = client.publish(bill_topic, msg)
   # result: [0, 1]
   status = result[0]
   if status == 0:
      print(f"Send `{msg}` to topic `{bill_topic}`")
   else:
      print(f"Failed to send message to topic {bill_topic}")
      

def subscribe(client: mqtt_client):
   def on_message(client, userdata, msg):
      global message_recieved
      print(f"Lacey Received `{msg.payload.decode()}` from `{msg.topic}` topic")
      message_recieved = True
      return decode_json(msg.payload.decode())

   client.subscribe(dj_topic)
   client.on_message = on_message


def find_random_position():
   global handoff_middle_pos
   new_pos=handoff_middle_pos
   x=handoff_middle_pos[0]
   y=handoff_middle_pos[1]
   z=handoff_middle_pos[2]
   new_pos[0]=random.randint(x-150,x+150)
   new_pos[1]=random.randint(y-250,y+250)
   new_pos[2]=random.randint(z-125,z+125)
   return new_pos

def package_json(coordinate, gripper_status, loop_finished=False):
    """Package the JSON string to send over MQTT to other robot

    Args:
        coordinate (List): current position to send to other robot
        gripper_status (Bool): 0=closed, 1=open
        loop_done (bool, optional): _description_. Defaults to False.
    """
    data = {
        "gripper": gripper_status,
        "x_coordinate": coordinate[0],
        "y_coordinate": coordinate[1],
        "z_coordinate": coordinate[2],
        "loop_finished": loop_finished
        }
    encoded_msg = json.dumps(data, indent=4)
    return encoded_msg
 
 
def decode_json(message):
   data = json.loads(message)
   print(data)
   print(f"**DECODED**Gripper status: {data['gripper']}, X: {data['x_coordinate']}, Y: {data['y_coordinate']}, Z: {data['z_coordinate']}")
   return data

def main():
   global message_recieved
   
   client = connect_mqtt()
   client.loop_start()

   test_pos=find_random_position()
   msg=package_json(test_pos,True)
   # print('Encoded message: ')
   # print(msg)
   # decode_json(msg)

   publish(client, f'{msg}')

   message_recieved = False
   msg=subscribe(client)
   while not message_recieved:  # wait for gripper to close, eventually actually look at message
      time.sleep(0.2)
   
   client.loop_stop()
   client.disconnect()
    



start_dt = datetime.now()

main()

end_dt = datetime.now()
execution_duration = end_dt - start_dt
print(f"Program duration: {execution_duration}")