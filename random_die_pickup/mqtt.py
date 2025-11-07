import sys
import time
from paho.mqtt import client as mqtt_client
import random
import json
from datetime import datetime

"""  
   USAGE:
   
   
   # Initialize client
   client = connect_mqtt()
   client.loop_start()

   # Publish position
   msg=package_json(rand_pos, True)
   publish(client, msg)
   
   # Wait for DJ to go to and send new handoff coordinates
   message_recieved = False
   subscribe(client)
   while not message_recieved:
      time.sleep(0.2)

   # Disconnect MQTT
   client.loop_stop()
   client.disconnect()
"""

# Michael's IP
broker = '10.8.4.17'
port = 1883

cluster_topic = 'topics/cluster'
coord_topic = 'topics/coords'

# global variable that stores the latest decoded mqtt packet
message_i_got=None

robot_ip = '10.8.4.6' # Bill (OnRobot)
# robot_ip = '10.8.4.16' # DJ (Schunk)

def connect_mqtt():
   """Connects to the mqtt broker
   """
   def on_connect(client, userdata, flags, rc):
      if rc == 0:
         print("Connected to MQTT Broker!")
      else:
         print("Failed to connect, return code %d\n", rc)

   client = mqtt_client.Client()
   client.on_connect = on_connect
   client.connect(broker, port)
   return client

def publish(client, msg, topic):
   """Publishes a message to the 'robot/bill' topic

   Args:
      client (mqtt client): the mqtt object loop that has been started
      msg (json object): message to publish
   """
   result = client.publish(topic, msg)
   # result: [0, 1]
   status = result[0]
   if status == 0:
      print(f"Send `{msg}` to topic `{topic}`")
   else:
      print(f"Failed to send message to topic {topic}")
      

def subscribe(client: mqtt_client, topic):
   """subscribes to the 'robot/dj' topic and defines on_message behavior

   Args:
      client (mqtt client): the mqtt object loop that has been started
   """
   retmsg = None
   def on_message(client, userdata, msg):
      """behavior for when a message is received
         assigns the decoded message contents to the global 'message_i_got' variable, sets 'message_received' flag to break out of loop in main

      Args:
         client (mqtt client): the mqtt object loop that has been started
         userdata (unknown): handled by the paho mqtt library
         msg (json object): packet received 
      """
      global message_recieved, message_i_got
      print(f"Lacey Received `{msg.payload.decode()}` from `{msg.topic}` topic")
      message_recieved = True
      message_i_got=decode_json(msg.payload.decode())

   client.subscribe(topic)
   client.on_message = on_message


def package_json(coordinate_list=None,cluster_flag=None):
   """Package the JSON string to send over MQTT to other robot

   Args:
      coordinate (List): current position to send to other robot
      gripper_status (Bool): 0=closed, 1=open
      loop_done (bool, optional): _description_. Defaults to False.
   """
   if(not cluster_flag): 
      data = {
         "coords": coordinate_list,
         }
   elif(cluster_flag):
      data = {
         "cluster_flag": cluster_flag,  # Options: "dj", "theo", "none", "theo_done"
         "coordinate": coordinate_list[0]
         }
   encoded_msg = json.dumps(data, indent=4)
   return encoded_msg


def decode_json(message):
   """decodes json object into a dictionary and returns

   Args:
      message (json object): the encoded json object in the format:

   Returns:
      dict: decoded json object
   """
   data = json.loads(message)
   return data
