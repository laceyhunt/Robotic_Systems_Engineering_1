import sys
import time
from paho.mqtt import client as mqtt_client
import random
import json
driver_dir = './FANUC-Ethernet_IP_Drivers/src/'
sys.path.append(driver_dir)
from robot_controller import robot
from datetime import datetime

broker = '10.8.4.13'
port = 1883

bill_topic = 'robot/bill'
dj_topic = 'robot/dj'
# test_topic = 'robot/test'
message_i_got=None

robot_ip = '10.8.4.6' # Bill (OnRobot)
# robot_ip = '10.8.4.16' # DJ (Schunk)
# client_id = 12

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
    retmsg = None
    def on_message(client, userdata, msg):
        global message_recieved, message_i_got
        print(f"Lacey Received `{msg.payload.decode()}` from `{msg.topic}` topic")
        message_recieved = True
        message_i_got=decode_json(msg.payload.decode())

    client.subscribe(dj_topic)
    client.on_message = on_message
    
# OFFSETS TO GET TO DJ POSITION
y_offset=-1750
z_offset=8
safe_offset = 300
message_recieved = False
# Gripper specs (for OnRobot gripper)
open_width = 100
close_width = 80
force = 40
yaw, pitch, roll = 145,-90,-55
# JOINT positions
home_pos_joint = [0,0,0,0,-90,0]
# CARTESIAN positions
home_pos = [540, -150, 550, -179.9, 0, 0]
on_dice_start = [640, 4.3, 49.328, 178.758, 0, .94]
above_dice_start = [640, 4.3, 150, 178.758, 0, .94]
handoff_middle_pos = [250,-900,100,145,-90,-55]   # could be 88 instead of 90
safe_pose = [250,-600,100,145,-90,-55] 

def find_random_position():
   global handoff_middle_pos
   new_pos=handoff_middle_pos.copy()
   x=handoff_middle_pos[0]
   y=handoff_middle_pos[1]
   z=handoff_middle_pos[2]
   new_pos[0]=random.randint(x-150,x+150)
   new_pos[1]=random.randint(y-150,y+150)
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

def get_new_pos():
    global message_i_got
    new_pos = [message_i_got['x_coordinate'], message_i_got['y_coordinate'], message_i_got['z_coordinate'], yaw, pitch, roll]
    new_pos[1]+=y_offset
    new_pos[2]+=z_offset
    return new_pos


def decode_json(message):
   data = json.loads(message)
   
#    print(data)
#    print(f"**DECODED**Gripper status: {data['gripper']}, X: {data['x_coordinate']}, Y: {data['y_coordinate']}, Z: {data['z_coordinate']}")
   return data



def main():
    global message_recieved, message_i_got
    
    num_loops = 3
    
    
    client = connect_mqtt()
    client.loop_start()

    print('Setting up...')
    bill = robot(robot_ip)
    bill.set_speed(300)

    bill.onRobot_gripper(open_width,force)
    time.sleep(.5)
    
    # bill.write_cartesian_position(home_pos)
    bill.write_joint_pose(home_pos_joint)
    bill.write_cartesian_position(above_dice_start)
    bill.write_cartesian_position(on_dice_start)
    bill.onRobot_gripper(close_width,force)
    time.sleep(.5)
    bill.write_cartesian_position(home_pos)
    
    bill.write_cartesian_position(safe_pose)
    
    # Go to random position, LOOP STARTS HERE
    for i in range(1,num_loops+1):
        print(f"***START OF LOOP {i}***")
        
        rand_pos=find_random_position()
        bill.write_cartesian_position(rand_pos)
        
        msg=package_json(rand_pos, True)
        publish(client, msg)

        message_recieved = False
        subscribe(client)
        while not message_recieved:  # wait for gripper to close, eventually actually look at message
            time.sleep(0.2)
        
        bill.onRobot_gripper(open_width,force)
        time.sleep(.5)

        curpos=bill.read_current_cartesian_pose()
        curpos[1]+=50
        bill.write_cartesian_position(curpos)
        bill.write_cartesian_position(safe_pose)
        msg=package_json(safe_pose, True)
        publish(client, msg)# Tell DJ I am away
        
        message_recieved = False
        subscribe(client)
        while not message_recieved:  # wait for position
            time.sleep(0.2)
        
        new_pos = get_new_pos()
        intermediate_pos=new_pos.copy()
        intermediate_pos[1]+=50
        bill.write_cartesian_position(intermediate_pos)
        bill.write_cartesian_position(new_pos)
        bill.onRobot_gripper(close_width,force)
        time.sleep(.5)
        
        if(i!=num_loops):
            msg=package_json(new_pos, False) # Tell DJ I have the die
        else:
            msg=package_json(new_pos, False, loop_finished=True)
        publish(client, msg)
        
        
        print(f"***END OF LOOP {i}***")
        
        subscribe(client)
        message_recieved = False
        while not message_recieved:  # wait for dj to be away
            time.sleep(0.2)
        
    # END LOOP HERE
    
    bill.write_joint_pose(home_pos_joint)
    bill.write_cartesian_position(above_dice_start)
    bill.write_cartesian_position(on_dice_start)
    bill.onRobot_gripper(open_width,force)
    time.sleep(.5)
    bill.write_cartesian_position(home_pos)
    
    client.loop_stop()
    client.disconnect()



start_dt = datetime.now()

main()


end_dt = datetime.now()
execution_duration = end_dt - start_dt
print(f"Program duration: {execution_duration}")