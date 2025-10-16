import sys
import time
from paho.mqtt import client as mqtt_client
import random
import json
driver_dir = './FANUC-Ethernet_IP_Drivers/src/'
sys.path.append(driver_dir)
from robot_controller import robot
from datetime import datetime

# Michael's IP
broker = '10.8.4.13'
port = 1883

bill_topic = 'robot/bill'
dj_topic = 'robot/dj'

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

def publish(client, msg):
    """Publishes a message to the 'robot/bill' topic

    Args:
        client (mqtt client): the mqtt object loop that has been started
        msg (json object): message to publish
    """
    result = client.publish(bill_topic, msg)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic `{bill_topic}`")
    else:
        print(f"Failed to send message to topic {bill_topic}")
      

def subscribe(client: mqtt_client):
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
handoff_middle_pos = [250,-900,125,145,-90,-55]   # could be 88 instead of 90
safe_pose = [250,-600,125,145,-90,-55] 

def find_random_position():
    """finds a random position reachable by both robots to hand off the die
        randomized position +-150 cm from the center handoff location

    Returns:
        list of floats: six-value list of cartesian coordinates (x,y,z,w,p,r)
    """
    global handoff_middle_pos
    new_pos=handoff_middle_pos.copy()
    x=handoff_middle_pos[0]
    y=handoff_middle_pos[1]
    z=handoff_middle_pos[2]
    new_pos[0]=random.randint(x-150,x+150)
    new_pos[1]=random.randint(y-150,y+150)
    new_pos[2]=random.randint(z-150,z+150)
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
    """translates coordinates from DJ coordinate system to Bill's

    Returns:
        list of floats: six-value list of cartesian coordinates (x,y,z,w,p,r)
    """
    global message_i_got
    new_pos = [message_i_got['x_coordinate'], message_i_got['y_coordinate'], message_i_got['z_coordinate'], yaw, pitch, roll]
    new_pos[1]+=y_offset
    new_pos[2]+=z_offset
    return new_pos


def decode_json(message):
    """decodes json object into a dictionary and returns

    Args:
        message (json object): the encoded json object in the format:
        data = {
            "gripper": gripper_status,
            "x_coordinate": coordinate[0],
            "y_coordinate": coordinate[1],
            "z_coordinate": coordinate[2],
            "loop_finished": loop_finished
        }

    Returns:
        dict: decodded json object
    """
    data = json.loads(message)
    return data


def main():
    """Handles all logic for Bill (robot A) in MQTT die handoff including timing, MQTT pub/sub, robot operation, and loop counting
    """
    global message_recieved, message_i_got
    num_loops = 3
    
    # Initialize client
    client = connect_mqtt()
    client.loop_start()

    # Set up robot
    print('Setting up...')
    bill = robot(robot_ip)
    bill.set_speed(300)

    # Open gripper
    bill.onRobot_gripper(open_width,force)
    time.sleep(.5)
    
    # Go home, untwist any joints if needed
    bill.write_joint_pose(home_pos_joint)
    
    # Pick up die
    bill.write_cartesian_position(above_dice_start)
    bill.write_cartesian_position(on_dice_start)
    bill.onRobot_gripper(close_width,force)
    time.sleep(.5)
    bill.write_cartesian_position(home_pos)
    
    # Move just outside handoff bounds
    bill.write_cartesian_position(safe_pose)
    
    # HANDOFF LOOP STARTS HERE
    for i in range(1,num_loops+1):
        print(f"***START OF LOOP {i}***")
        
        # Go to random handoff position
        rand_pos=find_random_position()
        bill.write_cartesian_position(rand_pos)
        
        # Publish position
        msg=package_json(rand_pos, True)
        publish(client, msg)

        # Wait for DJ to close his gripper on the die
        message_recieved = False
        subscribe(client)
        while not message_recieved:  # wait for gripper to close
            time.sleep(0.2)
        
        # Release die
        bill.onRobot_gripper(open_width,force)
        time.sleep(.5)
        
        # Carefully back away
        curpos=bill.read_current_cartesian_pose()
        curpos[1]+=50
        bill.write_cartesian_position(curpos)
        
        # Return to safe position outside handoff bounds
        bill.write_cartesian_position(safe_pose)
        
        # Publish that Bill is at safe location
        msg=package_json(safe_pose, True)
        publish(client, msg)
        
        # Wait for DJ to go to and send new handoff coordinates
        message_recieved = False
        subscribe(client)
        while not message_recieved:
            time.sleep(0.2)
        
        # Translate received coordinate to Bill coordinate system
        new_pos = get_new_pos()
        # Carefully approach die by going 50 cm to the right of it and then moving to it
        intermediate_pos=new_pos.copy()
        intermediate_pos[1]+=50
        bill.write_cartesian_position(intermediate_pos)
                
        # Grab die
        bill.write_cartesian_position(new_pos)
        bill.onRobot_gripper(close_width,force)
        time.sleep(.5)
        
        # Set loop_finished flag
        if(i!=num_loops):
            msg=package_json(new_pos, False) # Tell DJ I have the die
        else:
            msg=package_json(new_pos, False, loop_finished=True)
        # Publish gripper status and loop_finished flag 
        publish(client, msg)
        
        print(f"***END OF LOOP {i}***")
        
        # Wait for DJ to move to safe location
        subscribe(client)
        message_recieved = False
        while not message_recieved:
            time.sleep(0.2)
    # END LOOP HERE
    
    # Return home
    bill.write_joint_pose(home_pos_joint)
    
    # Set die back at start location
    bill.write_cartesian_position(above_dice_start)
    bill.write_cartesian_position(on_dice_start)
    bill.onRobot_gripper(open_width,force)
    time.sleep(.5)
    
    # Go home
    bill.write_cartesian_position(home_pos)
    
    # Disconnect MQTT
    client.loop_stop()
    client.disconnect()

# Start timing
start_dt = datetime.now()

# Execute main
main()

# Stop timing
end_dt = datetime.now()

# Calculate and print program duration
execution_duration = end_dt - start_dt
print(f"Program duration: {execution_duration}")