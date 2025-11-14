#coding=utf-8
import cv2
import numpy as np
import mvsdk
import platform
import detect_and_count
from standardbots import models, StandardBotsRobot
import time
import random
import sys
sys.path.append('fanuc_ethernet_ip_drivers/src')
from robot_controller import robot
import paho.mqtt.client as mqtt_client
import json

# Michael is the broker
broker = '10.8.4.17'
# broker port
port = 1883
# Just a declaration of the global variable
topic = "robot/test"
# Ip of DJ robot
dj_ip = '10.8.4.16'
# Topic that DJ will publish to
dj_topic = "robot/dj"
theo_topic = "robot/theodore"
sdk = StandardBotsRobot(
    url="http://10.8.4.11:3000",
    token="8geqfqu0-qbbkig-ozwgr4-tl2xfj7",
    robot_kind=StandardBotsRobot.RobotKind.Live
)
position = 10.0
DJ=robot(dj_ip)
home_position = [0.0,0.0,0.0,0.0,-90.0,30.0]
DJ.write_joint_pose(home_position)
def home_robot():
    DJ.write_joint_pose(home_position)
    time.sleep(1)
def pickup_die(x, y, z=122.70266723632812):
    DJ.write_cartesian_position([x, y, z + 100.0, -179.5, 0, 120])
    time.sleep(0.25)
    DJ.write_cartesian_position([x, y, z, -179.5, 0, 120])
    time.sleep(0.25)
    DJ.schunk_gripper('close')
    time.sleep(0.25)
    DJ.write_cartesian_position([x, y, z + 100.0, -179.5, 0, 120])
    time.sleep(0.25)

def save_H_mtx(H):
    np.savetxt("homography_mtx_DJ.txt", H)

def read_H_mtx():
    H_loaded = np.loadtxt("homography_mtx_DJ.txt")
    return H_loaded

def find_die_in_image():
    # Read in H mtx
    H = read_H_mtx()
    # Take photo of die
    img=detect_and_count.take_photo()
    x,y,_,_ = detect_and_count.find_die(img)
    # Use it to convert a pixel (x,y) to real (X,Y)
    pixel = np.array([x, y, 1], dtype=np.float32)
    real = H @ pixel
    real /= real[2] # normalize
    X, Y = real[0], real[1]
    print(X, Y)
    return X, Y
    # Send robot to X,Y and pick up

def find_homography_matrix():
    pickup_die(844.6377563476562, 353.57177734375, 125.70266723632812)
    DJ.write_cartesian_position([844.6377563476562, 353.57177734375, 125.70266723632812, -179.5, 0, 120])
    offset = 300.0
    px = 450.0
    py = 650.0
    DJ.write_cartesian_position([px, py, 225.70266723632812, -179.5, 0, 120])
    time.sleep(1)
    DJ.write_cartesian_position([px, py, 125.70266723632812, -179.5, 0, 120])
    DJ.schunk_gripper('open')
    DJ.write_cartesian_position([px, py, 222.70266723632812, -179.5, 0, 120])
    home_robot()
    real_points = np.array([
    [px,py],
    [px,py + offset],
    [px - offset,py + offset],
    [px - offset,py],
    ], dtype=np.float32)
    
    # Corresponding real-world points (robot coordinates)
    image_points = np.array([
    [0,0],
    [0,0],
    [0,0],
    [0,0]
    ], dtype=np.float32)

    for i in range(0,4):
        # Move the robot out of the way
        img=detect_and_count.take_photo()
        x,y,w,h = detect_and_count.find_die(img)
        # Save die center coordinates
        image_points[i]=[(x+(0.5*w)),(y+(0.5*h))]
        # Pick die back up
        pickup_die(float(real_points[(i) % 4][0]), float(real_points[(i) % 4][1]))
        DJ.write_cartesian_position([float(real_points[(i) % 4][0]), float(real_points[(i) % 4][1]), 222.70266723632812, -179.5, 0, 120])
        DJ.write_cartesian_position([float(real_points[(i + 1) % 4][0]), float(real_points[(i + 1) % 4][1]), 125.70266723632812, -179.5, 0, 120])
        time.sleep(1)
        DJ.write_cartesian_position([float(real_points[(i+1) % 4][0]), float(real_points[(i+1) % 4][1]), 125.70266723632812, -179.5, 0, 120])
        DJ.schunk_gripper('open')
        DJ.write_cartesian_position([float(real_points[(i + 1) % 4][0]), float(real_points[(i + 1) % 4][1]), 222.70266723632812, -179.5, 0, 120])
        home_robot()
    

    # Compute homography matrix
    H, _ = cv2.findHomography(image_points, real_points)
    save_H_mtx(H)
    print("Homography Matrix:")
    print(H)
    # Use it to convert a pixel (u,v) to real (X,Y)
    #pixel = np.array([u, v, 1], dtype=np.float32)
    #real = H @ pixel
    #real /= real[2]  # normalize

    #X, Y = real[0], real[1]
    #print(X, Y)

def main():
    #try:
        #main_loop()
    #finally:
    #    cv2.destroyAllWindows()
    DJ.set_speed(300)
    DJ.schunk_gripper('open')
    home_robot()
    time.sleep(1)
    #find_homography_matrix()
    x_offset = 45.0
    y_offset = 50.0

    x, y = find_die_in_image()
    pickup_die(x + x_offset, y + y_offset)
    #find_homography_matrix()
    #pickup_die(x + x_offset, y + y_offset)
    #calibrate_robot_with_camera()
    #pickup_die(x=-0.885, y=0.460)
main()
