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
sys.path.append('../ASN3/fanuc_ethernet_ip_drivers/src')
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
def get_position_info():
    with sdk.connection():
        sdk.movement.brakes.unbrake().ok()
        response = sdk.movement.position.get_arm_position()

        try:
            data = response.ok()
            j_1, j_2, j_3, j_4, j_5, j_6 = data.joint_rotations
            position = data.tooltip_position.position
            orientation = data.tooltip_position.orientation
            joints = data.joint_rotations
            print(f"Joints: {joints}")
            print(f"Position: {position}")
            print(f"Orientation: {orientation}")
            return j_1, j_2, j_3, j_4, j_5, j_6
        except Exception as e:
            print(response.data.message)
            return None
def move_robot_cartesian(x,y,z,rx=-0.503444496073135,ry=-0.48649383205875135,rz=0.5136321065685251,rw=-0.4960332468544624):
    with sdk.connection():
        sdk.movement.brakes.unbrake().ok()
        sdk.movement.position.move(
            position=models.Position(
                unit_kind=models.LinearUnitKind.Meters,
                x=x,
                y=y,
                z=z
            ),
            orientation=models.Orientation(
                kind=models.OrientationKindEnum.Quaternion,
                quaternion=models.Quaternion(rx,ry,rz,rw),
            ),
        ).ok()
def move_robot_joint(j1, j2, j3, j4, j5, j6):
    with sdk.connection():
        sdk.movement.brakes.unbrake().ok()
        arm_rotations = models.ArmJointRotations(joints=(j1,j2,j3,j4,j5,j6))

        position_request = models.ArmPositionUpdateRequest(
            kind=models.ArmPositionUpdateRequestKindEnum.JointRotation,
            joint_rotation=arm_rotations,
        )
        sdk.movement.position.set_arm_position(position_request).ok()
def grab_cartesian_position():
    with sdk.connection():
        response = sdk.movement.position.get_arm_position()

        try:
            data = response.ok()
            position = data.tooltip_position.position
            orientation = data.tooltip_position.orientation
            print(f"Position: {position}")
            print(f"Orientation: {orientation}")
            return position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w
        except Exception as e:
            print(response.data.message)
            return None
def gripper_request(WIDTH,FORCE):
    with sdk.connection():
        response = sdk.equipment.control_gripper(
            models.GripperCommandRequest(
            kind=models.GripperKindEnum.Onrobot2Fg14,
            onrobot_2fg14=models.OnRobot2FG14GripperCommandRequest(
                grip_direction=models.LinearGripDirectionEnum.Inward,
                target_grip_width=models.LinearUnit(
                    value=WIDTH,
                    unit_kind=models.LinearUnitKind.Meters
                ),
                target_force=models.ForceUnit(
                    value=FORCE,
                    unit_kind=models.ForceUnitKind.Newtons,
                ),
                control_kind=models.OnRobot2FG14ControlKindEnum.Move,
            ),
            )
        )
    try:
        response.ok()
    except Exception:
        print(response.data.message)
def gripper_command(open):
    if open:
        gripper_request(0.11,10.0)
    else:
        gripper_request(0.036,10.0)
def home_robot():
    move_robot_joint(0.0,0.0,-1.57,0.0,1.57,-3.14)
def find_dice(img):
    def count_pips(sub_img):
        gray_sub = cv2.cvtColor(sub_img, cv2.COLOR_BGR2GRAY)
        kernel = np.ones((3, 3), np.uint8)
        _, thresh_sub = cv2.threshold(gray_sub, 0, 1, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        roi_thresh = cv2.morphologyEx(thresh_sub, cv2.MORPH_OPEN, kernel, iterations=2)
        contours_sub = cv2.findContours(roi_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        pip_count = 0
        for cnt in contours_sub:
            area = cv2.contourArea(cnt)
            x, y, w, h = cv2.boundingRect(cnt)
            #if (0.5*h < w < 1.5*h and area > 0):
            pip_count += 1
            cv2.rectangle(sub_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return pip_count
    
    # Scale down the image so it fits on the screen and processes faster
    #img_scaled = cv2.resize(img, (0, 0), fx=0.25, fy=0.25)
    # Preprocess the image to find the dice
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Define range for dice color in HSV
    lower_mask = np.array([15, 40, 40])
    upper_mask = np.array([28, 255, 255])
    # Create a mask for the dice color
    mask = cv2.inRange(hsv, lower_mask, upper_mask)
    # Apply the mask to get the dice regions
    masked_img = cv2.bitwise_and(img, img, mask=mask)
    # Display the masked image because it looks cool
    cv2.imshow('Masked Image', masked_img)
    # Find contours of the dice
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    boxes = []
    # For each contour found, draw bounding boxes and count pips
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)

        sub_img = img[y:y + h, x:x + w]
        if w > 75 and h > 75:
            boxes.append((x, y, w, h))
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.astype(box, np.int32)
            cv2.drawContours(img, [box], 0, (0, 0, 255), 2)
            #cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            pip_count = count_pips(sub_img)
            cv2.putText(img, f"Counted Pips: {pip_count}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    # Show the final image with detected dice and pip counts
    cv2.imshow('Detected Lines', img)
    # Wait indefinitely until a key is pressed
    cv2.waitKey(0)
def pickup_die(x, y, z=0.37929102710513794):
    move_robot_cartesian(x, y, z + 0.1)
    time.sleep(0.25)
    move_robot_cartesian(x, y, z)
    time.sleep(0.25)
    gripper_command(False)  # Close gripper to pick up die
    time.sleep(0.25)
    move_robot_cartesian(x, y, z + 0.1)
    time.sleep(0.25)

def save_H_mtx(H):
    np.savetxt("homography_mtx_Theo.txt", H)

def read_H_mtx():
    H_loaded = np.loadtxt("homography_mtx_Theo.txt")
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
    pickup_die(x=-0.885, y=0.460)
    offset = 0.3
    px = -0.5
    py = 0.75
    move_robot_cartesian(x=px, y=py, z=0.47929102710513794)
    time.sleep(1)
    move_robot_cartesian(x=px, y=py, z=0.37929102710513794)
    gripper_command(True)
    move_robot_cartesian(x=px, y=py, z=0.47929102710513794)
    home_robot()
    real_points = np.array([
    [px,py],
    [px,py + offset],
    [px + offset,py + offset],
    [px + offset,py],
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
        pickup_die(x=float(real_points[(i) % 4][0]), y=float(real_points[(i) % 4][1]))
        move_robot_cartesian(x=float(real_points[(i) % 4][0]), y=float(real_points[(i) % 4][1]), z=0.47929102710513794)
        move_robot_cartesian(x=float(real_points[(i + 1) % 4][0]), y=float(real_points[(i + 1) % 4][1]), z=0.47929102710513794)
        time.sleep(1)
        move_robot_cartesian(x=float(real_points[(i+1) % 4][0]), y=float(real_points[(i+1) % 4][1]), z=0.37929102710513794)
        gripper_command(True)
        move_robot_cartesian(x=float(real_points[(i + 1) % 4][0]), y=float(real_points[(i + 1) % 4][1]), z=0.47929102710513794)
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
    x_offset = -0.04
    y_offset = -0.05
    gripper_command(True)
    get_position_info()
    home_robot()
    x, y = find_die_in_image()
    #find_homography_matrix()
    pickup_die(x + x_offset, y + y_offset)
    #calibrate_robot_with_camera()
    #pickup_die(x=-0.885, y=0.460)
main()
