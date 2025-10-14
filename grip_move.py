import sys
import time
driver_dir = './FANUC-Ethernet_IP_Drivers/src/'
sys.path.append(driver_dir)
from robot_controller import robot


drive_path = '10.8.4.6' # Bill (OnRobot)
# drive_path = '10.8.4.16' # DJ (Schunk)


print('Setting up...')
bill = robot(drive_path)
bill.set_speed(100)


# Gripper specs (for OnRobot gripper)
open_width = 100
close_width = 78
force = 40
# JOINT positions
home_pos_joint = [0,0,0,0,-90,-179.9] # p2
# CARTESIAN positions
handoff_middle_pos = [200,-900,100,145,-90,-55]
# bill.onRobot_gripper(105,40)
above_dice_start  = [621.617432, -1.351940, 97.204506, 179.479996, 4.369098, 0] 
on_dice_start = [628.230896, 0.380070, 50.960602, -179.9, 1.924459, 0] 
on_dice_start = [640, 4.3, 49.328, 178.758, 0, .94]
above_dice_start = [640, 4.3, 150, 178.758, 0, .94]
home_pos = [540, -150, 550, -179.9, 0, 0]

bill.onRobot_gripper(open_width,force)
time.sleep(.5)


home_pos_joint = [0,0,0,0,-90,0]
bill.write_joint_pose(home_pos_joint)