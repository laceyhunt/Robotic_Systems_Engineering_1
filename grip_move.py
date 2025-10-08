import sys
import time
driver_dir = './FANUC-Ethernet_IP_Drivers/src/'
sys.path.append(driver_dir)
from robot_controller import robot


drive_path = '10.8.4.6' # Bill (OnRobot)
#drive_path = '10.8.4.2' # DJ (Schunk)


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
home_pos = [540, -150, 550, -179.9, 0, 0]
above_dice_start  = [621.617432, -1.351940, 97.204506, 179.479996, 4.369098, 0] # p3
on_dice_start = [628.230896, 0.380070, 50.960602, -179.9, 1.924459, 0] # p1
above_conveyor  = [-137.841873, -618.357849, 533.174805, -179.0, -2, -96] # p4
on_conveyor  = [-173.302933, -598.991699, 242.622101, -179.373367, -1.630008, -88.767609] # p5
above_moved_on_conveyor  = [-176.507, -799.084, 328, -179.785, -0.487, -90.657] # p6
on_moved_on_conveyor  = [-176.507, -799.084, 242.993, -179.785, -0.487, -90.657] # p7
rotated_pos = [-178.159, -629.112, 368.815, 57.47, -88.773, 31.438]
rotated_on_conveyor  = [-172.358, -675.829, 12.147, 161.2, -88.117, -72.448] # p9
above_dice_final = [-170, -948.831, 303.061, 178.524, 0, -91.625] # p10
on_dice_final = [-172.953, -948.833, 239.7, 178.524, 0, -91.625] # p11
mid_pos = [-186.956, -685.636, 477, -178.843, -42, -95]
# bill.onRobot_gripper(105,40)

bill.write_cartesian_position(home_pos)
