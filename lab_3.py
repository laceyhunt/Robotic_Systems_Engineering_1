"""Lab 3
   Lacey Bowden
   19 September 2025
   
   Description: recreate lab 2 using the Python Ethernet-IP Driver
   
   Steps:
      Robot home
      Pick up die from start position
      Place die on conveyor
      Turn on conveyor until die breaks proximity sensor
      Rotate die 90 degrees on x-axis
      Place die back at start location
      Robot home
"""

import sys
import time
driver_dir = './FANUC-Ethernet_IP_Drivers/src/'
sys.path.append(driver_dir)
from robot_controller import robot


drive_path = '10.8.4.6' # Bill (OnRobot)
#drive_path = '10.8.4.2' # DJ (Schunk)

def check_joint(robot,joint=6):
   """Checks a joint for potential singularity, untwists it before moving on.
   I did not end up using this function in this code, but it was useful for testing
      and I will probably use in later assignments to avoid singularities.

   Args:
      robot (_type_): instance of class robot()
      joint (int, optional): joint to check for possible singularity. Defaults to 6.
   """
   threshold = 150
   neg_threshold = threshold*(-1)
   joint_pos = robot.read_current_joint_position()
   print('CHECKING!!! Current Joint positions:')
   print(joint_pos)
   print(f'Joint {joint} of interest position:')
   print(joint_pos[joint-1])
   if(joint_pos[joint-1] < neg_threshold) or (joint_pos[joint-1] > threshold):
      # move that joint to 0
      robot.write_joint_position(joint,0) # untwist before next move

def main():
   """Main robot movement code
   """
   
   # Initialize Robot object
   print('Setting up...')
   bill = robot(drive_path)
   bill.set_speed(100)
   
   # Gripper specs (for OnRobot gripper)
   open_width = 100
   close_width = 78
   force = 40
   # JOINT positions (UNUSED, pulled from original .ls program from last lab)
   home_pos_joint = [0,0,0,0,-90,-179.9] 
   # CARTESIAN positions [x,y,z,w,p,r]
   home_pos = [540, -150, 550, -179.9, 0, 0]
   above_dice_start  = [621.617432, -1.351940, 97.204506, 179.479996, 4.369098, 0] 
   on_dice_start = [628.230896, 0.380070, 50.960602, -179.9, 1.924459, 0] 
   above_conveyor  = [-137.841873, -618.357849, 533.174805, -179.0, -2, -96] 
   on_conveyor  = [-173.302933, -598.991699, 242.622101, -179.373367, -1.630008, -88.767609] 
   above_moved_on_conveyor  = [-176.507, -799.084, 328, -179.785, -0.487, -90.657] 
   on_moved_on_conveyor  = [-176.507, -799.084, 242.993, -179.785, -0.487, -90.657]
   rotated_pos = [-178.159, -629.112, 368.815, 57.47, -88.773, 31.438]
   rotated_on_conveyor  = [-172.358, -675.829, 12.147, 161.2, -88.117, -72.448]
   above_dice_final = [-170, -948.831, 303.061, 178.524, 0, -91.625]
   on_dice_final = [-172.953, -920.417, 239.7, 178.524, 0, -91.625] 
   mid_pos = [-186.956, -685.636, 477, -178.843, -42, -95]
   
   # Send robot to home
   print('Going home...')
   bill.write_cartesian_position(home_pos)
   
   print('Grabbing die...')
   # Open gripper
   bill.onRobot_gripper(open_width,force)
   # Program sleeps after every gripper command to avoid timeout error
   time.sleep(.5)
   # Move to die starting location
   bill.write_cartesian_position(above_dice_start)
   bill.write_cartesian_position(on_dice_start)
   # Close gripper
   bill.onRobot_gripper(close_width,force)
   time.sleep(.5)
   # Take die to home
   bill.write_cartesian_position(home_pos)

   # Move die to conveyor
   print('Placing die on conveyor...')
   bill.write_cartesian_position(above_conveyor)
   bill.write_cartesian_position(on_conveyor)
   # Open gripper
   bill.onRobot_gripper(open_width,force)
   time.sleep(.5)
   # Move robot out of the way
   bill.write_cartesian_position(above_conveyor)

   print('Moving die on conveyor...')   
   prox_sensor=0
   # Turn on conveyor reverse (so die moves left)
   bill.conveyor('reverse')
   # While proximity sensor not broken
   while not prox_sensor:
      # Check sensor
      prox_sensor=bill.conveyor_proximity_sensor('right')
      # Sleep to allow sensor to read
      time.sleep(0.1)
   # Turn off conveyor
   bill.conveyor('stop')
    
   print('Picking up die...')
   # Move robot to new die position
   bill.write_cartesian_position(above_moved_on_conveyor)
   bill.write_cartesian_position(on_moved_on_conveyor)
   # Close gripper
   bill.onRobot_gripper(close_width,force)
   time.sleep(.5)
   # Pick up die
   bill.write_cartesian_position(above_moved_on_conveyor)
   
   print('Rotating die...')
   # Rotate die in air above conveyor
   bill.write_cartesian_position(rotated_pos)
   
   print('Placing die back on conveyor...')
   # Place rotated die back on conveyor
   bill.write_cartesian_position(rotated_on_conveyor)
   # Open gripper
   bill.onRobot_gripper(open_width,force)
   time.sleep(.5)

   print('Un-twisting...')
   # Move robot to neutral position slowly to avoid singularities
   bill.write_cartesian_position(rotated_pos)
   bill.write_cartesian_position(mid_pos)
   bill.write_cartesian_position(above_conveyor)
       
   print('Picking up die...')
   # Pick up die from final spot
   bill.write_cartesian_position(above_dice_final)
   bill.write_cartesian_position(on_dice_final)
   # Close gripper
   bill.onRobot_gripper(close_width,force)
   time.sleep(.5)
   # Back away from pick up spot
   bill.write_cartesian_position(above_dice_final)
   bill.write_cartesian_position(above_conveyor)
   
   print('Placing die back at start...')
   # Move back home
   bill.write_cartesian_position(home_pos)
   
   # Place die back at start
   bill.write_cartesian_position(above_dice_start)
   bill.write_cartesian_position(on_dice_start)
   # Open gripper
   bill.onRobot_gripper(open_width,force)
   time.sleep(.5)
   bill.write_cartesian_position(above_dice_start)
   
   print('All done, going home...')
   # Move robot back home and done
   bill.write_cartesian_position(home_pos)

if __name__=="__main__":
   main()