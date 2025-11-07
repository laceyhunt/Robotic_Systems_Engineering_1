import detect_and_count
import mqtt
import homography_mtx
from detect_and_count import cv2, np
import time
from standardbots import StandardBotsRobot
sdk = StandardBotsRobot(
    url="https://mybot.sb.app",
    token="token",
    robot_kind=StandardBotsRobot.RobotKind.Live
)
# Offsets for theo
x_offset=-0.04
y_offset=-0.05


def main():
   coords=[]
   
   cluster_free = 0
   while not cluster_free:
      img=detect_and_count.take_photo()
      cluster_free=homography_mtx.locate_die(img)
      time.sleep(2)
   img_coords = homography_mtx.read_dice_pixel_coords()
   print("---Image Coordinates---")
   print(img_coords)
   # for coord in img_coords:
   for i in range(0,len(img_coords)):
      # x,y,w,h
      (x,y)=homography_mtx.convert_pix_to_robot_coords(img_coords[i][0],img_coords[i][1],img_coords[i][2],img_coords[i][3])
      coords.append((float(x),float(y)))
   print("---Real World Coordinates---")
   print(coords)
   cv2.waitKey(0)
   cv2.destroyAllWindows()
   
main()















# de