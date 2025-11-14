import detect_and_count
from detect_and_count import cv2, np

# Image points (pixels)
image_points = np.array([
   [0,0],
   [0,0],
   [0,0],
   [0,0]
], dtype=np.float32)

# Corresponding real-world points (robot coordinates)
real_points = np.array([
   [0,0],
   [0,0],
   [0,0],
   [0,0]
], dtype=np.float32)

def save_H_mtx(H,filename="homography_mtx.txt"):
   """saves H mtx to txt file

   Args:
      H: Homography matrix
   """
   np.savetxt(filename, H)

def read_H_mtx(filename="homography_mtx.txt"):
   """reads homography martrix file into list of lists

   Returns:
      H: loaded H mtx
   """
   H_loaded = np.loadtxt(filename)
   return H_loaded

def calibrate_H_mtx():
   """skeleton to calibrate a H matrix for conversion. This will be different for each robot (bc they have different move methods)
   """
   for i in range(0,4):
      # Robot move die to somewhere
      # Save the x,y for the location
      robot_x = 0
      robot_y = 0
      real_points[i] = [robot_x, robot_y]
      # Move the robot out of the way
      img=detect_and_count.take_photo()
      x,y,w,h = locate_die(img,calib=True)
      # Save die center coordinates
      image_points[i]=[(x+(0.5*w)),(y+(0.5*h))]
      # Pick die back up
   # Compute homography matrix
   H, _ = cv2.findHomography(image_points, real_points)
   save_H_mtx(H)

      
def convert_pix_to_robot_coords(x,y,w,h,x_offset=100,y_offset=120):
   """converts some pixel coordinate to a robot coordinate
         offsets will be different depending on the robot

   Args:
       x: x (corner)
       y: y (corner)
       w: width
       h: height

   Returns:
       tuple: (x,y) robot coordinate
   """
   # Read in H mtx
   H = read_H_mtx("theo_homography_mtx.txt")
   pixel = np.array([x, y, 1], dtype=np.float32)
   real = H @ pixel
   real /= real[2]  # normalize
   X, Y = real[0], real[1]
   X+=x_offset  
   Y+=y_offset   
   # print(X, Y)
   return (X,Y)

def break_cluster(coordinate,width=1, height=2):
   """break up a cluster

   Args:
       coordinate (Tuple): x,y robot coord of center of cluster
       width (float, optional): width of cluster in pixels. Defaults to None.
       height (_type_, optional): height of cluster in pixels. Defaults to None.
   """
   print("cluster found!")
   if(width>height):
      # go to bottom of table with y=coordinate y
      # Move robot up to x=coordinate x + some 
      pass
   elif(height>=width):
      # go to side of table with x=coordinate x
      # Move robot sideways to y=coordinate y + some 
      pass
   # Go home


def locate_die(image, calib=False):
   """_summary_

   Args:
       image (_type_): _description_
   """
   with open('img_die_loc.txt', 'w') as file:
      file.write("Die locations from image (x,y,w,h)\n")
   hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
   # show_img(hsv_img,'2','2: HSV Image')
   lower_yellow = np.array([15,40,40])
   upper_yellow = np.array([25,255,255])
   mask = cv2.inRange(hsv_img,lower_yellow,upper_yellow)
   median = cv2.medianBlur(mask,5)
   # show_img(mask,'3','3: Mask')

   # # Find contours in the mask
   contours, _ = cv2.findContours(median, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
   # print(f"Num contours: {len(contours)}")

   num_dice=0
   for contour in contours:
      # Filter out small contours that might be noise
      if cv2.contourArea(contour) > 1500:  # For the outside of the die...
         print(cv2.contourArea(contour))
         
         # CHECK FOR CLUSTERS!
         if cv2.contourArea(contour) > 3500:
            break_cluster(0,0)                  # Will be different for each person, will write
            return 0
         num_dice+=1
         
         # Get rotated rectangle
         rect = cv2.minAreaRect(contour)
         box = cv2.boxPoints(rect)
         box = np.intp(box)
         (_,_), (wid, hei), angle = rect

         # Optional: normalize angle so long side aligns with horizontal
         # if wid < hei:
         #    angle = 90 + angle
         print(f"Die {num_dice} angle is {angle}")
         
         x, y, w, h = cv2.boundingRect(contour)
         # Going to look different if not sending mqtt... will read in from dictionary
         (x_coord,y_coord)=convert_pix_to_robot_coords(x,y,w,h)
         if calib:
            return x,y,w,h
         with open('img_die_loc.txt', 'a') as file:
               file.write(f"{float(x)},{float(y)},{float(w)},{float(h)}\n")
         cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green bounding box
         # Define region of interest (new img with those coords)
         die_face = median[y:(y + h), x:(x + w)].copy()
         num_pips=detect_and_count.count_pips(die_face,x,y, num_dice)
         text = str(num_pips)+" pips"
         text2 = str((round(float(x_coord), 2), round(float(y_coord), 2)))
         # print(f"Found die # {str(num_dice)}")
         org1 = (x, y-5)
         org2 = (x,y+h+20)
         # Put the text on the image
         cv2.putText(image, text, org1, detect_and_count.fontFace, 1, detect_and_count.red, detect_and_count.thickness)#, lineType)
         cv2.putText(image, text2, org2, detect_and_count.fontFace, 1, detect_and_count.blue, detect_and_count.thickness)#, lineType)
   if num_dice==0:
      print("\n\n\nNo dice detected! Quitting...")
      quit()
   detect_and_count.show_img(image,'Table Image','Locations are rounded to 2 d.p.')
   return 1