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

for i in range(0,4):
   # Robot move die to somewhere
   # Save the x,y for the location
   robot_x = 0
   robot_y = 0
   real_points[i] = [robot_x, robot_y]
   # Move the robot out of the way
   img=detect_and_count.take_photo()
   x,y,w,h = detect_and_count.find_die(img)
   # Save die center coordinates
   image_points[i]=[(x+(0.5*w)),(y+(0.5*h))]
   # Pick die back up


# Compute homography matrix
H, _ = cv2.findHomography(image_points, real_points)

# Use it to convert a pixel (u,v) to real (X,Y)
pixel = np.array([u, v, 1], dtype=np.float32)
real = H @ pixel
real /= real[2]  # normalize

X, Y = real[0], real[1]
print(X, Y)
