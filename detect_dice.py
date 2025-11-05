import cv2
import numpy as np

# Text params
fontFace = cv2.FONT_HERSHEY_SIMPLEX 
fontScale = 3 
color = (0, 0, 255) 
thickness = 10 
num_windows=0

def show_img(img,window='default',text=None,
                                    scale=8,
                                    thick=12,
                                    org = (50, 250),
                                    color = (255, 255, 255)):
    global num_windows
    horiz=num_windows
    vert=0
    print_img=img.copy()
    if(text):
        # Put the text on the image
        cv2.putText(print_img, text, org, fontFace, scale, color, thick)
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 400, 600)
    if num_windows%4==0:
        horiz=0
        if num_windows>0:
            vert=900
    # print(f"window {num_windows} at x={horiz}, y={vert}")
    cv2.moveWindow(window, (horiz*480), vert)
    cv2.imshow(window, print_img)
    num_windows+=1
    
def count_pips(img,orig_x, orig_y, p=None):
    pips, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    num_pips=0
    if(p):
        print_img=img.copy()
    for pip in pips:
        # Filter out small contours that might be noise
        if (cv2.contourArea(pip) > 1500) and (cv2.contourArea(pip)<10000):  # For the outside of the pips...
            num_pips+=1
            x, y, w, h = cv2.boundingRect(pip)
            # cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 6)
            cv2.rectangle(image, (orig_x+x, orig_y+y), (orig_x+x + w, orig_y+y + h), (255, 0, 0), 6)
            if(p):
                cv2.rectangle(print_img, (x, y), (x + w, y + h), (0, 0, 0), 2)
                cv2.putText(print_img, f"Pip #{num_pips}", (x,y-5), fontFace, 0.75, (0,0,0), 2)
                
    # print(num_pips," pips")
    # Define text properties
    text = str(num_pips)+" pips"
    org = (orig_x, orig_y) 
    # Put the text on the image
    if(p):
        show_img(print_img,'4', '4: Sample Die Face',scale=1,color=(0,0,0),thick=2,org=(5,80))
    cv2.putText(image, text, org, fontFace, fontScale, color, thickness)#, lineType)
    


image = cv2.imread('dice_on_table.jpg')
show_img(image,'1','1: Original Image')

hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
show_img(hsv_img,'2','2: HSV Image')

lower_yellow = np.array([25,100,100])
upper_yellow = np.array([35,255,255])
mask = cv2.inRange(hsv_img,lower_yellow,upper_yellow)
show_img(mask,'3','3: Mask')

# # Find contours in the mask
contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
# print(f"Num contours: {len(contours)}")
# print(contours)

# Draw bounding boxes around the detected objects
num_dice=0
for contour in contours:
    # Filter out small contours that might be noise
    if cv2.contourArea(contour) > 20000:  # For the outside of the die...
        num_dice+=1
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 6)  # Green bounding box
        # Define region of interest (new img with those coords)
        die_face = mask[y:(y + h), x:(x + w)].copy()
        if(num_dice==1):        
            count_pips(die_face,x,y,p='y')
        else:
            count_pips(die_face,x,y)

show_img(image,'5','Final Image')

cv2.waitKey(0)
cv2.destroyAllWindows()