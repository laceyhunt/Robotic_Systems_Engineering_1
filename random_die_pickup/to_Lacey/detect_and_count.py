#coding=utf-8
import cv2
import numpy as np
import mvsdk
import platform
from standardbots import StandardBotsRobot
sdk = StandardBotsRobot(
    url="https://mybot.sb.app",
    token="token",
    robot_kind=StandardBotsRobot.RobotKind.Live
)
# Text params
fontFace = cv2.FONT_HERSHEY_SIMPLEX 
fontScale = 3 
color = (0, 0, 255) 
thickness = 4
num_windows=0
def show_img(img,window='default',text=None,
                                    scale=3,
                                    thick=5,
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
    cv2.imshow(window, print_img)


def count_pips(img,orig_x, orig_y,die_num, p=None):
    pips, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    print("Found a die...")
    num_pips=0
    for pip in pips:
        # Filter out small contours that might be noise
        if (cv2.contourArea(pip) > 12) and (cv2.contourArea(pip)<100):  # For the outside of the pips...
            print(cv2.contourArea(pip))
            num_pips+=1
            x, y, w, h = cv2.boundingRect(pip)
            # cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 6)
            cv2.rectangle(img, (orig_x+x, orig_y+y), (orig_x+x + w, orig_y+y + h), (0, 0, 0), 2)
    text = "Die "+ str(die_num)+ " has " + str(num_pips)+" pips"
    print(text)
    return num_pips

def take_photo():
    ret_frame=None # what we will be returning
    # Enumerate cameras
    DevList = mvsdk.CameraEnumerateDevice()
    nDev = len(DevList)
    if nDev < 1:
        print("No camera was found!")
        return

    for i, DevInfo in enumerate(DevList):
        print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))
    i = 0 if nDev == 1 else int(input("Select camera: "))
    DevInfo = DevList[i]
    # Open the camera
    hCamera = 0
    try:
        hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
    except mvsdk.CameraException as e:
        print("CameraInit Failed({}): {}".format(e.error_code, e.message))
        return
    # Get camera capability description
    cap = mvsdk.CameraGetCapability(hCamera)
    # Determine whether it’s a monochrome or color camera
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)
    # For monochrome cameras, let ISP output MONO data directly
    # instead of expanding it into R=G=B 24-bit grayscale
    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)
    # Set camera mode to continuous acquisition
    mvsdk.CameraSetTriggerMode(hCamera, 0)

    # Manual exposure, exposure time = 80 ms
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera,29 * 1000)

    # Start the SDK’s internal image capture thread
    mvsdk.CameraPlay(hCamera)

    # Calculate the required size of the RGB buffer
    # Allocate it according to the camera’s maximum resolution
    FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)

    # Allocate the RGB buffer, used to store the ISP’s output image
    # Note: The data transmitted from the camera to the PC is RAW.
    # On the PC side, software ISP converts it to RGB (for color cameras).
    # Even for monochrome cameras, ISP still does processing, so the buffer is still required.
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)
    # Capture one frame from the camera
    try:
        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
        mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
        mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
        
        # At this point, the image is stored in pFrameBuffer.
        # For color cameras: pFrameBuffer = RGB data.
        # For monochrome cameras: pFrameBuffer = 8-bit grayscale data.
        # Convert pFrameBuffer into an OpenCV image for further processing.
        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))
        # cv2.imwrite("frame.jpg", frame)
        # frame = cv2.resize(frame, (3840, 2160), interpolation=cv2.INTER_LINEAR)
        # find_dice(frame)
        ret_frame=frame.copy()
        
    except mvsdk.CameraException as e:
        if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
            print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message))
    # Close the camera
    mvsdk.CameraUnInit(hCamera)

    # Free the frame buffer
    mvsdk.CameraAlignFree(pFrameBuffer)
    return ret_frame


image = take_photo()
#show_img(image,'1','1: Original Image')

def find_die(image):
    hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    # show_img(hsv_img,'2','2: HSV Image')
    lower_yellow = np.array([15,40,40])
    upper_yellow = np.array([25,255,255])
    mask = cv2.inRange(hsv_img,lower_yellow,upper_yellow)
    # show_img(mask,'3','3: Mask')

    # # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    print(f"Num contours: {len(contours)}")
    num_dice=0
    for contour in contours:
        # Filter out small contours that might be noise
        if cv2.contourArea(contour) > 1500:  # For the outside of the die...
            # print(cv2.contourArea(contour))
            num_dice+=1
            x, y, w, h = cv2.boundingRect(contour)
            return x,y,w,h
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green bounding box
            # Define region of interest (new img with those coords)
            # die_face = mask[y:(y + h), x:(x + w)].copy()
            # num_pips=count_pips(die_face,x,y, num_dice)
            # text =str(num_dice)+": " + str(num_pips)#+" pips"
            print(f"Found die # {str(num_dice)}")
            org = (x, y) 
            # Put the text on the image
            # cv2.putText(image, text, org, fontFace, 1, color, thickness)#, lineType)

    # show_img(image,'5','Final Image')



cv2.waitKey(0)
cv2.destroyAllWindows()