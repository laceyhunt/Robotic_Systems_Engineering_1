import cv2
import numpy as np
import mvsdk
import platform

# Text params
fontFace = cv2.FONT_HERSHEY_SIMPLEX 
fontScale = 1
red = (0, 0, 255) 
blue = (255, 0, 0) 
thickness = 2
num_windows=0
def show_img(img,window='default',text=None):
    """shows an image on a resized window

    Args:
        img: image to show
        window (str, optional): Name of window. Defaults to 'default'.
        text (string, optional): text to display on window. Defaults to None.
    """
    global num_windows
    # horiz=num_windows
    # vert=0
    print_img=img.copy()
    if(text):
        cv2.putText(img, text, (200,0), fontFace, 1, red, thickness)
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 900, 720)
    cv2.imshow(window, print_img)

def take_photo():
    """takes and returns photo

    Returns:
        img: image taken
    """
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
    cap = mvsdk.CameraGetCapability(hCamera)
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)
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
    # Allocate it according to the camera’s maximum resolution
    FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)
    # Allocate the RGB buffer
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)
    # Capture one frame from the camera
    try:
        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
        mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
        mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))
        ret_frame=frame.copy()        
    except mvsdk.CameraException as e:
        if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
            print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message))
    # Close the camera
    mvsdk.CameraUnInit(hCamera)
    # Free the frame buffer
    mvsdk.CameraAlignFree(pFrameBuffer)
    return ret_frame

def count_pips(img,orig_x, orig_y,die_num):
    """counts pips in a cropped image

    Args:
        img: cropped die face img
        orig_x: x coord (note: in corner)
        orig_y: y coord (note: in corner)
        die_num: which die it is

    Returns:
        int: number of pips
    """
    pips, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    # print("Found a die...")
    num_pips=0
    for pip in pips:
        # Filter out small contours that might be noise
        if (cv2.contourArea(pip) > 12) and (cv2.contourArea(pip)<90):  # For the outside of the pips...
            # print(cv2.contourArea(pip))
            num_pips+=1
            x, y, w, h = cv2.boundingRect(pip)
            # cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 6)
            cv2.rectangle(img, (orig_x+x, orig_y+y), (orig_x+x + w, orig_y+y + h), (0, 0, 0), 2)
    text = "Die "+ str(die_num)+ " has " + str(num_pips)+" pips"
    print(text)
    return num_pips

def read_dice_pixel_coords():
    """reads die pixel coordinates from text file
    
    Note: split at x=610 line. 
            x>610 is for theo
            x<=610 is for dj

    Returns:
        list: list of lists (coordinates of the form [x,y,w,h])
    """
    coords = []
    with open("img_die_loc.txt", "r") as file:
        next(file)  # skip the header line ("coordinates")
        for line in file:
            # strip removes newline, split by comma, convert each to float (or int)
            parts = [float(x) for x in line.strip().split(",")]
            coords.append(parts)
    return coords
