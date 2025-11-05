#coding=utf-8
import cv2
import numpy as np
import mvsdk
import platform
# from standardbots import StandardBotsRobot
# sdk = StandardBotsRobot(
#     url="https://mybot.sb.app",
#     token="token",
#     robot_kind=StandardBotsRobot.RobotKind.Live
# )
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
def main_loop():
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
    mvsdk.CameraSetExposureTime(hCamera,80 * 1000)

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

    while (cv2.waitKey(1) & 0xFF) != ord('q'):
        # Capture one frame from the camera
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

            # On Windows, the image data obtained is vertically flipped and stored as BMP format.
            # When converting to OpenCV format, you must flip it vertically.
            # On Linux, the image is already correct — no flipping needed.
            if platform.system() == "Windows":
                mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)
            
            # At this point, the image is stored in pFrameBuffer.
            # For color cameras: pFrameBuffer = RGB data.
            # For monochrome cameras: pFrameBuffer = 8-bit grayscale data.
            # Convert pFrameBuffer into an OpenCV image for further processing.
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))
            cv2.imwrite("frame.jpg", frame)
            frame = cv2.resize(frame, (3840, 2160), interpolation=cv2.INTER_LINEAR)
            find_dice(frame)
            cv2.imshow("Press q to end", frame)
            
        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message))

    # Close the camera
    mvsdk.CameraUnInit(hCamera)

    # Free the frame buffer
    mvsdk.CameraAlignFree(pFrameBuffer)

def main():
    try:
        main_loop()
    finally:
        cv2.destroyAllWindows()

main()
