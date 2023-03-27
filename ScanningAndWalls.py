#!/usr/bin/env python3

import cv2
import depthai as dai
import mediapipe as mp
import socket
import ast
import numpy as np
from statistics import mean

def getFrame(queue):
    # Get frame from queue
    frame = queue.get()
    # Convert frame to OpenCV format and return
    return frame.getCvFrame()

def getMonoCamera(pipeline, isLeft):
    # Configure mono camera
    mono = pipeline.createMonoCamera()

    # Set Camera Resolution
    mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    #mono.setPreviewSize(800,400)
    if isLeft:
        # Get left camera
        mono.setBoardSocket(dai.CameraBoardSocket.LEFT)
    else:
        # Get right camera
        mono.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    return mono


def getStereoPair(pipeline, monoLeft, monoRight):
    # Configure stereo pair for depth estimation
    stereo = pipeline.createStereoDepth()
    # Checks occluded pixels and marks them as invalid
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(True)
    stereo.setSubpixel(False)

    # Configure left and right cameras to work as a stereo pair
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)

    return stereo

# Getting all the socket stuff set up
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
serverAddressPort = ("127.0.0.1", 5052)
#Creating the sockets for IMU stuff
#IMUsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#IMUserverAddressPort = ("127.0.0.1", 5053)

#Creating all of the hand tracking stuff
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

# Create pipeline
pipeline = dai.Pipeline()
# Set up left and right cameras
monoLeft = getMonoCamera(pipeline, isLeft=True)
monoRight = getMonoCamera(pipeline, isLeft=False)
# Combine left and right cameras to form a stereo pair
stereo = getStereoPair(pipeline, monoLeft, monoRight)
# Define source and output
camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setFps(60)
xoutRgb = pipeline.create(dai.node.XLinkOut)
imu = pipeline.create(dai.node.IMU)
xlinkOut = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")
xlinkOut.setStreamName("imu")
# Set XlinkOut for disparity, rectifiedLeft, RGB, and rectifiedRight
xoutDisp = pipeline.createXLinkOut()
xoutDisp.setStreamName("disparity")

#xoutRectifiedLeft = pipeline.createXLinkOut()
#xoutRectifiedLeft.setStreamName("rectifiedLeft")

#xoutRectifiedRight = pipeline.createXLinkOut()
#xoutRectifiedRight.setStreamName("rectifiedRight")

# Properties
camRgb.setPreviewSize(640, 400)
camRgb.setInterleaved(False)
#camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

# enable ACCELEROMETER_RAW at 100 hz rate
imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500)
# enable GYROSCOPE_RAW at 100 hz rate
imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 500)
# it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
# above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
imu.setBatchReportThreshold(10)
# maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
# if lower or equal to batchReportThreshold then the sending is always blocking on device
# useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
imu.setMaxBatchReports(10)


# Linking
camRgb.preview.link(xoutRgb.input)
# Creating the stereo camera
stereo.disparity.link(xoutDisp.input)
#Linking the IMU link
# Link plugins IMU -> XLINK
imu.out.link(xlinkOut.input)

#stereo.rectifiedLeft.link(xoutRectifiedLeft.input)
#stereo.rectifiedRight.link(xoutRectifiedRight.input)


# Creating a distance variable
distance = 0
# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    def timeDeltaToMilliS(delta) -> float:
        return delta.total_seconds()*1000
    print('Connected cameras:', device.getConnectedCameraFeatures())
    # Print out usb speed
    print('Usb speed:', device.getUsbSpeed().name)
    # Bootloader version
    if device.getBootloaderVersion() is not None:
        print('Bootloader version:', device.getBootloaderVersion())
    # Device name
    print('Device name:', device.getDeviceName())

    # Output queue will be used to get the rgb frames from the output defined above
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    disparityQueue = device.getOutputQueue(name="disparity", maxSize=1, blocking=False)
    imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
    #rectifiedLeftQueue = device.getOutputQueue(name="rectifiedLeft", maxSize=1, blocking=False)
    #rectifiedRightQueue = device.getOutputQueue(name="rectifiedRight", maxSize=1, blocking=False)
    disparityMultiplier = 255 / stereo.getMaxDisparity()

    # cv2.namedWindow("Stereo Pair")
    # cv2.setMouseCallback("Stereo Pair", mouseCallback)

    # Variable use to toggle between side by side view and one frame view.
    sideBySide = False
    baseTs = None
    # Creating the variables and lists for the correction for the gyroscope rad/s
    xcorr = 0
    ycorr = 0
    zcorr = 0
    prevgyroTs = 0
    #newXvalues = 0
    #newYvalues = 0
    #newZvalues = 0
    while True:
        inRgb = qRgb.get()  # blocking call, will wait until a new data has arrived

        # Retrieve 'bgr' (opencv format) frame
        #cv2.imshow("rgb", inRgb.getCvFrame())

        #Doing all of the hand stuff
        # Get disparity map
        disparity = getFrame(disparityQueue)

        # Colormap disparity for display
        disparity = (disparity * disparityMultiplier).astype(np.uint8)
        # Printing the array of the disparity and specific values I'm interested in
        #multiplied_list = [item for item in disparity]
        a = 89.3322
        h = 0.0000117139
        p = -0.00288686
        b = 3.6007*(10**16)
        i = -62.381
        q = -7.49748
        multiplied_list = [(a*((item-h)**(item*p))+b*((item-i)**q)+20) for item in disparity]
        disparity = cv2.applyColorMap(disparity, cv2.COLORMAP_JET)

        # We don't need the framebyframe right now
        '''
        # Get left and right rectified frame
        leftFrame = getFrame(rectifiedLeftQueue)
        rightFrame = getFrame(rectifiedRightQueue)
        if sideBySide:
            # Show side by side view
            imOut = np.hstack((leftFrame, rightFrame))
        else:
            # Show overlapping frames
            imOut = np.uint8(leftFrame / 2 + rightFrame / 2)

        imOut = cv2.cvtColor(imOut, cv2.COLOR_GRAY2RGB)
        '''
        imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived
        cameraIMUdata = []
        imuPackets = imuData.packets
        distanceZone = []
        '''
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image = inRgb.getCvFrame()
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image)

        # Draw the hand annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        '''


        # Flip the image horizontally for a selfie-view display.
        for i in range(-5,5):
            for h in range(-5,5):
                distanceZone.append(multiplied_list[200+i][320+h])
        #print(distanceZone)
        #disparity = cv2.circle(disparity, (300, 200), radius=10, color=(0, 0, 255), thickness=2)
        #cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
        #cv2.imshow("Stereo Pair", imOut)
        #cv2.imshow("Disparity", disparity)
        #print(landmarks)
        #Sending Unity the distance that the code thinks it is from the wall
        #sock.sendto(str.encode(str(min(distanceZone))), serverAddressPort)
        for imuPacket in imuPackets:
            acceleroValues = imuPacket.acceleroMeter
            gyroValues = imuPacket.gyroscope
            acceleroTs = acceleroValues.getTimestampDevice()
            gyroTs = gyroValues.getTimestampDevice()
            if baseTs is None:
                baseTs = acceleroTs if acceleroTs < gyroTs else gyroTs
            acceleroTs = timeDeltaToMilliS(acceleroTs - baseTs)
            gyroTs = timeDeltaToMilliS(gyroTs - baseTs)
            ''''''
            newXvalues = gyroValues.x- 0.0016884374625499225
            newYvalues = gyroValues.y- 0.011683018374207613
            newZvalues = gyroValues.z- 0.0017421931295073833
            '''
            if gyroValues.y < 0:
                newYvalues = gyroValues.y/1.024447
            else:
                newYvalues = gyroValues.y/1.0229874

            if gyroValues.x > 0:
                newXvalues = gyroValues.x/0.99985
            else:
                newXvalues = gyroValues.x/0.99954404

            if gyroValues.z > 0:
                newZvalues = gyroValues.z/1.00995
            else:
                newZvalues = gyroValues.z/0.999901
            cameraIMUdata = [gyroValues.x, gyroValues.y, gyroValues.z, acceleroValues.x, acceleroValues.y,
                             acceleroValues.z]
            '''
            '''
            cameraIMUdata = [newXvalues, newYvalues, newZvalues, acceleroValues.x + 0.25860592617865413,
                             acceleroValues.y + 0.1708861339984958,acceleroValues.z - 9.79588319982978]
            '''
            cameraIMUdata = [newXvalues, newYvalues, newZvalues, acceleroValues.x,
                             acceleroValues.y, acceleroValues.z]
            '''
            cameraIMUdata = [gyroValues.x-((gyroTs-prevgyroTs)*0.000503377218544), gyroValues.y-((gyroTs-prevgyroTs)*0.00217858414959), gyroValues.z-((gyroTs-prevgyroTs)*0.000215769665467)]
            '''
            #print(cameraIMUdata)
            #IMUsock.sendto(str.encode(str(cameraIMUdata)), IMUserverAddressPort)
            #For every item in the list, find whether or not it's an x y or z, and then add that to the total the corresponding coordinate
            xcorr = cameraIMUdata[3]
            ycorr = cameraIMUdata[4]
            zcorr = cameraIMUdata[5]
            #tsF = "{:.03f}"
            print(f"Gyroscope timestamp: {(gyroTs-prevgyroTs)} ms")
            #print(f"Gyroscope timestamp: {tsF.format(gyroTs)} ms")
            print('xcorr: ' + str(xcorr) + '\nycorr: ' + str(ycorr) + '\nzcorr: ' + str(zcorr))
            #print('gyrox: ' + str(cameraIMUdata[0]) + 'gyroy: ' + str(cameraIMUdata[1]) + 'gyroz: ' + str(cameraIMUdata[2]))
            prevgyroTs = gyroTs
        cameraIMUdata.append(min(distanceZone))
        sock.sendto(str.encode(str(cameraIMUdata)), serverAddressPort)
        print(cameraIMUdata)
        #If a certain key is pressed then break the code
        if cv2.waitKey(1) == ord('q'):
            break