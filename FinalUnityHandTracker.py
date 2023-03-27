#!/usr/bin/env python3

import cv2
import depthai as dai
import mediapipe as mp
import socket
import ast
import numpy as np

# Getting all the socket stuff set up
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
serverAddressPort = ("127.0.0.1", 5052)

# Making all the functions for getting a frame
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

xoutRgb.setStreamName("rgb")
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

# Linking
camRgb.preview.link(xoutRgb.input)
# Creating the stereo camera
stereo.disparity.link(xoutDisp.input)

#stereo.rectifiedLeft.link(xoutRectifiedLeft.input)
#stereo.rectifiedRight.link(xoutRectifiedRight.input)

# Creating a distance variable
distance = 0

# Making the landmarks list
landmarks = ""
# Connect to device and start pipeline
with dai.Device(pipeline) as device:
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
    #rectifiedLeftQueue = device.getOutputQueue(name="rectifiedLeft", maxSize=1, blocking=False)
    #rectifiedRightQueue = device.getOutputQueue(name="rectifiedRight", maxSize=1, blocking=False)
    disparityMultiplier = 255 / stereo.getMaxDisparity()

    # cv2.namedWindow("Stereo Pair")
    # cv2.setMouseCallback("Stereo Pair", mouseCallback)

    # Variable use to toggle between side by side view and one frame view.
    sideBySide = False

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
        a = 89.3322
        h = 0.0000117139
        p = -0.00288686
        b = 3.6007 * (10 ** 16)
        i = -62.381
        q = -7.49748
        multiplied_list = [(a * ((item - h) ** (item * p)) + b * ((item - i) ** q) + 20) for item in disparity]
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
        with mp_hands.Hands(
                model_complexity=0,
                min_detection_confidence=0.3,
                min_tracking_confidence=0.3) as hands:
            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image = inRgb.getCvFrame()
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = hands.process(image)

            # Draw the hand annotations on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            #Create list for coordinates of all the landmarks
            #landmarks = ""
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    landmarks = (((((str(hand_landmarks).replace('landmark ', '')).replace('{\n  x: ', '')).
                                   replace('\n  y: ', ',')).replace('\n  z: ', ','))).replace('\n}\n', ',')
                    landmarks = list(ast.literal_eval(landmarks))
                    '''
                    mp_drawing.draw_landmarks(
                        image,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style())
                    '''
                    mp_drawing.draw_landmarks(
                        disparity,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style())

                    #Hand model correction for finding depth
                    #print(str(landmarks[0]*640) + ',' + str(landmarks[1]*400))
                    wristx = (landmarks[0] * 640)/(1+((landmarks[0])/3))
                    wristy = (landmarks[1] * 400) - 40
                    disparity = cv2.circle(disparity, (int(wristx),int(wristy)), radius=10, color=(0, 0, 255), thickness=-1)
                    #Adding the position of the hand
                    # A more specified distance and radius for the hand to get the distance from
                    # print(min(multiplied_list))
                    distanceZone = []
                    try:
                        for i in range(-10, 10):
                            for h in range(-10, 10):
                                distanceZone.append(multiplied_list[int(wristy) + i][int(wristx) + h])
                        if min(distanceZone) <= 100:
                            distance = min(distanceZone)
                        #print(distance)
                    except:
                        print("no hand lol")
                    '''
                    distance = min(distanceZone)
                    print(distance)
                    print(multiplied_list[200])
                    '''
                    #landmarks.append(landmarks[0])
                    #landmarks.append(landmarks[1])
                    landmarks.append(distance)
            # Flip the image horizontally for a selfie-view display.
            print(distance)
            cv2.imshow('MediaPipe Hands', image)
            #cv2.imshow("Stereo Pair", imOut)
            cv2.imshow("Disparity", disparity)
            print(landmarks)
            sock.sendto(str.encode(str(landmarks)), serverAddressPort)
        #If a certain key is pressed then break the code
        if cv2.waitKey(1) == ord('q'):
            break