#!/usr/bin/env python3

import cv2
import depthai as dai
import mediapipe as mp
import socket
import ast
import numpy as np

#Creating all of the hand tracking stuff
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

# Create pipeline
pipeline = dai.Pipeline()

# Define source and output
camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setFps(30)
xoutRgb = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")

# Properties
camRgb.setPreviewSize(800, 400)
camRgb.setInterleaved(False)
#camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

# Linking
camRgb.preview.link(xoutRgb.input)

def fingerStraight(wristjoint,fingermid,fingertip):
    wristtomid = [fingermid[0]-wristjoint[0],fingermid[1]-wristjoint[1],fingermid[2]-wristjoint[2]]
    midtotip = [fingertip[0]-fingermid[0],fingertip[1]-fingermid[1],fingertip[2]-fingermid[2]]
    if np.degrees(np.arccos(np.dot(wristtomid, midtotip) / ((np.linalg.norm(wristtomid)) * (np.linalg.norm(midtotip))))) < 10:
        #print(np.arccos(np.dot(wristtomid, midtotip) / ((np.linalg.norm(wristtomid)) * (np.linalg.norm(midtotip)))))
        return True
    else:
        #print(np.degrees(np.arccos(np.dot(wristtomid, midtotip) / ((np.linalg.norm(wristtomid)) * (np.linalg.norm(midtotip))))))
        return False

def isHandOpen(landmarklist):
    # Creating the 3d point for the wrist
    wrist = [landmarklist[0], landmarklist[1], landmarklist[2]]

    #Creating the 3d points for the thumb
    thumbbeg = [landmarklist[3],landmarklist[4],landmarklist[5]]
    thumbtip = [landmarklist[12],landmarklist[13],landmarklist[14]]
    thumbstraight = fingerStraight(wrist,thumbbeg,thumbtip)

    #Creating the 3d points for index finger
    indexbeg = [landmarklist[15],landmarklist[16],landmarklist[17]]
    indextip = [landmarklist[24],landmarklist[25],landmarklist[26]]
    indexstraight = fingerStraight(wrist, indexbeg, indextip)

    # Creating the 3d points for middle finger
    middlebeg = [landmarklist[27], landmarklist[28], landmarklist[29]]
    middletip = [landmarklist[36], landmarklist[37], landmarklist[38]]
    middlestraight = fingerStraight(wrist, middlebeg, middletip)

    # Creating the 3d points for ring finger
    ringbeg = [landmarklist[39], landmarklist[40], landmarklist[41]]
    ringtip = [landmarklist[48], landmarklist[49], landmarklist[50]]
    ringstraight = fingerStraight(wrist, ringbeg, ringtip)

    # Creating the 3d points for pinky finger
    pinkybeg = [landmarklist[51], landmarklist[52], landmarklist[53]]
    pinkytip = [landmarklist[60], landmarklist[61], landmarklist[62]]
    pinkystraight = fingerStraight(wrist, pinkybeg, pinkytip)
    '''
    print('thumb: ' + str(thumbstraight))
    print('index: ' + str(indexstraight))
    print('middle: ' + str(middlestraight))
    print('ring: ' + str(ringstraight))
    print('pinky: ' + str(pinkystraight))
    '''
    if indexstraight and middlestraight and ringstraight and pinkystraight:
        return True
    else:
        return False

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

    while True:
        inRgb = qRgb.get()  # blocking call, will wait until a new data has arrived

        # Retrieve 'bgr' (opencv format) frame
        #cv2.imshow("rgb", inRgb.getCvFrame())

        #Doing all of the hand stuff
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
            landmarks = ""
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    landmarks = (((((str(hand_landmarks).replace('landmark ', '')).replace('{\n  x: ', '')).
                                   replace('\n  y: ', ',')).replace('\n  z: ', ','))).replace('\n}\n', ',')
                    landmarks = list(ast.literal_eval(landmarks))
                    mp_drawing.draw_landmarks(
                        image,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style())
            # Flip the image horizontally for a selfie-view display.
            cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
            try:
                print(isHandOpen(landmarks))
            except:
                print('No Hand Here')
            print(landmarks)
        #If a certain key is pressed then break the code
        if cv2.waitKey(1) == ord('q'):
            break