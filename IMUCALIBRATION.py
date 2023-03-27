#!/usr/bin/env python3
import decimal

import cv2
import depthai as dai
import time
import math
import socket
import ast
import numpy as np
import csv,datetime
import matplotlib.pyplot as plt

# Create sockets
IMUsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
IMUserverAddressPort = ("127.0.0.1", 5052)

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
imu = pipeline.create(dai.node.IMU)
xlinkOut = pipeline.create(dai.node.XLinkOut)

xlinkOut.setStreamName("imu")

# enable ACCELEROMETER_RAW at 100 hz rate
imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 2000)
# enable GYROSCOPE_RAW at 100 hz rate
imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 2000)
# it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
# above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
imu.setBatchReportThreshold(1)
# maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
# if lower or equal to batchReportThreshold then the sending is always blocking on device
# useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
imu.setMaxBatchReports(1)

# Link plugins IMU -> XLINK
imu.out.link(xlinkOut.input)

# Pipeline is defined, now we can connect to the device
with dai.Device(pipeline) as device:
    def timeDeltaToMilliS(delta) -> float:
        return delta.total_seconds()*1000
    # Output queue for imu bulk packets
    imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
    baseTs = None
    gyroTs = 0
    acceleroTs = 0
    #Creating the variables and lists for the correction for the gyroscope rad/s
    xcorr = 0
    ycorr = 0
    zcorr = 0
    mpu_array = []
    x_array = []
    calibrated_time = 0
    while acceleroTs < 100000:
        imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived
        cameraIMUdata = []
        imuPackets = imuData.packets
        for imuPacket in imuPackets:
            acceleroValues = imuPacket.acceleroMeter
            gyroValues = imuPacket.gyroscope
            acceleroTs = acceleroValues.getTimestampDevice()
            gyroTs = gyroValues.getTimestampDevice()
            if baseTs is None:
                baseTs = acceleroTs if acceleroTs < gyroTs else gyroTs
            acceleroTs = timeDeltaToMilliS(acceleroTs)
            gyroTs = timeDeltaToMilliS(gyroTs)
            cameraIMUdata = [gyroValues.x, gyroValues.y, gyroValues.z, acceleroValues.x, acceleroValues.y, acceleroValues.z]
            #cameraIMUdata = [gyroValues.x, gyroValues.y, gyroValues.z]
            #print(cameraIMUdata)
            #IMUsock.sendto(str.encode(str(cameraIMUdata)), IMUserverAddressPort)
            #For every item in the list, find whether or not it's an x y or z, and then add that to the total the corresponding coordinate
            #xcorr = cameraIMUdata[3]
            #ycorr = cameraIMUdata[4]
            #zcorr = cameraIMUdata[5]
            tsF = "{:.03f}"
            '''
            mpu_array.append([gyroValues.x,gyroValues.y,gyroValues.z])
            print(f"Gyroscope timestamp: {tsF.format(gyroTs)} ms")
            print('xcorr: ' + str(xcorr) + '\nycorr: ' + str(ycorr) + '\nzcorr: ' + str(zcorr))
            print('gyrox: ' + str(cameraIMUdata[0]) + 'gyroy: ' + str(cameraIMUdata[1]) + 'gyroz: ' + str(cameraIMUdata[2]))
            '''
            mpu_array.append(acceleroValues.z)
            x_array.append(acceleroTs)
            calibrated_time += acceleroValues.z
    calibrated_time = [(calibrated_time/len(mpu_array))]*len(mpu_array)
    print(x_array)
    plt.plot(x_array, mpu_array)
    print(calibrated_time)
    plt.plot(x_array,calibrated_time)
    # naming the x axis
    plt.xlabel('x - axis')
    # naming the y axis
    plt.ylabel('y - axis')

    # giving a title to my graph
    plt.title('My first graph!')
    plt.show()