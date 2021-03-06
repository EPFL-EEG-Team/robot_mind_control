'''
    @file: encoder.py

    @date: 12/05/2022

    @author: Emile Janho Dit Hreich
             emile.janhodithreich@epfl.ch

             Marin Vogelsang
             marin.vogelsang@epfl.ch

    @brief: This file contains all processing functions for
            EEG, EMG and IMU data. It is subscribed to the 
            listener (listener.py file). Upon reception of
            data, it will process and encode corresponding 
            controls for the car. The commands will be sent
            to the car via Bluetooth serial.

'''
# ==========================================================
# Libraries

import serial           # Pyserial library
import rospy            # ROS Noetic
import time
import numpy as np
import struct

from sklearn import svm
from std_msgs.msg import Float32MultiArray, Float32, Int32

# from listener import EEG_data
# ==========================================================
# setup

# ------------------------------------------
# Bluetooth
COM_PORT   = "COM12"
BAUDRATE   = 9600

# ------------------------------------------
# EEG
list_types = ['Relax', 'Focus']
NUM_TRIALS = 3
NUM_ITER_PER_TYPE = 40  #TODO put back to 60

baseline_frames = NUM_TRIALS * len(list_types) * NUM_ITER_PER_TYPE

#Speed change delta
delta_speed = 5

# Fast-Fourier Transform
length  = 99
fftfreq = np.fft.fftfreq(length, d = 2 / 1000)[1:10] # data interval is 2ms

num_samples = 10                                     # number of frames for mean
fft_samples = np.zeros((len(fftfreq), num_samples))

#svm instance
svm_classifier = svm.SVC()
X = []
y = []

#Speed array
speed_array = []

# ------------------------------------------
# EMG
EMG_BASELINE_ITERATION = 50
EMG_THRESHOLD          = 20      # Difference between basline and recorded value

EMG_baseline_acquisition_table = []
EMG_baseline = -1

EMG_FLAG = 1

# ------------------------------------------
# IMU
IMU_THRESHOLD_R = 0
IMU_THRESHOLD_L = 0

# ==========================================================
# Functions

def connect():
    '''
        Establishes connection between device and ESP32 Board
    '''
    while (True):
        try:
            controller = serial.Serial(COM_PORT, BAUDRATE, timeout = 1)
            print("Connection established")
            return controller
        except:
            print("Unable to connect to remote device. Retrying...")
            time.sleep(5)

def baseline_acquisition(i, data):
    '''Documentation missing
    '''
    mode = (i // NUM_ITER_PER_TYPE) % len(list_types)
    print("Base line acquisition: " + list_types[mode], end="\r")
    
    # fft calculation
    fft_samples[:, i%num_samples] = np.log(np.abs(np.fft.fft(data))**2)[1:10]

    # Only use latter half of the data
    if (i % NUM_ITER_PER_TYPE) >= NUM_ITER_PER_TYPE // 2 :
        mean = np.mean(fft_samples, axis=1)
        X.append(mean)
        y.append(mode)
    
    speed_array.append(0)

def get_speed(data):
    '''
        data: Array[float] contains data aquired over a period of 2 ms.

        EEG processing function.Trains the SVM, performs calibration with respect to a 
        baseline then starts computing the speed of the car. The baseline is acquired 
        with baseline_acquisition() function.

    '''
    print(data.shape)
    assert data.shape==(length,), "data size does not match"
    
    i = len(speed_array)
    
    # When the base line data acquisition is done
    # Train svm
    if i == baseline_frames:
        svm_classifier.fit(X, y)
        print("Base line done!", end="\r")
    
    # At first ground truth data collection
    if i < baseline_frames:
        baseline_acquisition(i, data)
    else:    
        #fft calculation
        fft_samples[:, i % num_samples] = np.log(np.abs(np.fft.fft(data))**2)[1:10]
        mean = np.mean(fft_samples, axis=1)

        #Obtain the prediction and update speed
        pred = svm_classifier.predict([mean])[0]
        if pred == 0:
            speed = speed_array[-1] - delta_speed
        else:
            speed = speed_array[-1] + delta_speed

        #Limit the range of the speed (0 to 255 inclusive)
        if speed > 255:
            speed = 255
        elif speed < 0:
            speed = 40

        speed_array.append(speed)
        print("Current state: " + list_types[pred] + ", Speed: " + str(speed), end= "\r")

    # print(speed_array[-1])
    return speed_array[-1]



def get_orientation(data):
    '''
        EMG processing function
    '''
    global EMG_baseline
    global EMG_FLAG
    # baseline acquisition
    if (len(EMG_baseline_acquisition_table) < EMG_BASELINE_ITERATION):
        EMG_baseline_acquisition_table.append(data)
    else:

        if EMG_baseline == -1:
            EMG_baseline = np.mean(np.array(EMG_baseline_acquisition_table))

        # encoding
        if (data - EMG_baseline > EMG_THRESHOLD and EMG_FLAG == 1):
            data = "EMG_00000000\n"
            controller.reset_input_buffer()
            controller.write(bytes(data, 'ascii'))
            controller.flush()
            EMG_FLAG = 0
        elif(data == 0):
            EMG_FLAG = 1

    

def process_EEG(msg):
    '''
        callback function for EEG topic
    '''

    # global EEG_data

    values = np.array(msg.data)

    speed  = get_speed(values)
    # ===========================
    # debug
    # print(speed)
    # ===========================
    
    data = "EEG_" + str(int(speed)).zfill(8) + "\n"
    controller.reset_output_buffer()
    controller.write(bytes(data, 'ascii'))
    controller.flush()


def process_EMG(msg):
    '''
        callback function for EMG topic
    '''
    get_orientation(msg.data)

def process_IMU(msg):
    '''
        callback function for IMU topic
    '''
    # directly send to the car. No prior processing is necessary
    # because the conversion to roll-pitch-yaw happens on the previous layer
    # we could do it here as well. 

    # print(str(int(msg.data)).zfill(8))
    data = "IMU_" + str(int(msg.data)).zfill(8) + "\n"

    controller.reset_output_buffer()
    controller.write(bytes(data, 'ascii'))
    controller.flush()




# ==========================================================
# Main

if __name__ == "__main__":

    # ROS node initialzation
    rospy.init_node("Processor", anonymous=False)

    # ROS Subscribers
    rospy.Subscriber("EEG", Float32MultiArray, process_EEG)
    rospy.Subscriber("EMG", Float32, process_EMG)
    rospy.Subscriber("IMU", Float32, process_IMU)

    # ROS publishers
    reenable_pub = rospy.Publisher("EN", Int32,  queue_size=1)

    # connect to remote device
    controller = connect()

    # ROS loop
    rospy.spin()

