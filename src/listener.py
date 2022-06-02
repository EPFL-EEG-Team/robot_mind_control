'''
    @file: listener.py

    @date: 12/05/2022

    @author: Emile Janho Dit Hreich
             emile.janhodithreich@epfl.ch

    @brief:


'''
# ==========================================================
# Libraries

import serial           # Pyserial library
import rospy            # ROS Noetic
import time

from std_msgs.msg import Float32MultiArray, Float32, Int32

# ==========================================================
# Some constants

COM_PORT   = "COM9"
BAUDRATE   = 115200
ENCODING   = "ascii"
PARSE_CHAR = "_"
# ==========================================================
# Functions

def connect():
    '''
        Establishes connection between device and ESP32 Board
    '''
    while (True):
        # will try to listen on specified port (COM_PORT)
        try:
            listener = serial.Serial(COM_PORT, BAUDRATE, timeout = 1)
            print("Connection established")
            return listener
        except:
            print("Unable to connect to remote device. Retrying...")
            time.sleep(5)
            
def process_EEG(array):
    EEG_publisher.publish(Float32MultiArray(data=array))

def process_EMG(data):
    
    EMG_publisher.publish(data)

def process_IMU(data):
    IMU_publisher.publish(data)



EEG_data = []    

def retrieve_data(listener):
    '''
        Listens to the ESP32 board on the headgear and call functions
        to prepare its processing

    '''
    # initializing the array that will contain EEG data
    global EEG_data  


    while (True):

        # Parse the string of fromat "<Type of data>_<value>"
        if (listener.in_waiting > 0):

            data = listener.readline().decode(ENCODING)
            # print(data)
            (value, msg_type) = data.split(PARSE_CHAR)
            
            
            
            if (msg_type == "EEG\n"):

                # convert string to float
                value = float(value)
                EEG_data.append(value)
                # print(EEG_data)
                if len(EEG_data) == 99:
                    # print("sent")
                    process_EEG(EEG_data)
                    # time.sleep(0.2)
                    listener.flushInput()
                    EEG_data = []


            elif (msg_type == "EMG\n"):
                # convert string to float
                print(value, msg_type)

                value = float(value)
                process_EMG(value)
                

            elif (msg_type == "IMU\n"):
                # convert string to float
                print(value, msg_type)
            
                value = float(value)
                process_IMU(value)





# ==========================================================
# Main

if __name__ == "__main__":

    # ROS node initialzation
    rospy.init_node("Listener", anonymous=False)

    # ROS publishers
    EEG_publisher = rospy.Publisher("EEG", Float32MultiArray, queue_size=1)
    EMG_publisher = rospy.Publisher("EMG", Float32, queue_size=1)
    IMU_publisher = rospy.Publisher("IMU", Float32, queue_size=1)

    # ROS Subscribers
    # rospy.Subscriber("EN", Int32, reenable_callback)

    # Establish connection
    listener = connect()
    
    retrieve_data(listener)
