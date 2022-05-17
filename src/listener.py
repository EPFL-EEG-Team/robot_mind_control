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

from std_msgs.msg import Float32MultiArray, Float32

# ==========================================================
# Some constants

COM_PORT   = "/dev/rfcomm2"
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
    print("publish_test")
    EMG_publisher.publish(data)

def process_IMU(data):
    IMU_publisher.publish(data)


def retrieve_data(listener):
    '''
        Listens to the ESP32 board on the headgear and call functions
        to prepare its processing

    '''
    # initializing the array that will contain EEG data
    EEG_data = []  


    while (True):

        # Parse the string of fromat "<Type of data>_<value>"
        if (listener.in_waiting > 0):

            data = listener.readline().decode(ENCODING)
            # print(data)
            (value, msg_type) = data.split(PARSE_CHAR)
            print(msg_type)
            # print(value)
            listener.flush()
            if (msg_type == "EEG\n"):
                print("debug")
                if (value == "BEGIN"):
                    # Resets the array
                    print("EEG_BEGIN")
                    EEG_data = []

                elif (value == "END"):
                    print("EEG_END")
                    process_EEG(EEG_data)

                else:
                    # convert string to float
                    print("debug")
                    value = float(value)
                    EEG_data.append(value)

            elif (msg_type == "EMG\n"):
                # convert string to float
                value = float(value)
                process_EMG(value)

            elif (msg_type == "IMU"):
                # convert string to float
                value = float(value)
                process_IMU(value)




# ==========================================================
# Main

if __name__ == "__main__":

    # ROS node initialzation
    rospy.init_node("Listener", anonymous=False)
    # ROS publishers definition
    EEG_publisher = rospy.Publisher("EEG", Float32MultiArray, queue_size=1)
    EMG_publisher = rospy.Publisher("EMG", Float32, queue_size=1)
    IMU_publisher = rospy.Publisher("IMU", Float32, queue_size=1)


    # Establish connection
    listener = connect()
    retrieve_data(listener)
