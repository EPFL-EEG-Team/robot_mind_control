'''
    @file: encoder.py

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

COM_PORT   = "COM11"
BAUDRATE   = 115200

# ==========================================================