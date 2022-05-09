import serial 
import rospy

# ==========================================================
COM_PORT = "COM11"
BAUDRATE = 115200
# ==========================================================


listener = serial.Serial(COM_PORT, BAUDRATE, timeout = 1)


