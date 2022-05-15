import serial 

ser = serial.Serial("/dev/rfcomm1", 115200, timeout = 1)

def retrieveData():
    # ser.write(b'1')
    data = ser.readline().decode("ascii")
    return data

while (True):
    uinput = input("retrieve data")
    if uinput == '1' :
        print(retrieveData())
    else:
        ser.write(b'0')