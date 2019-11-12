#from toolbox import comtool
import serial

#port = comtool.Port()
#port.setBaudrate(9600)\
#    .setPort('com6')\
#    .setTimeout(1)
#ser = serial.Serial(port.baudrate,port.portId,port.timeout)
with serial.Serial('com6',9600,1) as ser:
    while(1):
        if(ser.in_waiting>0):
            msg = ser.read(ser.in_waiting)
        print('msg: '+msg)
