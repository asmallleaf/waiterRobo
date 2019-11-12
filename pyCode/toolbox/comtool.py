from .basetool import BaseTool
import serial

class ComTool(BaseTool):
    def _help(self):
        print('This tool is designed for communication in python')
        print('It has included bluetooth communication with serial port(serialBlue)')
        print('use _helpfor to check some specific information')

    def _helpfor(self,fnc_name):
        if fnc_name == 'serialBlue':
            print('It is a serial bluetooth communication tool box')

class Port:
    def __init__(self):
        self.baudrate = None
        self.portId = None
        self.timeout = None
        self.msg = []
        self.msgNum = None
        self.endCh = None

    def clone(self,temp):
        self.baudrate = temp.baudrate
        self.portId = temp.portId
        self.timeout = temp.timeout
        self.msg = temp.msg
        self.msgNum = temp.msg_num
        self.endCh = temp.endCh

    def setBaudrate(self,rate):
        self.baudrate = rate
        return self

    def setPort(self,id):
        self.portId = id
        return self

    def setTimeout(self,time):
        self.timeout = time
        return self

    def getMsg(self,id):
        if self.msg is None:
            return None
        elif self.msgNum >= id:
            return self.msg[id]

    def getMsgNum(self):
        return self.msgNum

    def setEndCh(self,ch):
        self.endCh = ch
        return self

    def addMsg(self,msg):
        self.msg.append(msg)
        self.msgNum = self.msgNum+1

    def delMsg(self,id = -1):
        if id <= self.msgNum and id >= 0:
            del self.msg[id]

class SerialBT:
    @classmethod
    def getMsgOnce(cls,port):
        with serial.Serial(port.portId,port.baudrate,port.timeout) as ser:
            msg = ''
            cbuffer = None
            while cbuffer != port.endCh:
                cbuffer = ser.read()
                msg += cbuffer
            port.msg.append(msg)
            port.msgNum = port.msgNum+1

    @classmethod
    def start(cls,port):
        return serial.Serial(port.portId,port.baudrate,port.timeout)



