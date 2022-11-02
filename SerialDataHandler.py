import serial
import numpy as np

class SerialDataHandler():
    def __init__(self, port, baudrate):
        self.handler = serial.Serial(port, baudrate)

    def send(self, dir):
        self.handler.write(bytes(dir, 'utf-8'))

    def receiveOneInput(self):
        incoming = self.handler.readline()
        converted = str(incoming,'ascii').rstrip()
        data = converted
        return data

    def receiveTwoInputs(self):
        incoming = self.handler.readline()
        converted = str(incoming,'ascii').rstrip()
        data = np.fromstring(converted,dtype = float, sep = ',')
        return data[0], data[1]

    def receiveThreeInputs(self):
        incoming = self.handler.readline()
        converted = str(incoming,'ascii').rstrip()
        data = np.fromstring(converted,dtype = float, sep = ',')
        return data[0], data[1], data[2]

    def receiveFourInputs(self):
        incoming = self.handler.readline()
        converted = str(incoming,'ascii').rstrip()
        data = np.fromstring(converted,dtype = float, sep = ',')
        return data[0], data[1], data[2], data[3] 

