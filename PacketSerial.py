import numpy as np
from serial import Serial
import time
from time import ctime

HEAD_BYTE = 0x7E
ESCAPE_BYTE = 0x7D
ESCAPE_MASK = 0x20

class MYSERIAL(Serial):

    def __init__(self, Port, baud = 115200):
        self.myserial = super().__init__(Port, baudrate = baud, timeout = 1)
        self.Registar = [0]*128

    def sendFloatData(self, data, reg):
        decimal, integer = modf(data)
        self.send(int(integer), reg)
        self.send(int(decimal*1000000), reg+1)

    def getFloatData(self, reg):
        return self.Registar[reg] + (self.Registar[reg+1] / 1000000)

    def writeData(self, data):
        tmp = data
        #print("tmp=",tmp)
        if tmp == 0:
            w = tmp.to_bytes(1, byteorder='big')
        else:
            w = tmp.to_bytes((tmp.bit_length() + 7) // 8, byteorder='big')
        #Serial.write(self, bytes(chr(data), 'utf-8'))
        Serial.write(self, w)
        #time.sleep(0.010)

    def readData(self):
        r=0
        r = Serial.read(self)
        d = int.from_bytes(r, 'little')
        return d

    def send(self, data, reg):
        dataBytes =[
            (data >> 24) & 0xFF,
            (data >> 16) & 0xFF,
            (data >>  8) & 0xFF,
            (data >>  0) & 0xFF
        ]
        self.writeData(HEAD_BYTE)
        checksum = 0
        self.writeData(reg)
        for i in range(4):
            if ((dataBytes[i] == ESCAPE_BYTE) or (dataBytes[i] == HEAD_BYTE)):
                self.writeData(ESCAPE_BYTE)
                checksum += ESCAPE_BYTE
                self.writeData(dataBytes[i] ^ ESCAPE_MASK)
                checksum += dataBytes[i] ^ ESCAPE_MASK
            else:
                self.writeData(dataBytes[i])
                checksum += dataBytes[i]

        self.writeData(checksum&0xFF)

        time.sleep(0.001)

    def receive(self):
        h_data = self.readData()
        d_bytes=[0,0,0,0]
        checksum = 0
        if (h_data == HEAD_BYTE):
            reg = self.readData()
            for i in range(4):
                d = self.readData()
                if (d == ESCAPE_BYTE):
                    nextByte = self.readData()
                    d_bytes[i] = nextByte ^ ESCAPE_MASK
                    checksum += d + nextByte
                else:
                    d_bytes[i] = d
                    checksum += d
            data = 0
            for i in range(4):
                data |= ((d_bytes[i]&0xFF) << (24 - (i*8)))

            checksum_recv = self.readData()
            if ((checksum & 0xFF) == checksum_recv):
                self.Registar[reg] = data

            else:
                print("data error", checksum, checksum_recv)

def main():
    device = MYSERIAL("COM10")
    i = 0
    inc = 1
    while True:
        print(i)
        device.send(i, 0x10)
        device.send(i, 0x20)
        device.send(i, 0x30)
        device.send(i, 0x40)
        i+=inc
        if i > 0xFF:
            inc = -1
        if i < 0:
            inc = 1

if __name__ == "__main__":
    main()