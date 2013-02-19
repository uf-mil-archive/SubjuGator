import struct
import time

import serial

class XBee(object):
    def __init__(self, port):
        self.s = serial.Serial(port, 9600)
        self.s.flushInput()

        time.sleep(1.1)
        self.s.write('+++')
        time.sleep(1.1)
        print repr(self.s.read(self.s.inWaiting()))
        self.s.write('ATAP2\r')
        time.sleep(.1)
        print repr(self.s.read(self.s.inWaiting()))
        self.s.write('ATAC\r')
        time.sleep(.1)
        print repr(self.s.read(self.s.inWaiting()))
    
    def _read_unescaped(self, length):
        res = ''
        while len(res) < length:
            c = self.s.read(1)
            if c == '\x7d':
                x = self.s.read(1)
                res += chr(ord(x) ^ 0x20)
            else:
                res += c
        return res
    
    def read(self):
        while True:
            x = self.s.read(1)
            if x != '\x7e':
                print 'garbage', x.encode('hex')
                continue
            
            length, = struct.unpack('>H', self._read_unescaped(2))
            if length == 0:
                print 'length == 0'
                continue
            if length > 270:
                print 'length too long', length
                continue
            
            data = self._read_unescaped(length)
            
            checksum_should = 0xFF - sum(map(ord, data)) % 256
            checksum, = struct.unpack('>B', self._read_unescaped(1))
            if checksum != checksum_should:
                print 'invalid checksum!'
                continue
            
            cmdID = ord(data[0])
            cmdData = data[1:]
            
            return cmdID, cmdData
