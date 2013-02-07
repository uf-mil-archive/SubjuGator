import StringIO
import socket
import struct

def crc16(data, prev=0, mask=0x1021):
    crc = prev
    for char in data:
        c = ord(char)
        c = c << 8
        
        for j in xrange(8):
            if (crc ^ c) & 0x8000:
                crc = ((crc << 1) ^ mask) & 0xffff
            else:
                crc = (crc << 1) & 0xffff
            c = c << 1
    return crc

class Embedded(object):
    def __init__(self, host, port, local_address, remote_address):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind(('', 50000))
        self.s.connect((host, port))
        self.local_address = local_address
        self.remote_address = remote_address
        self.packet_count = 0
    
    def recv(self):
        while True:
            try:
                f = StringIO.StringIO(self.s.recv(2**16))
            except:
                import traceback, time
                traceback.print_exc()
                time.sleep(1)
                continue
            
            if f.read(1) != '\x7e':
                print 'packet did not start with 7E'
                continue
            
            data = []
            while True:
                b = f.read(1)
                if b == '\x7e':
                    break
                elif b == '\x7d':
                    data.append(chr(ord(f.read(1)) ^ 0x20))
                else:
                    data.append(b)
            if f.read():
                print 'extra data'
            data2 = ''.join(data)
            
            if len(data2) < 2 or data2[-2:] != struct.pack('<H', crc16(data2[:-2])):
                print 'embedded invalid', data.encode('hex')
                continue
            data3 = data2[:-2]
            
            if len(data3) < 4:
                print 'too short'
                continue
            
            (src_address, dest_address, packetcount), data4 = struct.unpack('BBH', data3[:4]), data3[4:]
            
            if dest_address != self.local_address or src_address != self.remote_address:
                print 'embedded address mismatch', (dest_address, src_address), (self.local_address, self.remote_address)
                continue
            
            return data4
    
    def send(self, contents):
        data_pre = struct.pack('BBH', self.local_address, self.remote_address, self.packet_count % 2**16) + contents
        data = data_pre + struct.pack('H', crc16(data_pre))
        
        res = []
        res.append('\x7e')
        for char in data:
            if char == '\x7d' or char == '\x7e':
                res.append('\x7d')
                res.append(chr(ord(char) ^ 0x20))
            else:
                res.append(char)
        res.append('\x7e')
        try:
            self.s.send(''.join(res))
        except:
            import traceback
            traceback.print_exc()
            return
        
        self.packet_count += 1
