#!/usr/bin/env python
import rospy
import time
import serial
from kill_handling.listener import KillListener
from kill_handling.broadcaster import KillBroadcaster
from std_msgs.msg import Bool
try:
    rospy.init_node('kill_switch') 
    kill_broadcaster = KillBroadcaster(id=rospy.get_name(), description='kill button kill')
    begin = rospy.Publisher("begin", Bool, queue_size = 1) 
    last_pressed = ''
    killed = True
    # configure the serial connections (the parameters differs on the device you are connecting to)
    ser = serial.Serial(
        port='/dev/serial/by-id/usb-Black_Sphere_Technologies_CDC-ACM_Demo_DEMO-if00',
        baudrate=9600,
        #parity=serial.PARITY_ODD,
        #stopbits=serial.STOPBITS_TWO,
        #bytesize=serial.SEVENBITS
    )

    ser.close()
    ser.open()
    ser.isOpen()
    kill_broadcaster.send(killed)
    ser.write('B')
    print 'Enter your commands below.\r\nInsert "exit" to leave the application.'

    input=1
    while not rospy.is_shutdown() :
            # send the character to the device
            # (note that I happend a \r\n carriage return and line feed to the characters - this is requested by my device)
        ser.write('?')
        out = ''
        # let's wait one second before reading output (let's give device time to answer)
        while ser.inWaiting() > 0:
            out += ser.read(1)
        if out == str(4) and str(out) != last_pressed:
            print last_pressed
            if killed == False:
                ser.write('B')
                killed = True
            elif killed == True:
                ser.write('b')
                killed = False
            kill_broadcaster.send(killed)

        if out == str(8) and str(out) != last_pressed:
            print last_pressed
            begin.publish(True)

        last_pressed = str(out)
        time.sleep(.5)
finally:
    ser.write('b')

