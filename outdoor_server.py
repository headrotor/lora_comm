# coding=utf-8
"""
----------------------------------------------------------------------
outfoor_server.py: read serial stream from Heltec esp32 LoRa board,
parse data, and do stuff with it. 
----------------------------------------------------------------------
"""
#----------------------------------------------------------------------

import sys
import time 
import traceback
import serial
# local serial wrapper class
#sys.path.append('../common')
import serial

def parse_message(thestr):
    if len(instr) == 0:
        return
    parse = instr.split()
    #print(str(parse))
    if len(parse) == 3:
        if parse[0] == 'Message:':
            #print(instr.strip())
            print("{}, {}, {}".format(time.time(), parse[1], parse[2]))

            
if __name__ == '__main__':
    portname = '/dev/ttyUSB1'
    portbaud = '115200'
    #portbaud = '120000000'
    try:
        ser = serial.Serial(portname,portbaud,timeout=0.1)
    except serial.serialutil.SerialException as e:
        print("Error opening port " + portname + " for device outdoor_rcv")
        raise e
        exit(0)

    then = time.time()
    while(True):
        try:
            bstr = ser.readline()
        except serial.serialutil.SerialException as e: 
            print("error reading serial data")
            print(e)
            bstr = b"oops"
            time.sleep(10)

        instr = bstr.decode('utf-8')
        if len(instr) > 0:
            parse_message(instr)
 
        now = time.time()

        #if (now - then) > 2:
        if False:
            then = now
            print("interval time: sending")
 
            try:
                ser.write(bytes("disp: 1\n", 'utf-8'))
                #ser.write("rgb: 0  0 0 0\n")
            except Exception as e:
                print(e)
