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
    if len(parse) == 3:
        if parse[0] == 'Message:':
            print(instr.strip())

if __name__ == '__main__':
    portname = '/dev/ttyUSB0'
    portbaud = '115200'
    #portbaud = '120000000'
    daq_ser = serial.Serial(portname,portbaud,timeout=0.0)
    if daq_ser.status == "OK":
        print("opened port " + portname + " for device outdoor_rcv")
    else:
        print("Error opening port " + portname + " for device outdoor_rcv")
        exit(0)

    then = time.time()
    while(True):
        try:
            instr = daq_ser.ser.readline()
        except serial.serialutil.SerialException as e: 
            print("error reading serial data")
            print(e)
            instr = ""
            time.sleep(10)

        if len(instr) > 0:
            print(instr.strip())
        parse_message(instr)
 
        now = time.time()
        if (now - then) > 2:
            then = now
            print("interval time: sending")
 
            try:
                daq_ser.ser.write("disp: 1\n")
                daq_ser.ser.write("rgb: 0  0 00\n")
            except Exception as e:
                print e
