#!/usr/bin/python3
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
import socket
from pathlib import Path
import os
# local serial wrapper class
#sys.path.append('../common')
import simplejson

import psutil


from ubidots import ApiClient

mailfile = '/tmp/mail.txt'

def carbon_write(varname, value):
    sock = socket.socket()
    sock.connect( ("localhost", 2003) )
    varname = varname.strip(':')
    msg = "{}.metric {} {}\n".format(varname, value, time.time())
    sock.send(msg.encode('utf-8'))
    sock.close()
    print("sent carbon " + msg)


    
def ubidots_write(varname, value):
    var = None
    try:
        if varname == "temp:":
            var = api.get_variable('5dfb1a7e1d84721740527b94')
        elif varname == "baro:":
            var = api.get_variable('5dfb27f91d84722f098dd73b')             
        elif varname == "humid:":
            var = api.get_variable('5dfbc00c1d847274748affbd')
        elif varname == "mail:":
            print("mail")
            print(str(value))
            if float(value) > 0.5:
                Path(mailfile).touch()                
                print("touched {}".format(mailfile))
            else:
                if os.path.exists(mailfile):
                    print("removing " + mailfile)
                    os.remove(mailfile)
            var = api.get_variable('5dfbc10c1d84727786722701')             
        elif varname == "gasr:":
            var = api.get_variable('5dfbc1431d847277843eb54d')             
        elif varname == "angy:":
            var = api.get_variable('5dfbc17a1d847278d932665e')             
        elif varname == "shake:":
            var = api.get_variable('5dfbc1961d84727933e5b16f')

        elif varname == "door:":
            var = api.get_variable('5dfcf6be1d8472084f0d6e7c')


    except simplejson.scanner.JSONDecodeError:
        return

    if var is not None:
        try:
            val = var.save_value({'value': float(value)})
            #print("sent val")
        except ValueError:
            print("No JSON received, skipping")

    
def parse_message(thestr, logfn, api):
    if len(instr) == 0:
        return

    ndict = {"temp:":0 ,
             "angy:":1,
             "baro:":2 ,
             "mail:":3 ,
             "gasr:":4 ,
             "humid:":5 ,
             "shake:":6,
             "door:":7}

    parse = instr.split()
    #print(str(parse))
    if len(parse) == 3:
        if parse[0] == 'Message:':
            #print(instr.strip())
            print("{}, {}, {}".format(time.strftime('%l:%M%p %Z on %b %d, %Y'), parse[1], parse[2]))
            with open(logfn, "a") as logfile:
                logfile.write("{}, {}, {}\n".format(time.time(),
                                                    ndict[parse[1]],
                                                    parse[2]))

            
            var = None
            # parse[1] is type of message, parse[2] is value
            try:
                if parse[1] == "mail:":
                    if int(parse[2]) > 10:
                        Path(mailfile).touch()                
                        print("touched mail file {}".format(mailfile))
                #ubidots_write(parse[1], parse[2])
            except Exception as e:
                print("caught parse exception, ignoring")
                time.sleep(1)
            try:
                carbon_write(parse[1], parse[2])
            except Exception as e:
                print("caught carbon exception, ignoring")
                # happens when server is down, just ignore
                time.sleep(5)
            sys.stdout.flush()
            
if __name__ == '__main__':
    portname = '/dev/ttyUSB0'
    portbaud = '115200'
    #portbaud = '120000000'

    logfname = "/home/pi/outdoor.log"

    #ubidots
    api = ApiClient(token='BBFF-gjV5Rkr2FXKR4DRCwjDvlBRSjruPKP')

    print("starting")
    
    try:
        ser = serial.Serial(portname,portbaud,timeout=0.1)
    except serial.serialutil.SerialException as e:
        print("Error opening port " + portname + " for device outdoor_rcv")
        raise e
        exit(0)

    then = time.time()
    print("listening")
    while(True):
        try:
            bstr = ser.readline()
        except serial.serialutil.SerialException as e: 
            print("error reading serial data")
            print(e)
            bstr = b"oops"
            time.sleep(10)


        try:
            instr = bstr.decode('utf-8')
        except UnicodeDecodeError:
            instr = ""

        if len(instr) > 0:
            #print(instr)
            parse_message(instr, logfname, api)
            sys.stdout.flush()
            
        now = time.time()

        if (now - then) > 60:
            
            then = now
            print("interval time: sending")
            #zero on first call, should fix that
            cpu_pct = psutil.cpu_percent()
            #cpu_pct = psutil.getloadavg()
            #print("cpu load:  {}".format(cpu_pct))
            #print('memory % used:', psutil.virtual_memory()[2])
            cpu_mem = psutil.virtual_memory()[2]
            print("cpu load:  {}".format(cpu_pct))

            try:
                carbon_write("pidp_load", cpu_pct)
                carbon_write("pidp_mem", cpu_meme)
            except Exception as e:
                print("caught carbon exception, ignoring")
                # happens when server is down, just ignore
                time.sleep(5)


            # try:
            #     ser.write(bytes("disp: 1\n", 'utf-8'))
            #     #ser.write("rgb: 0  0 0 0\n")
            # except Exception as e:
            #     print(e)
