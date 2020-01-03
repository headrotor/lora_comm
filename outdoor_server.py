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
import simplejson


from ubidots import ApiClient



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
            print(instr.strip())
            print("{}, {}, {}".format(time.time(), parse[1], parse[2]))
            with open(logfn, "a") as logfile:
                logfile.write("{}, {}, {}\n".format(time.time(),
                                                    ndict[parse[1]],
                                                    parse[2]))

            
            var = None
            try:
                if parse[1] == "temp:":
                    var = api.get_variable('5dfb1a7e1d84721740527b94')
                elif parse[1] == "baro:":
                    var = api.get_variable('5dfb27f91d84722f098dd73b')             
                elif parse[1] == "humid:":
                    var = api.get_variable('5dfbc00c1d847274748affbd')
                elif parse[1] == "mail:":
                    var = api.get_variable('5dfbc10c1d84727786722701')             
                elif parse[1] == "gasr:":
                    var = api.get_variable('5dfbc1431d847277843eb54d')             
                elif parse[1] == "angy:":
                    var = api.get_variable('5dfbc17a1d847278d932665e')             
                elif parse[1] == "shake:":
                    var = api.get_variable('5dfbc1961d84727933e5b16f')

                elif parse[1] == "door:":
                    var = api.get_variable('5dfcf6be1d8472084f0d6e7c')

                    
            except simplejson.scanner.JSONDecodeError:
                return

            if var is not None:
                try:
                    val = var.save_value({'value': float(parse[2])})
                    print("sent val")
                except ValueError:
                    print("No JSON received, skipping")


            
if __name__ == '__main__':
    portname = '/dev/ttyUSB1'
    portbaud = '115200'
    #portbaud = '120000000'

    logfname = "/home/pi/outdoor.log"


    api = ApiClient(token='BBFF-gjV5Rkr2FXKR4DRCwjDvlBRSjruPKP')


    
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

        instr = bstr.decode('utf-8')
        if len(instr) > 0:
            #print(instr)
            parse_message(instr, logfname, api)
 
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
