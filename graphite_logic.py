#!/usr/bin/python3
# coding=utf-8
"""
----------------------------------------------------------------------
Send and recieve data from from graphite/carbon database
https://graphite.readthedocs.io/en/latest/render_api.html
----------------------------------------------------------------------
"""
#----------------------------------------------------------------------


import sys
import time 
import requests
import socket
# local serial wrapper class
#sys.path.append('../common')
import simplejson as json


cserver = "pidp.local"



def carbon_write(varname, value, cserver):
    sock = socket.socket()
    sock.connect( (cserver, 2003) )
    msg = "{}.metric {} {}\n".format(varname, value, time.time())
    sock.send(msg.encode('utf-8'))
    sock.close()
    print("sent carbon " + msg)


carbon_write('test', float(99.55), cserver)    

r = requests.get('http://' + cserver + ':8013/render?target=test.metric&from=-90s&format=json')
#r = requests.get('http://' + cserver + ':8013/render?target=baro.metric&from=-90s&format=json')

print(r.json())  # this will give your the JSON file with the data

foo = r.json()

#print(foo)

bar = foo[0]

print(bar['target'])
#print(bar['datapoints'])

for dp in bar['datapoints']:
    print("val: {}  time: {}".format(dp[0],dp[1]))
    print("stale: {} seconds".format(time.time() - dp[1]))

