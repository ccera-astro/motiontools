#!/usr/bin/python
import xmlrpc.client as xml
import time
import os
import sys
import math



#
# Get XMLRPC objects for both sensor server and motor server
#
rpc_motor = xml.ServerProxy("http://localhost:36036")

rpc_motor.AngleMove(0,float(360.0*10.0))
print ("Done with forward move")
time.sleep(3)
rpc_motor.AngleMove(0,float(-1.0*(360.0*10.0)))
print ("Done with reverse move")
