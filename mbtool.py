#!/usr/bin/python3
import os
import sys
import argparse
import minimalmodbus as mb
import time

registers = {"address" : 0x0004, "position" : 0x0000,
    "rotation" : 0x0009, "encoder" : 0x000B, "midpoint" : 0x000E}

parser = argparse.ArgumentParser()

parser.add_argument("--device", type=str, default="/dev/ttyUSB0", help="Serial Device")
parser.add_argument("--addr", type=int, default=1, help="Sensor address (1-127)")
parser.add_argument("--register", type=str, default=None, help="Which register")
parser.add_argument("--value", type=int, default=None, help="Value to place in register")
parser.add_argument("--optype", type=str, default=None, help="Optype: read or write")
parser.add_argument("--continuous", type=bool, default=False, action="store_true")
args = parser.parse_args()

Sensor = mb.Instrument(args.device, args.addr)
Sensor.serial.baudrate = 9600

if (args.register not in registers):
    print ("Unknown register %s\n" % args.register)
    exit(0)

reg = registers[args.register]
    
if (args.optype == "read"):
    # Stuff
    loopcount = 1
    if (args.continuous == True):
        loopcount = 1000
    for i in range(loopcount):
        v = Sensor.read_register(reg, 0)
        flconv = (float(v)/ float(16384)) * 360.0
        print ("%04x %f" % (v, flconv))
        time.sleep(0.5)
        
elif (args.optype == "write"):
    if (args.value != None):
        v = Sensor.write_register(reg, args.value)
    else:
        print ("Failed to specify --value for write operation\n")
else:
    print ("Unknown --optype.  Doing nothing")


