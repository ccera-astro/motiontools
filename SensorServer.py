#!/usr/bin/python

import os
import sys
import argparse
import xmlrpc.server
import minimalmodbus as mb
from xmlrpc.server import SimpleXMLRPCServer
import threading

#
# The sensor code may move into a separate XMLRPC server at some point,
#  so that other parts of our overall system at the observatory can
#  get at it.
#
elSensor = None
azSensor = None
sensorFailed = False
SENSORS_PORT = '/dev/ttyUSB0'
SENSOR_BITS = 2**14

def init_sensor_system():
    global elSensor
    global azSensor
    
    if (elSensor == None or azSensor == None):
        try:
            elSensor = mb.Instrument(SENSORS_PORT, 1)
            azSensor = mb.Instrument(SENSORS_PORT, 2)
            elSensor.serial.baudrate = 9600
        except:
            elSensor = None
            azSensor = None
            sensorFailed = True
        
#
# Functions for position sensors
#
def get_el_sensor():
    global elSensor
    init_sensor_system()
    try:
        r = elSensor.read_register(0, 0)
        return (float(r)/float(SENSOR_BITS))*360.0
    except:
        return -1000.0


def get_az_sensor():
    init_sensor_system()
    global azSensor
    try:
        r = azSensor.read_register(0, 0)
        return (float(r)/float(SENSOR_BITS))*360.0
    except:
        return -1000.0

parser = argparse.ArgumentParser()
parser.add_argument("--xmlport", type=int, default=None, help="XML Port")
parser.add_argument("--serialport", type=int, default=SENSOR_PORT, help="Serial port")
args = parser.parse_args()

SENSOR_PORT = args.serialport

if (args.xmlport != None):
    xmlserver = SimpleXMLRPCServer(('0.0.0.0', args.xmlport), allow_none=True, logRequests=False)
    xmlserver.register_function(get_el_sensor)
    xmlserver.register_function(get_az_sensor)
    server_thread = threading.Thread(target=xmlserver.serve_forever)
    server_thread.daemon = True
    server_thread.start()
