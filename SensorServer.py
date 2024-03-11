#!/usr/bin/python
#
# This little XMLRPC server interacts with the AZ and EL sensors, and
#  serves out their information.
#
import os
import sys
import argparse
import xmlrpc.server
import minimalmodbus as mb
from xmlrpc.server import SimpleXMLRPCServer
import threading

current_elev = 0.0
current_az = 0.0
last_elev = -99.00
last_az = -99.00
elev_rate = 0.0
az_rate = 0.0

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

def query_el_sensor():
    global current_elev
    return (current_elev)
    
def query_el_rate():
    global elev_rate
    return (elev_rate)

def query_az_sensor():
    global current_az
    return (current_az)

def query_az_rate():
    global az_rate
    return (az_rate)

def query_both_axes():
    global current_az
    global current_elev
    
    return (current_elev,current_az)

def device_loop():
    global current_az
    global current_elev
    
    while (True):
        current_az = get_az_sensor()
        if (last_az < -20.0):
            last_az = current_az
            last_az_time = time.time()
        time.sleep(0.05)

        current_elev = get_el_sensor()
        if (last_elev < -20.0):
            last_elev = current_elev
            last_elev_time = time.time()
        
        #
        # Compute rates
        #

        #
        # Elevation
        t = current_elev - last_elev
        elev_rate = abs(t) / (time.time() - last_elev_time())
        
        #
        # Azimuth
        #
        t = current_az - last_az
        az_rate = abs(t) / (time.time() - last_az_time())
        
        #
        # Once computed, make "last" == "current"
        #
        last_elev = current_elev
        last_elev_time = time.time()
        last_az = current_az
        last_az_time = time.time()
        time.sleep(0.250)

parser = argparse.ArgumentParser()
parser.add_argument("--xmlport", type=int, required=True, help="XML Port")
parser.add_argument("--serialport", type=int, default=SENSORS_PORT, help="Serial port")
args = parser.parse_args()

SENSORS_PORT = args.serialport

#
# Create and run the XML server in a separate thread
#
xmlserver = SimpleXMLRPCServer(('0.0.0.0', args.xmlport), allow_none=True, logRequests=False)
xmlserver.register_function(query_el_sensor)
xmlserver.register_function(query_az_sensor)
xmlserver.register_function(query_az_rate)
xmlserver.register_function(query_el_rate)
server_thread = threading.Thread(target=xmlserver.serve_forever)
server_thread.daemon = True
server_thread.start()

#
# Loop getting values from the sensor hardware
#
device_loop()
