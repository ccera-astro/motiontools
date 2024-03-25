#!/usr/bin/python
import xmlrpc.client as xml
import time
import os
import sys
import math

#
# Rotation time -- 45 seconds
#
RTIME = 45.0
RRATE = 300.0

#
# Get XMLRPC objects for both sensor server and motor server
#
rpc_motor = xml.ServerProxy("http://localhost:36036")
rpc_sensors = xml.ServerProxy("http://localhost:9090")

#
# Spin up elevation at RRATE, wait RTIME seconds, then command it to zero
#
print ("Measuring elevation...")
initial_elev = rpc_sensors.query_both_axes()[0]
rpc_motor.Move(0,RRATE)
time.sleep(RTIME)
rpc_motor.Move(0,0.0)
time.sleep(1)
final_elev = rpc_sensors.query_both_axes()[0]
time.sleep(2)

#
# Spin up azimuth at RRATE, wait RTIME seconds, then command it to zero
#

print ("Measuring azimith")
initial_az = rpc_sensors.query_both_axes()[1]
rpc_motor.Move(1,RRATE)
time.sleep(RTIME)
rpc_motor.Move(1,0.0)
time.sleep(1)
final_az = rpc_sensors.query_both_axes()[1]

#
# Shutdown motors and server program
#
print ("Shutting down motors, and exiting server")
rpc_motor.Shutdown(0)
time.sleep(1)
rpc_motor.Shutdown(1)
time.sleep(1)
rpc_motor.SysExit(0)

#
# Calculate apparent ratios -- they won't be exact, because sloppy timing,
#  but this will reveal any gross differences between reality and expectation
#
rotations = RRATE*(RTIME/60.0)
el_degrees = abs(final_elev - initial_elev)
az_degrees = abs(final_az - initial_az)

print ("Total motion (DEG):  EL %f  AZ %f" % (el_degrees, az_degrees))

el_rotations = el_degrees / 360.0
az_rotations = az_degrees / 360.0

print ("Total motion (ROTATIONS): EL %f  AZ %f" % (el_rotations, az_rotations))

el_ratio = rotations / el_rotations
az_ratio = rotations / az_rotations

print ("Apparent EL gear ratio is: %f" % el_ratio)
print ("Apparent AZ gear ratio is; %f" % az_ratio)


