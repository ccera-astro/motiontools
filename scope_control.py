#!/usr/bin/python

import struct
import socket
import time
import ephem
import argparse
import xmlrpc.client as xml
import math

#
# Protocol fields for Stellarium telescope-control protocol
#

LENGTH = 24

#
# Type 0 == Tell Stellarium where you're pointing
#
TYPE = 0
TIME = 0

STATUS = 0

HOST = 'localhost'
PORT = 10001

def main():
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--latitude", type=float, default=45.3491, help="Local latitude")
    parser.add_argument("--longitude", type=float, default=-76.0413, help="local longitude")
    parser.add_argument("--xmlrpc", type=str, default="http://localhost:9090")
    
    args = parser.parse_args()
    
    rpc = xml.ServerProxy(args.xmlrpc)
    
    local = ephem.Observer()
    local.lon = math.radians(args.longitude)
    local.lat = math.radians(args.latitude)
    local.elevation = 100
    local.date = ephem.now()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # enable address reuse
    s.bind((HOST, PORT))
    s.listen(1)
    conn, addr = s.accept()
    print ('Connected by', addr)

    running = 1
    conn.settimeout(1.0)
    while running:
        local.date = ephem.now()
        try:
            posns = rpc.query_both_axes()
            el = posns[0]
            az = posns[1]
        except:
            el = 63.0
            az = 180.0
            
        ra, dec = local.radec_of(az=math.radians(az), alt=math.radians(el))
        
        #
        # Do a bit of units conversion
        #
        ra = math.degrees(ra)
        ra /= 360.0
        ra *= 24.0
        
        RA = (ra/24.0) * float(1<<32)
        RA = int(RA)
        
        dec = math.degrees(dec)
        DEC = (dec/90.0) * float(1<<30)
        DEC = int(DEC)
        print ("RA: %f DEC %f" % (ra, dec))
        TIME = (time.time()/1e6)
        TIME = int(TIME)
        fmt = '<hhqIii'
        netPacket = struct.pack(fmt, LENGTH, TYPE, TIME, int(RA), int(DEC), int(STATUS))
        #print ("sending data length:  %d" % len(netPacket))
        conn.send(netPacket)
        try:
            recvPacket = conn.recv(256)
            if (len(recvPacket) > 0):
                command_tuple = struct.unpack('<hhqIi', recvPacket)
                slew_ra = command_tuple[3]
                slew_dec = command_tuple[4]
                slew_ra = float(slew_ra) / float(1<<32)
                slew_ra *= 24.0
                
                slew_dec = float(slew_dec) / float(1<<30)
                slew_dec *= 90.0
                
                print ("Got slew command: RA %f DEC %f" % (slew_ra, slew_dec))
            else:
                print ("Client disconnected.  Done")
                break
            
        except TimeoutError:
            pass
        except Exception as e:
            print ("Error on socket during receive")
            print (e)
            break
 

    conn.close()

if __name__ == '__main__':
    main()
