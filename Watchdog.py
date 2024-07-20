#!/usr/bin/python
import time
import xmlrpc.client as xml
import argparse
import requests

def dissect(s):
    lines = s.split("\n")
    return(lines[1])

parser = argparse.ArgumentParser()
parser.add_argument("--proxy", type=str, default="http://localhost:36036", help="XMLRPC Server")
parser.add_argument("--sensorproxy", type=str, default="http://localhost:9090", help="XMLRPC Sensor Server")
parser.add_argument("--timeout", type=int, default=120, help="Watchdog timeout (seconds)")
parser.add_argument("--relay", type=str, default="192.168.1.4", help="relay server address")

args = parser.parse_args()
rpc = xml.ServerProxy(args.proxy)
rpc2 = xml.ServerProxy(args.sensorproxy)
stamp = 0
OKrets = ["None", "BADNODE"]
sleeptime = 3

commanded_standby = False
requests.get("http://%s/30000/04" % args.relay)
requests.get("http://%s/30000/06" % args.relay)
while True:
    now = time.time()
    try:
        tmstrings = rpc.QueryTime(0)
        tmstrings = tmstrings.split(",")
        stamp = float(tmstrings[0])
        elspeed = float(tmstrings[1])
        azspeed = float(tmstrings[2])
        #
        # Turn OFF lights and horn if both axes have gone to (effectively) 0 speed
        #
        if (elspeed <= 0.0 and azspeed <= 0.0):
            result = requests.get("http://%s/30000/04" % args.relay)
            result = requests.get("http://%s/30000/06" % args.relay)
        #
        # Turn ON lights and horn if both axes have turned on
        #
        
        #
        # Lights first
        #
        if (elspeed > 0.00 or azspeed > 0.0):
            result = requests.get("http://%s/30000/05" % args.relay)
        #
        # Then horn
        #
        # Speeds are in RPM from the QueryTime call
        #
        if (elspeed >= 100.0 or azpeed >= 50.0):
            result = requests.get("http://%s/30000/07" % args.relay)
        if (elspeed > 1600 or azspeed > 1600):
            rpc.Shutdown(0)
            time.sleep(3)
            rpc.Shutdown(1)
            time.sleep(1)
            rpc.SysExit(0)
            print ("EL or AZ speed exceeded--shutting down motion server!")
        else:
            alerts_0 = rpc.QueryAlerts(0)
            alerts_1 = rpc.QueryAlerts(1)
            if (alerts_0 not in OKrets):
                print ("Motor 0: alerts: %s" % dissect(alerts_0))
            if (alerts_1 not in  OKrets):
                print ("Motor 1: alerts: %s" % dissect(alerts_1))
            if (commanded_standby is False and stamp > 1000 and ((now - stamp) >= args.timeout)):
                print ("Timeout reached (%f)--putting motors in stand-by" % (now-stamp))
                rpc.Shutdown(0)
                time.sleep(1)
                rpc.Shutdown(1)
                commanded_standby = True
            if (commanded_standby is True and ((now - stamp) < args.timeout)):
                commanded_standby = False
        sleeptime = 10
    except Exception as e:
        #print ("No comms with server...sleeping")
        #print (e)
        time.sleep (sleeptime)
        if (sleeptime < 60):
            sleeptime += 10
            
    time.sleep(5)

        
