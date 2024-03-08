#!/usr/bin/python
import time
import xmlrpc.client as xml
import argparse

def dissect(s):
    lines = s.split("\n")
    return(lines[1])

parser = argparse.ArgumentParser()
parser.add_argument("--proxy", type=str, default="http://localhost:36036", help="XMLRPC Server")
parser.add_argument("--sensorproxy", type=str, default="http://localhost:9090", help="XMLRPC Sensor Server")
parser.add_argument("--timeout", type=int, default=900, help="Watchdog timeout (seconds)")
args = parser.parse_args()
rpc = xml.ServerProxy(args.proxy)
rpc2 = xml.ServerProxy(args.sensorproxy)
stamp = 0
OKrets = ["None", "BADNODE"]
sleeptime = 3
while True:
    now = time.time()
    try:
        stamp = rpc.QueryTime(0)
        elspeed = rpc2.query_el_rate()
        azspeed = rpc2.query_az_rate()
        if (elspeed > 0.3 or azspeed > 0.5):
            rpc.Shutdown(0)
            time.sleep(1)
            rpc.Shutdown(1)
            time.sleep(1)
            rpc.SysExit(0)
            print ("EL or AZ speed exceeded--shutting down motion server!")
        alerts_0 = rpc.QueryAlerts(0)
        alerts_1 = rpc.QueryAlerts(1)
        if (alerts_0 not in OKrets):
            print ("Motor 0: alerts: %s" % dissect(alerts_0))
        if (alerts_1 not in  OKrets):
            print ("Motor 1: alerts: %s" % dissec(alerts_1))
        if (stamp > 1000 and ((now - stamp) > args.timeout)):
            rpc.Shutdown(0)
            time.sleep(5)
            rpc.Shutdown(1)
            time.sleep(5)
            rpc.SysExit(0)
        sleeptime = 10
    except:
        print ("No comms with server...sleeping")
        time.sleep (sleeptime)
        if (sleeptime < 60):
            sleeptime *= 2
    time.sleep(5)

        
