#!/usr/bin/python
import time
import xmlrpc.client as xml
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--proxy", type=str, default="http://localhost:36036", help="XMLRPC Server")
parser.add_argument("--timeout", type=int, default=900, help="Watchdog timeout (seconds)")
args = parser.parse_args()
rpc = xml.ServerProxy(args.proxy)

while True:
    now = time.time()
    stamp = rpc.QueryTime(0)
    if ((now - stamp) > args.timeout):
        rpc.Shutdown(0)
        time.sleep(5)
        rpc.Shutdown(1)
        time.sleep(5)
        rpc.SysExit(0)
        break
    time.sleep(10)

        
