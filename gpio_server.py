#!/usr/bin/python
import telnetlib
from argparse import ArgumentParser
import xmlrpc.server
from xmlrpc.server import SimpleXMLRPCServer
import time
import threading

states = {"EBRK1" : -1, "EBRK2" : -1, "ABRK1" : -1, "ABRK2" : -1, "ESTOW" : -1, "ASTOW": -1}
names = ["EBRK1", "EBRK2", "ABRK1", "ABRK2", "ESTOW", "ASTOW"]

def get_input(num,tnc):
    regstr = "gpio read %d"  % num
    tnc.write(regstr.encode('ascii') + b"\n")
    time.sleep(0.1)
    reg = tnc.read_until(b">", timeout=0.5)
    #reg = reg.strip(b"\r\n")
    reg = reg.strip(b">")
    reg = reg.strip(b"\r")
    reg = reg.strip(b"\n")
    return(reg)

def query_switch_states():
    global states
    return (str(states))

def query_single_switch(swname):
    global states
    try:
        return (states[swname])
    except:
        return -1


parser = ArgumentParser()

parser.add_argument("--xmlport", type=int, default="6660", help="XML Port")
parser.add_argument("--gpioaddress", type=str, default="192.168.1.5", help="gpio TELNET address")

args = parser.parse_args()


tnc = telnetlib.Telnet(args.gpioaddress)
exp = tnc.read_until(b": ")
time.sleep(0.2)
tnc.write("admin".encode('ascii') + b"\n")
exp = tnc.read_until(b": ")
time.sleep(0.2)
tnc.write("admin".encode('ascii') + b"\n")
exp = tnc.read_until(b">")
time.sleep(0.2)
tnc.write("gpio iodir FFFF".encode('ascii') + b"\n")
exp = tnc.read_until(b">")
time.sleep(0.25)

xmlserver = SimpleXMLRPCServer(('0.0.0.0', args.xmlport), allow_none=True, logRequests=False)
xmlserver.register_function(query_switch_states)
xmlserver.register_function(query_single_switch)
server_thread = threading.Thread(target=xmlserver.serve_forever)
server_thread.daemon = True
server_thread.start()
while True:
    for regs in range(len(names)):
        r = get_input(regs,tnc)
        #print("%d %s" % (regs, r.decode('ascii')))
        states[names[regs]] = int(r.decode('ascii'))
        time.sleep(0.10)
    #print (states)
    time.sleep(1)

