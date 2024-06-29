# this module will be imported in the into your flowgraph
import time
import numpy
import os
import sys
import xmlrpc.client as xml
import time

RPC = None
RPC2 = None

InitReset = False

def init_RPC(srvr):
    global RPC
    global InitReset
    if (RPC == None):
        try:
            RPC = xml.ServerProxy(srvr)
        except:
            RPC = None
    if (RPC is not None and InitReset is False):
        try:
            RPC.AccLimit(0, 2500)
            time.sleep(0.25)
            RPC.VelLimit(0, 1600)
            time.sleep(0.5)
            RPC.AccLimit(1, 2500)
            time.sleep(0.25)
            RPC.VelLimit(1, 1600)
        except:
            pass

def sysExit(v,srvr):
    global RPC
    init_RPC(srvr)
    
    if (v == 1):
        try:
            RPC.SysExit(0)
        except:
            pass
    
def init_RPC2(srvr):
    global RPC2
    if (RPC2 == None):
        RPC2 = xml.ServerProxy(srvr)

def ResetLimits(btn,srvr):
    if (btn == 1):
        axLimits(0,srvr)
        time.sleep(0.25)
        axLimits(1,srvr)
       
def axLimits(ax,srvr):
    global RPC
    init_RPC(srvr)
    try:
        RPC.AccLimit(ax, 2200)
        time.sleep(0.25)
        RPC.VelLimit(ax, 1600)
        rv = True
    except:
        rv = False
    return rv

def tryMove(lbl, ax,spd):
    global RPC
    try:
        RPC.HeartBeat(0)
        time.sleep(0.333)
        RPC.Move(ax,spd)
    except:
        print ("Exception in move on axis %d/%f" % (ax, spd))
        pass

upstate = 0
def elev_up(up,speed,srvr):
    global RPC
    global upstate

    init_RPC(srvr)
    if (RPC != None):
        if (upstate != up):
            upstate = up
            if (up == 1):
                tryMove("Up", 0,speed)
            if (up == 0):
                tryMove("Up", 0,0.0)
            
downstate = 0          
def elev_down(down,speed,srvr):
    global RPC
    global downstate

    init_RPC(srvr)
    if (RPC != None):
        if (down != downstate):
            downstate = down
            if (down == 1):
                tryMove("Down", 0,-1.0*speed)
            if (down == 0):
                tryMove("Down", 0,0.0)
weststate = 0               
def azim_west(west,speed,srvr):
    global RPC
    global weststate
    init_RPC(srvr)
    if (RPC != None):
        if (weststate != west):
            weststate = west
            if (west == 1):
                tryMove("West", 1, speed)
            if (west == 0):
                tryMove("West", 1, 0.0)
eaststate = 0
def azim_east(east,speed,srvr):
    global RPC
    global eaststate
    init_RPC(srvr)
    if (RPC != None):
        if (eaststate != east):
            eaststate = east
            if (east == 1):
                tryMove("East", 1, -1.0*speed)
            if (east == 0):
                tryMove("East", 1, 0.0)

def get_posns(pacer,srvr):
    global RPC2
    init_RPC2(srvr)
    try:
        posns = RPC2.query_both_axes()
    except:
        #print ("get_posns called, but exception was raised")
        posns = (-1000, -1000)
    return(posns)

def do_heartbeat(pacer,buttons):
    global RPC
    if (1 in buttons):
        try:
            RPC.HeartBeat(0)
        except:
            pass
    return 1

import requests
import time
def pwr_on(val,ipaddr):
    if (val == 1):
        try:
            #print ("PWR ON with: %d" % val)
            r = requests.get('http://%s/30000/03' % ipaddr, timeout=3)
            time.sleep(0.5)
            r = requests.get('http://%s/30000/02' % ipaddr, timeout=3)
        except Exception as e:
            print (e)
            pass
    return True

def pwr_off(val,ipaddr):
    if (val == 1):
        try:
            #print ("PWR OFF with: %d" % val)
            r = requests.get('http://%s/30000/01' % ipaddr, timeout=3)
            time.sleep(0.5)
            r = requests.get('http://%s/30000/00' % ipaddr, timeout=3)
        except Exception as e:
            print(e)
            pass
    return True
    

