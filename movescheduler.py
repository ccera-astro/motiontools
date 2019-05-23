#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  movescheduler.py
#  
#  Copyright 2019 Marcus D. Leech <mleech@localhost.localdomain>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  
import os
import sys
import time
import math
import subprocess
import argparse


def main():
    
    parser = argparse.ArgumentParser(description='Schedule dish moves')
    parser.add_argument ("--latitude", dest='latitude', type=float, default=44.90, help="local geographic latitude")
    parser.add_argument ("--host", dest='host', type=str, default="localhost", help="host with motion hardware")
    parser.add_argument ("--user", dest='user', type=str, default="odroid", help="remote username")
    parser.add_argument ("--pass", dest='password', type=str, default="", help="remote password")
    parser.add_argument ("--hours", dest='hours', type=int, default=6, help="interval between moves (hours)")
    parser.add_argument ("--degrees", dest='degrees', type=int, default=5, help="motion increment (degrees)")
    parser.add_argument ("--min", dest='dec_min', type=int, default=-30, help="lower motion range value")
    parser.add_argument ("--max", dest='dec_max', type=int, default=30, help="upper motion range value")
    parser.add_argument ("--decfile", dest='decfile', type=str, default="declination.txt", help="declination update file")
    parser.add_argument ("--start", dest="start", type=int, default=0, help="starting motion index")
    parser.add_argument ("--rcmd", dest="rcmd", type=str, default="sudo ./newmoveto.py", help="remote command")
    
    args=parser.parse_args()
    
    #
    # Motion event period, in seconds
    #
    period = args.hours * 3600
    
    #
    # To adjust for local latitude and convert to elevation relative to south
    #   horizon
    #
    rotation = 90.0 - args.latitude
    
    #
    # Dish mount covers DEC range of args.dec_min to args.dec_max
    #   every args.hours, we move args.degrees degrees
    #
    schedule = range(args.dec_min,args.dec_max,args.degrees)+range(args.dec_max,args.dec_min,-args.degrees)
    
    i = args.start

    donetime=time.time()
    while True:
        #
        # Move to a new DEC every args.hours
        #
        goodmove = False
        lsecs = time.time()
        if ((lsecs % period) in [0,1,2,3] and (lsecs - donetime) > (period/3)):
            desired = schedule[i % len(schedule)]
            elevation = desired + rotation
            try:
                print "Moving to %d (%d)" % (elevation, desired)
                cmdstr = "sshpass -p %s ssh %s@%s %s %d >movement.log 2>&1" % (args.password, args.user, args.host, args.rcmd, elevation)
                retv = os.system(cmdstr)
                donetime = time.time()
                if (retv != 0):
                    goodmove = False
                else:
                    f = open("movement.log", "r")
                    ls = f.readlines()
                    f.close()
                    for l in ls:
						if ("Achieved" in l):
							i += 1
							goodmove = True
							break
            except:
                print "Exception was raised"
                goodmove = False
            if (goodmove == True):
                print "Movement success"
                f = open (args.decfile, "w")
                f.write ("%f\n" % desired)
                f.close()
                i += 1
            else:
				print "Movement failed"
        time.sleep(1.0)

    return 0

if __name__ == '__main__':
    main()

