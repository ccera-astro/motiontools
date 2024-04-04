#!/usr/bin/python
import ephem
import time
import os
import sys
import argparse

def precess(t_ra,t_dec,lat,lon):
	#
    # Prime ephem to know about our location and time
    #
    local = ephem.Observer()
    local.lat = str(lat)#to_ephem_coord(lat)
    local.long = str(lon)#to_ephem_coord(lon)
    local.elevation = 30
    local.pressure = 0
    local.epoch = ephem.J2000
    
    #
    # Do intial compute on the target
    #  celestial coordinate
    #
    local.date = ephem.now()
    v = ephem.FixedBody()
    e = ephem.Equatorial(t_ra, t_dec)
    v._ra = e.ra
    v._dec = e.dec
    v._epoch = ephem.J2000
    v.compute(local)
    print ("Precessed coordinates: RA %s DEC %s" % (v.g_ra, v.g_dec))


parser = argparse.ArgumentParser()

parser.add_argument("--latitude", type=float, default=45.3491, help="Local latitude")
parser.add_argument("--longitude", type=float, default=-76.0431, help="Local longitude")
parser.add_argument("--ra", type=str, required=True, help="Right Ascension")
parser.add_argument("--dec", type=str, required=True, help="Declination")

args = parser.parse_args()

precess(args.ra, args.dec, args.latitude, args.longitude)
