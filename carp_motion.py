#!/usr/bin/python3
import math
import os
import sys
import datetime
import time
import xmlrpc.client as xml
import argparse
import traceback
import ephem
import numpy
import random

#
# Target error when slewing
#
SERROR = 0.035

#
# Establish some constants
#
LATITUDE = 45.3491
LONGITUDE = -76.0413
ELEVATION = 70.00

#
# Pre-reduction ahead of the main gearboxes
#
ELEV_PREFIX = 7.5
AZ_PREFIX = 5.0

#
# Total ratios from the PREFIX gearing, and the main gearing
#

#
# Ratio of the main gearboxes, up to the output pinion
#
BOX_RATIO = (9.8125 * 8.368 * 6.095)

#
# The main gearboxes, out to the drive pinion are all the same,
#   given above as BOX_RATIO.
#
# But elevation and azimuth have different ratios between their
#  ouptut pinions and the ring or sector gear that they drive
#
ELEV_RATIO = ELEV_PREFIX * (BOX_RATIO * 16.0)
AZIM_RATIO = AZ_PREFIX * (BOX_RATIO * 10.0)

#
# Signs (might need to flip one or both of these on the actual machine)
#
ELEV_SIGN = 1.0
AZ_SIGN = 1.0

#
# Degrees per minute on the equator for sidereal
#
DEG_MINUTE = 0.25

#
# Soft limits for motion
#
ELEVATION_LIMITS = (0.5,89.5)
AZIMUTH_LIMITS = (0.2,359.8)

#
# Gear spin max
#
# Can be modified by command-line parameter
#
GEAR_SPIN_MAX = 1600.0
gear_spin_max = GEAR_SPIN_MAX

#
# Acceleration limit -- RPM/SEC
# Default.  Can be modifed by command-line limit
#
ACC_LIMIT = 2750

EQUANT = (1.0/16834.0)*360.0

#
# Motor server interface
#
def send_heartbeat():
    global rpc
    return rpc.HeartBeat(0)
    
def set_az_speed(spd):
    global rpc
    return rpc.Move(1,float(spd*AZ_SIGN))

def set_el_speed(spd):
    global rpc
    return rpc.Move(0,float(spd*ELEV_SIGN))

def query_el_torque():
    global rpc
    return rpc.QueryTorque(0)

def query_az_torque():
    global rpc
    return rpc.QueryTorque(1)

def move_az_angle(angle):
    global rpc

    shaft_angle = angle * AZIM_RATIO
    return rpc.AngleMove(1,float(shaft_angle*AZ_SIGN))

def wait_az_move(count):
    return rpc.MoveWait(1, count)

def move_el_angle(angle):
    global rpc

    shaft_angle = angle * ELEV_RATIO
    return rpc.AngleMove(0,float(shaft_angle*ELEV_SIGN))

def wait_el_move(count):
    return rpc.MoveWait(0, count)

def set_el_acclimit(limit):
    global rpc
    return rpc.AccLimit(0,int(limit))

def set_az_acclimit(limit):
    global rpc
    return rpc.AccLimit(1,int(limit))

def set_el_vellimit(limit):
    global rpc
    return rpc.VelLimit(0,int(limit))

def set_az_vellimit(limit):
    global rpc
    return rpc.VelLimit(1,int(limit))
#
# Sensor interface
#
def get_el_sensor():
    global rpc2
    return (rpc2.query_el_sensor())

def get_az_sensor():
    global rpc2
    return (rpc2.query_az_sensor())

def get_both_sensors():
    global rpc2

    val1 = rpc2.query_both_axes()
    time.sleep(0.15)
    val2 = rpc2.query_both_axes()
    val = numpy.add(val1, val2)
    val = numpy.divide(val,2.0)
    val = (float(val[0]),float(val[1]))
    return (val)

def restore_limits():
    set_az_acclimit(2000)
    set_az_vellimit(1600)
    set_el_acclimit(2000)
    set_el_vellimit(1600)
#
# Take a decimal-degrees coordinate, and transform to the HH:MM:SS
#   that ephem uses
#
def to_ephem_coord(decimal):
    secs = abs(decimal * 3600)
    hours = int(secs/3600.0)
    mins = int((secs - hours*3600)/60.0)
    secs = secs-int((hours * 3600)+(mins*60))
    if (decimal < 0.0):
        hours *= -1
    convstr = "%02d:%02d:%02d" % (hours, mins, secs)
    #print ("conversion %s" % convstr)
    return convstr

#
# Take an HH:MM:SS coordinate from ephem, and turn into decimal degrees
#
def from_ephem_coord(coord):
    q = coord.split(":")
    degs = float(q[0])
    mins = float(q[1])
    secs = float(q[2])
    sgn = 1.0
    if (degs < 0):
        sgn = -1.0
        degs = abs(degs)

    decimal = degs + (mins/60.0) + (secs/3600.0)
    decimal = decimal * sgn
    return (decimal)

#
# Given longitude(decimal degrees as a float)
#
# Return the current sidereal time as a string with
#  "," separated tokens
#
def cur_sidereal(longitude,latitude):
    longstr = to_ephem_coord(longitude)
    latstr = to_ephem_coord(latitude)
    x = ephem.Observer()
    x.date = ephem.now()
    x.long = longstr
    x.lat = latstr
    #print ("x is %s" % str(x))
    #jdate = ephem.julian_date(x)
    tokens=str(x.sidereal_time()).split(":")
    #print("sid time %s" % x.sidereal_time())
    hours=int(tokens[0])
    minutes=int(tokens[1])
    seconds=int(float(tokens[2]))
    sidt = "%02d,%02d,%02d" % (hours, minutes, seconds)
    return (sidt)

#
# Determine initial tracking rate--measurements will alter this
#  dynamically
#
def track_rate(declination):
    r = math.cos(math.radians(math.fabs(declination)))
    r = r * DEG_MINUTE
    r_elev = (r * ELEV_RATIO) / 360.0
    r_azim = (r * AZIM_RATIO) / 360.0
    return ((r_elev,r_azim))

#
# Return a speed proportional to difference between limit and actual
#
# We quantize to QUANTUM steps, to reduce the amount of needless changing of
#  motor speed at our loop cadence.
#
# The default proportionality is non-linear
#
# But if the "linear" parameter is true, it uses a linear ramp
#
#
QUANTUM=15
def proportional_speed(pdiff,lim,linear):
    global gear_spin_max
    if (linear is False):
        v1 = lim-pdiff
        v2 = gear_spin_max/(1+1.35*(v1*v1))
        v2 = int(int(v2/QUANTUM) * QUANTUM)
        v2 = float(v2)
        return v2
    else:
        v1 = pdiff / lim
        r = gear_spin_max - (gear_spin_max/15)
        v2 = (r * v1) + (gear_spin_max/15)
        v2 = int(int(v2/QUANTUM) * QUANTUM)
        v2 = float(v2)
        return v2

#
# Degrees/second from RPM and ratio
#
def dps(rpm, ratio):
    x = rpm / ratio
    x *= 360.0
    x /= 60.0
    return x

def dps_to_rpm(d, ratio):
    x = 60.0 * d
    x /= 360.0
    x *= ratio
    return x

def min_c_rpm(qinterval, ctime, ratio):
    minc = 4.0*qinterval
    minc /= ctime
    return (dps_to_rpm(minc, ratio))
    
#
# We start going into proportional control at this limit in offset between
#  the current and target position (degrees)
# Command line can change this
#
PROP_LIMIT = 1.75
prop_limit = PROP_LIMIT

#
# Determine desired slew rate, based on axis position offsets from target
#
# Basically, as we get closer, we slow down, to prevent over-shoot
#
# targ_el - target elevation in decimal-float format
# targ_az - target azimuth in decimal-float format
# cur_el - current elevation in decimal-float format
# cur_az - current azimuth in decimal-float format
# linear - boolean to indicate linear ramp for proportional speed
#
# Returns:  (el_slew, az_slew)   slew speeds in decimal-float
#
def slew_rate(targ_el, targ_az, cur_el, cur_az, linear, frate_el, frate_az):
    global gear_spin_max
    global prop_limit
    #
    # Compute for elevation
    #  For distances > "prop_limit", we are kind of "open loop"
    #  We just spin at the maximum drive rate
    #
    #
    if (abs(targ_el - cur_el) > prop_limit):
        el_slew = gear_spin_max
    else:
        #
        # This will ensure that the proportional rate will never dip below
        #   4 times the current sky rate.  We don't want a situation where
        #   we're moving slower than the sky, or just-barely-faster.
        #
        el_slew = proportional_speed(abs(targ_el - cur_el), prop_limit, linear)
        el_slew = max(el_slew, dps_to_rpm(frate_el*2.0, ELEV_RATIO))
    #
    # Compute for azimuth
    # Azimuth moves faster than elevation, so adjust the resulting curve
    #  a bit.
    #
    if (abs(targ_az - cur_az) > prop_limit*1.333):
        az_slew = gear_spin_max
    else:
        #
        # This will ensure that the proportional rate will never dip below
        #   4 times the current sky rate.  We don't want a situation where
        #   we're moving slower than the sky, or just-barely-faster.
        #
        az_slew = proportional_speed(abs(targ_az - cur_az), prop_limit*1.333, linear)
        az_slew = max(az_slew, dps_to_rpm(frate_az*2.0, AZIM_RATIO))

    #
    # Adjust sign
    #
    if (targ_el < cur_el):
        el_slew *= -1.0
    if (targ_az < cur_az):
        az_slew *= -1.0

    return ((el_slew,az_slew))

def exit_motion_server():
    global rpc
    
    rpc.SysExit(0)
    
def my_exit(rc,sexit):
    if (sexit):
        exit_motion_server()
    time.sleep(1)
    exit(rc)
    
#
# Pause time between slewing-rate updates
#
# Can be changed by command-line parameter
#
PAUSE_TIME = 0.333

#
# Sanity-checking interval
#
# Can be changed by command-line parameter
#
SANITY_TIME = 5.0

import ctypes

#
# Move dish to target
#
# t_ra - target RA in decimal-float format
# t_dec - target DEC in decimal-float format
# lat - local geo latitude in decimal-float format
# lon - local geo longitude in decimal-float format
# elev - local elevation in decimal/float format
# azoffset, eloffset - fixed offsets (used for offset feeds, etc)
# lfp - pointer to logging file object
# absolute - move to absolute position
# posonly - just compute positions print, and exit -- don't do any dish movement
# body - body object from ephem for planets
# ptime - time to pause through each iteration in the loop
# stime - sanity-checking time to detect stuck sensors, etc
# linear - use a linear speed-reduction model
# serror - allowable error to declare "on-target"
#
# Returns: True for success False otherwise
#
def moveto(t_ra, t_dec, lat, lon, elev, azoffset, eloffset, lfp, absolute, posonly, body,
    ptime, stime, linear, serror):
        
    global gear_spin_max

    #
    # Prime ephem to know about our location and time
    #
    local = ephem.Observer()
    local.lat = str(lat)#to_ephem_coord(lat)
    local.long = str(lon)#to_ephem_coord(lon)
    local.elevation = elev
    local.pressure = 0
    local.epoch = ephem.J2000

    #
    # Do intial compute on the target
    #  celestial coordinate
    #
    local.date = ephem.now()
    if (body is None):
        v = ephem.FixedBody()
        e = ephem.Equatorial(to_ephem_coord(t_ra), to_ephem_coord(t_dec))
        v._ra = e.ra
        v._dec = e.dec
        v._epoch = ephem.J2000
    else:
        v = body
        t_ra = from_ephem_coord(str(v.ra))
        t_dec = from_ephem_coord(str(v.dec))

    #
    # Compute current circumstances for the object (contained in 'v')
    #
    v.compute(local)

    #
    # Keep track of current speed
    #
    el_speed = 0.0
    az_speed = 0.0

    #
    # We assume that the axis will start in "running" state
    #
    el_running = True
    az_running = True

    #
    # Set return value
    #
    rv = True

    #
    # Pick up initial values for sensors
    # We use these to compare current actual position, on a lazy cadence
    #   (every SANITY_TIME seconds), to check that if there's a non-zero commanded
    #   motor speed, there is at least *some* motion over that interval.
    #
    if (posonly is False):
        last_time_sensors = time.time()
        cmp_el, cmp_az  = get_both_sensors()
        time.sleep(ptime)
        cur_el = cmp_el
        cur_az = cmp_az

    #
    # Set limits speed/accel limits
    #
    if (posonly is False):
        set_el_acclimit(1650)
        time.sleep(0.250)
        set_el_vellimit(int(gear_spin_max))

        time.sleep(0.250)
        set_az_acclimit(1650)
        time.sleep(0.250)
        set_az_vellimit(int(gear_spin_max))

    #
    # Forever, until we zero-in on the target
    #
    final_el_rate = 0.0
    final_az_rate = 0.0

    heartbeat_time = time.time()
    while True:
        
        #
        # Let motor server know we're still alive -- we don't do this on
        #  every iteration, since our cadence is quite high, but rather
        #  only every 10 seconds or so.
        #
        if ((time.time() - heartbeat_time) >= 10):
            send_heartbeat()
            heartbeat_time = time.time()
        limits = False
        #
        # Looks like down below, we have stopped motion on both
        #  axes--we exit this loop when this happens
        #
        if (posonly is False):
            if (az_running is False and el_running is False):
                set_az_speed(0.0)
                set_el_speed(0.0)
                break

        #
        # Update the ephem sky coordinates, which can still creep as we
        #   run this control loop, since, you know, the sky moves....
        #

        #
        # If not an absolute move
        #
        if (absolute is False):
            local.date = ephem.now()
            v.compute(local)
            t_az = math.degrees(v.az) + azoffset
            t_el = math.degrees(v.alt) + eloffset
            
            #
            # Compute future
            #
            local.date = ephem.now() + (20.0 / 86400.0)
            v.compute(local)
            f_az = math.degrees(v.az) + azoffset
            f_el = math.degrees(v.alt) + eloffset
            local.date = ephem.now()
            
            #
            # We use this as a rate computation near the very end of slewing
            # 
            final_az_rate = abs(f_az - t_az) / 20.0
            final_el_rate = abs(f_el - t_el) / 20.0
            
            if (posonly is True):
                print ("Current LMST: %s"  % cur_sidereal(lon,lat).replace(",", ":"))
                print ("AZ: %f EL %f for equatorial coordinate: RA %f DEC %f" %
                    (t_az, t_el, t_ra, t_dec))
                rv = True
                break
            print ("Converging on AZ %f  EL %f CUR AZ %f CUR EL %f" % (t_az, t_el, cur_az, cur_el))
        #
        # Else t_az and t_el will just be what they were at the top of this function
        #
        else:
            t_az = t_ra
            t_el = t_dec

        #
        # Target would place us beyond limits
        #
        if (t_el < ELEVATION_LIMITS[0] or t_el > ELEVATION_LIMITS[1]):
            set_el_speed(0.0)
            limits = True

        if (t_az < AZIMUTH_LIMITS[0] or t_az > AZIMUTH_LIMITS[1]):
            set_az_speed(0.0)
            limits = True

        if (limits is True):
            break

        #
        # Get sensors
        #
        cur_el, cur_az = get_both_sensors()

        if (cur_el < 0.5 or cur_el > 89.0):
            limits = True
            break
        if (cur_az < 0.5 or cur_az > 356.75):
            limits = True
            break

        ltp = time.gmtime()
        lfp.write ("%02d,%02d,%02d,SLEW,%f,%f,%f,%f\n" % (ltp.tm_hour,
            ltp.tm_min, ltp.tm_sec, t_az, t_el, cur_az, cur_el))
        lfp.flush()

        #
        # This accounts for the fact that the two axes will not be
        #  finishing at the same time, and indeed, one may be very
        #  "close" at the start, but if we stop, it will drift further
        #  away.
        #
        if (abs(cur_el - t_el) >= serror*2.0):
            if (el_running is False):
                last_time_sensors = time.time()
            el_running = True

        if (abs(cur_az - t_az) >= serror*2.0):
            if (az_running is False):
                last_time_sensors = time.time()
            az_running = True

        #
        # We haved reached the object in elevation--zero speed
        #
        if (abs(cur_el - t_el) <= serror):
            set_el_speed(0.0)
            el_speed = 0.0
            el_running = False

        #
        # We have reached the object in azimuth--zero speed
        #
        if (abs(cur_az - t_az) <= serror):
            set_az_speed(0.0)
            az_speed = 0.0
            az_running = False

        #
        # Determine desired slew-rate based on relative distances on
        #  both axes, and the actual current sky rate estimate.
        #  The actual current sky-rate kicks in at the bottom of the slew-rate
        #  curve so that we're moving at least as fast as the sky is.
        #
        #
        slew_tuple = slew_rate(t_el, t_az, cur_el, cur_az,linear, final_el_rate, final_az_rate)

        #
        # Update commanded motor speed if desired rate is different from
        #   current rate
        #
        if ((el_running is True) and (el_speed != slew_tuple[0])):
            el_speed = slew_tuple[0]
            r = set_el_speed(el_speed)
            if (r != 0):
                print ("Problem during set_el_speed: %08X" % ctypes.c_uint(r).value)
                rv = False
                break

        if ((az_running is True) and (az_speed != slew_tuple[1])):
            az_speed = slew_tuple[1]
            r = set_az_speed(az_speed)
            if (r != 0):
                print ("Problem during set_az_speed: %08X" % ctypes.c_uint(r).value)
                rv = False
                break

        #
        # Wait PAUSE_TIME second between updates
        #
        time.sleep(ptime)

        #
        # Do a very simple sanity check.
        # If there's some commanded speed (xx_running is true),
        #  check to make sure there has been some amount of motion.
        #
        if ((time.time() - last_time_sensors) >= stime):
            last_time_sensors = time.time()
            if (az_running is True and az_speed > 0.0 and abs(cur_az-cmp_az) <= 0.0):
                print ("AZ: apparently not spinning or encoder failure")
                set_az_speed(0.0)
                rv = False
                break
            if (el_running is True and el_speed > 0.0 and abs(cur_el-cmp_el) <= 0.0):
                set_el_speed(0.0)
                print ("EL: apparently not spinning or encoder failure")
                rv = False
                break
            cmp_az = cur_az
            cmp_el = cur_el

    #
    # No matter how we exit from this loop, make sure things are "safe"
    #
    if (posonly is False):
        set_az_speed(0.0)
        set_el_speed(0.0)

    if (limits is True):
        print ("At least one axis limit exceeded--not proceeding")
        rv = False

    return rv

def gain(x,coeff):
    if (x < 1.0):
        return(x/coeff)
    else:
        return(x*coeff)

def sign(x):
    if (x < 0):
        sign = "1"
    else:
        sign = "0"
    return(sign)

#
# Rate-based (continuous) tracking
#


#
# Track target (assumes already recently moved-to)
#
# t_ra - target RA in decimal-float format
# t_dec - target DEC in decimal-float format
# lat - local geo latitude in decimal-float format
# lon - local geo longitude in decimal-float format
# elev - local elevation in decimal/float format
#
# Returns: True for success False otherwise
#
def track_continuous (t_ra, t_dec, lfp, body, args):

    #
    # Pull everything out of "args" that we need
    # Used to be individual calling parameters, but it got ungainly
    #
    lat = args.lat
    lon = args.lon
    elev = args.elev
    tracktime = args.tracking
    azoffset = args.azoffset
    eloffset = args.eloffset
    minterval = args.tinterval
    simulate = args.simulate
    gerror = args.gerror
    serror = args.serror
    nocorrect = args.nocorrect
    ctime = args.ctime
    
        
    sidt = cur_sidereal(lon,lat)
    sidt = sidt.split(",")
    sidh = float(sidt[0]) + float(sidt[1])/60.0 + float(sidt[2])/3600.0
    sidupper = sidh + (tracktime/3600.0)
        

    rv = True
    #
    # Prime ephem to know about our location and time
    #
    local = ephem.Observer()
    local.lat = math.radians(lat)
    local.lon = math.radians(lon)
    local.elevation = elev
    local.pressure = 0
    local.epoch = ephem.J2000
    
    #
    # Do intial compute on the target
    #  celestial coordinate
    #
    if (body is None):
        v = ephem.FixedBody()
        v._ra = to_ephem_coord(t_ra)
        v._dec = to_ephem_coord(t_dec)
    else:
        v = body
        t_ra = from_ephem_coord(str(v.ra))
        t_dec = from_ephem_coord(str(v.dec))

    #
    # Compute required rate right now
    #
    local.date = ephem.now()
    v.compute(local)
    initial_az = math.degrees(v.az)
    initial_el = math.degrees(v.alt)
    
    #
    # See if an initial position correction will help
    #
    
    #
    # First, set limits
    #
    if (simulate is False):
        set_az_acclimit(600)
        set_az_vellimit(75)
        set_el_acclimit(800)
        set_el_vellimit(150)
    
    el_in_motion = False
    az_in_motion = False

    
    #
    # Check desired-vs-actual
    # Do an angle move if necessary
    #
    
    #
    # Azimuth
    #
    if (simulate is False):
        now_el, now_az = get_both_sensors()
        if (abs(initial_az - now_az) > (serror*0.7)):
            az_in_motion = True
            move_az_angle(initial_az-now_az)
            print ("TRACKING: Initially adjusting azimuth by: %f" % (initial_az - now_az))
     
    #
    # Elevation
    #
    if (simulate is False):
        if (abs(initial_el - now_el) > (serror*0.7)):
            move_el_angle(initial_el-now_el)
            el_in_motion = True
            print ("TRACKING: Initially adjusting elevation by: %f" % (initial_el - now_el))
    
    #
    # Wait for those moves to complete
    #
    while (simulate is False):
        #
        # Issue motion waits as necessary
        #
        
        #
        # Elevation
        #
        if (el_in_motion):
            if (wait_el_move(0) != 0):
                el_in_motion = False
        #
        # Azimuth
        #
        if (az_in_motion):
            if (wait_az_move(0) != 0):
                az_in_motion = False
        #
        # Break if neither axis is in motion
        #
        if (not az_in_motion and not el_in_motion):
            break
        #
        # Go to sleep for just a little bit
        # We want to be ready to proceed to rate-based tracking
        #   as soon as possible!
        #
        time.sleep(0.1)
        
    #
    # Crude test to see if this tracking request crosses the singularity region
    #  near the Zenith.
    #
    geo_ra = from_ephem_coord("%s" % v.g_ra)
    geo_dec = from_ephem_coord("%s" % v.g_dec)
    if (abs(geo_dec-lat) < 1.5 and
        (geo_ra >= sidh and geo_ra <= sidupper)):
        print ("ERROR: requested tracking crosses Zenith singularity")
        rv = False
        return

    #
    # Compute position that will be required in "minterval" seconds
    #
    local.date = ephem.now() + (float(minterval)/86400.0)
    v.compute(local)
 
    later_az = math.degrees(v.az)
    later_el = math.degrees(v.alt)

    #
    # Derive required drive rate from two positions
    #
    el_rate = (later_el - initial_el) / float(minterval)
    az_rate = (later_az - initial_az) / float(minterval)

    #
    # We need to convert from degrees/second into RPM
    #
    el_rpm = round(dps_to_rpm(el_rate, ELEV_RATIO), 2)
    az_rpm = round(dps_to_rpm(az_rate, AZIM_RATIO), 2)


    if (simulate is False):
        #
        # Acceleration limits during tracking
        #
        set_az_acclimit(900)
        time.sleep(0.1)
        set_el_acclimit(900)

        #
        # Velocity limits during tracking
        #
        # Close to the Zenith, tracking rates become eye-watering
        #
        set_az_vellimit(gear_spin_max)
        time.sleep(0.1)
        set_el_vellimit(gear_spin_max)

    #
    # Mark our starting time
    #
    start_time = time.time()

    #
    # Smoothing for rate estimates
    #
    alpha = 0.3
    beta = 1.0 - alpha
    
    #
    # Smoothing for error estimates
    #
    calpha = 0.125
    cbeta = 1.0 - calpha
    
    prev_az_rpm = 9999.0
    prev_el_rpm = 9999.0

    el_rate_corr = 1.0
    az_rate_corr = 1.0
    
    #
    # Enter the control loop
    #
    last_correct_time = time.time()
    previous_el = -99.0
    previous_az = -99.0
    smoothed_el_rate_corr = 1.0
    smoothed_az_rate_corr = 1.0
    while True:
        
        #
        # Let motor server know we're alive
        #
        if (simulate is False):
            send_heartbeat()
        
        #
        # Looks like we're done
        #
        if ((time.time() - start_time) >= tracktime):
            if (simulate is False):
                set_az_speed(0.0)
                set_el_speed(0.0)
            break

        #
        # We know what initial rates we need
        #
        if (simulate is False):
            if (az_rpm != prev_az_rpm):
                r = set_az_speed(az_rpm)
                prev_az_rpm = az_rpm
            if (r != 0):
                print ("TRACK: problem setting speed %f  %08X" % (az_rpm, ctypes.c_uint(r).value))
                rv = False
                break
            time.sleep(0.1)
            if (el_rpm != prev_el_rpm):
                r = set_el_speed(el_rpm)
                prev_el_rpm = el_rpm
            if (r != 0):
                print ("TRACK: problem setting speed %f  %08X" % (el_rpm, ctypes.c_uint(r).value))
                rv = False
                break
        #
        # If the apparent required rate is much higher than astro motion would
        #  imply...
        #
        if (el_rpm > dps_to_rpm(6.0/60.0, ELEV_RATIO) or
            az_rpm > dps_to_rpm(18.5/60.0, AZIM_RATIO)):
            print ("TRACK: required rate much greater than expected EL: %f AZ: %f" %
                (dps(el_rpm, ELEV_RATIO), dps(az_rpm, AZIM_RATIO)))
            rv = False
            break

        #
        # Update the ephem sky coordinates
        #
        local.date = ephem.now()
        v.compute(local)
        t_az = math.degrees(v.az) + azoffset
        t_el = math.degrees(v.alt) + eloffset
        
        if(t_az < AZIMUTH_LIMITS[0] or t_az > AZIMUTH_LIMITS[1]):
            rv = False
            print ("Azimuth tracking limit exceeded (%f) halting tracking" % t_az)
            break
        if (t_el < ELEVATION_LIMITS[0] or t_el > ELEVATION_LIMITS[1]):
            rv = False
            print ("Elevation tracking limit exceeded (%f) halting tracking" % t_el)
            break

        #
        # Get our current actual position
        #
        if (simulate is False):
            cur_el, cur_az = get_both_sensors()
        else:
            cur_el = t_el * random.uniform(0.999,1.001)
            cur_az = t_az * random.uniform(0.999,1.001)

        ltp = time.gmtime()
        thestring = "%02d,%02d,%02d,TRACK,%13.9f,%13.9f,%f,%f,%f,%f,%f,%f,%f,%f\n" % (ltp.tm_hour,
            ltp.tm_min, ltp.tm_sec, t_az, t_el, cur_az, cur_el, az_rpm, el_rpm, az_rate, el_rate,
            smoothed_az_rate_corr, smoothed_el_rate_corr)
        lfp.write (thestring)
        lfp.flush()
        sys.stderr.write(thestring)

        if (cur_el < ELEVATION_LIMITS[0] or cur_el > ELEVATION_LIMITS[1]):
            print ("Elevation position limit exceeded (%f).  Halting tracking" % cur_el)
            rv = False
            break

        if (cur_az < AZIMUTH_LIMITS[0] or cur_az > AZIMUTH_LIMITS[1]):
            print ("Azimuth position limit exceeded (%f).  Halting tracking" % cur_az)
            rv = False
            break

        #
        # Because time.sleep() can occasionally be "off" by 1 second, we
        #  grab the actual times, and use THAT instead of "minterval" for
        #  division.
        #
        time.sleep(minterval)
        
        #
        # Grab the DJD in "local" before we update it
        #
        old_djd = local.date
        
        #
        # Update "local" and compute on it
        #
        local.date = ephem.now()
        new_djd = local.date
        v.compute(local)
        tmp_el = math.degrees(v.alt)
        tmp_az = math.degrees(v.az)
        
        #
        # Compute DJD time difference between two ephem computes
        #
        djd_seconds = new_djd - old_djd
        djd_seconds *= 86400.0
        
        #
        # Actual motion may be really really slow--like too slow to measure given sensor
        #   resolution, even over a several-second measurement interval.
        #
        # So, we only compute the correction required on a longer interval
        #
        now = time.time()
        if (nocorrect is False and (now - last_correct_time) >= ctime):
            timediff = now-last_correct_time
            last_correct_time = now
            ltp = time.gmtime()
            print ("%02d:%02d:%02d TRACK: Time to compute correction" % (
                ltp.tm_hour, ltp.tm_min, ltp.tm_sec))
            #
            # Grab actual position after sleeping
            #
            if (simulate is False):
                actual_el, actual_az = get_both_sensors()
            else:
                qconst = EQUANT
                rlist = [-qconst,qconst]
                q_tmp_el = float(int(tmp_el / qconst)) * qconst
                q_tmp_az = float(int(tmp_az / qconst)) * qconst
                if (random.randint(0,3) == 0):
                    actual_el = q_tmp_el + rlist[random.randint(0,1)]
                    actual_az = q_tmp_az + rlist[random.randint(0,1)]
                else:
                    actual_el = q_tmp_el
                    actual_az = q_tmp_az
 
            #
            # By default, rate correction is 1.0
            #
            el_rate_corr = 1.0
            
            if (abs(el_rpm) >= min_c_rpm(EQUANT,ctime,ELEV_RATIO) and previous_el >= 0.0):
                #
                # Compute the measured elevation axis rate
                #
                el_actual_rate = actual_el - previous_el
                
                #
                # Compute the measured vs commanded ratio
                #
                el_rate_ratio = abs(el_actual_rate / (timediff*el_rate))
                print ("el_rate_ratio %f" % el_rate_ratio)
                
                #
                # If we're apparently "out" by at least 2.5%, correct
                #
                if (el_rate_ratio > 1.025 or el_rate_ratio < 0.975):
                    el_rate_corr = 1.0 / el_rate_ratio
                if (el_rate_ratio > 1.25 or el_rate_ratio < 0.75):
                    print ("Warning: EL rate ratio suspiciously high %f %f %f" % (el_rate_ratio, el_rate, el_actual_rate))
                    if (el_rate_ratio > 1.0):
                        el_rate_ratio = 1.25
                    else:
                        el_rate_ratio = 0.75
                    el_rate_corr = 1.0 / el_rate_ratio
            #
            # Update smoothed
            #      
            smoothed_el_rate_corr = (calpha * el_rate_corr) + (cbeta * smoothed_el_rate_corr)
            
            #
            # Default azimuth correction is 1.0
            #
            az_rate_corr = 1.0
            
            if (abs(az_rpm) >= min_c_rpm(EQUANT, ctime, AZIM_RATIO) and previous_az >= 0.0):
                #
                # Compute the measured Azimuth axis rate
                #
                az_actual_rate = actual_az - previous_az
                
                #
                # Compute the measured vs commanded rate ratio
                #
                az_rate_ratio = abs(az_actual_rate / (timediff*az_rate))
                print ("AZ rate ratio: %f" % az_rate_ratio)
                
                #
                # If the ratio exceeds limits, apply correction
                #
                if (az_rate_ratio > 1.025 or az_rate_ratio < 0.975):
                    az_rate_corr = 1.0 / az_rate_ratio                
                    
                if (az_rate_ratio > 1.25 or az_rate_ratio < 0.75):
                    print ("Warning: AZ rate ratio suspiciously high %f %f %f" % (az_rate_ratio, az_rate, az_actual_rate))
                    if (az_rate_ratio > 1.0):
                        az_rate_ratio = 1.25
                    else:
                        az_rate_ratio = 0.75
                    az_rate_corr = 1.0 / az_rate_ratio
            
            #
            # Update smoothed
            #       
            smoothed_az_rate_corr = (calpha * az_rate_corr) + (cbeta * smoothed_az_rate_corr)
            
            print ("%02d:%02d:%02d TRACK: Computed corrections %f %f" % (
                ltp.tm_hour, ltp.tm_min, ltp.tm_sec, smoothed_az_rate_corr, smoothed_el_rate_corr))
            previous_el = actual_el
            previous_az = actual_az

        #
        # We've gone to sleep for a bit, compute new rates
        #
        el_inst_rate = (tmp_el - t_el) / djd_seconds
        az_inst_rate = (tmp_az - t_az) / djd_seconds
        
        #
        # Compute smoothed rate
        #
        el_rate = (alpha * el_inst_rate * smoothed_el_rate_corr) + (beta * el_rate)
        az_rate = (alpha * az_inst_rate * smoothed_az_rate_corr) + (beta * az_rate)

        #
        # Convert to motor-shaft RPM
        #
        el_rpm = round(dps_to_rpm(el_rate, ELEV_RATIO), 2)
        az_rpm = round(dps_to_rpm(az_rate, AZIM_RATIO), 2)
    #
    # No matter how we exit from this loop, make sure things are "safe"
    #
    if (simulate is False):
        set_az_speed(0.0)
        set_el_speed(0.0)

    return rv

def main():
    global rpc
    global rpc2
    global gear_spin_max
    global prop_limit

    planets = {"sun" : ephem.Sun(), "moon" : ephem.Moon(), "mercury" : ephem.Mercury(),
        "venus" : ephem.Venus(), "jupiter" : ephem.Jupiter(), "saturn" : ephem.Saturn(),
        "neptune" : ephem.Neptune(), "uranus" : ephem.Uranus(), "pluto" : ephem.Pluto()}


    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument ("--ra", type=float, default=None, help="RA of object")
    parser.add_argument ("--dec", type=float, default=None, help="DEC of object")
    parser.add_argument ("--tracking", type=float, default=0.0, help="How long to track")
    parser.add_argument ("--lat", type=float, default=45.3491, help="Local latitude")
    parser.add_argument ("--lon", type=float, default=-76.0413, help="Local longitude")
    parser.add_argument ("--elev", type=float, default=96.0, help="Local elevation (m)")
    parser.add_argument ("--planet", type=str, default=None, help="Planetary body")
    parser.add_argument ("--motorproxy", type=str, default="http://localhost:36036",
        help="XMLRPC Server for motors")
    parser.add_argument ("--sensorproxy", type=str, default="http://localhost:9090",
        help="XMLRPC Server for sensors")
    parser.add_argument ("--azoffset", type=float, default=0.0, help="Azimuth offset")
    parser.add_argument ("--eloffset", type=float, default=0.0, help="Elevation offset")
    parser.add_argument ("--absolute", action="store_true", default=False, help="Absolute position mode")
    parser.add_argument ("--posonly", action="store_true", default=False, help="Just give me the coordinates")
    parser.add_argument ("--pause", type=float, default=PAUSE_TIME, help="Loop pause time")
    parser.add_argument ("--sanity", type=float, default=SANITY_TIME, help="Sanity checking interval")
    parser.add_argument ("--linear", action="store_true", default=False, help="Linear rampdown")
    parser.add_argument ("--acclimit", type=int, default=ACC_LIMIT, help="Acceleration limit, RPM/SEC")
    parser.add_argument ("--speedlimit", type=float, default=GEAR_SPIN_MAX, help="Motor speed limit")
    parser.add_argument ("--proplimit", type=float, default=PROP_LIMIT, help="Proportional boundary")
    parser.add_argument ("--tinterval", type=float, default=10.0, help="Tracking update interval, seconds")
    parser.add_argument ("--trackonly", action="store_true", default=False, help="Only track, no slew")
    parser.add_argument ("--simulate", action="store_true", default=False, help="Simulate only, no motors or sensors")
    parser.add_argument ("--serverexit", action="store_true", default=False, help="Exit motor server when done")
    parser.add_argument ("--gerror", type=float, default=1.0, help="Gain value for error estimate in tracking")
    parser.add_argument ("--serror", type=float, default=SERROR, help="Allowable error target during slewing")
    parser.add_argument ("--galactic", action="store_true", default=False, help="RA/DEC are galactic long/lat")
    parser.add_argument ("--nocorrect", action="store_true", default=False, help="Disable corrections during tracking")
    parser.add_argument ("--ctime", type=float, default=30, help="Correction surveillance interval")
    args = parser.parse_args()

    gear_spin_max = args.speedlimit
    prop_limit = args.proplimit
    
    #
    # We talk to the motors (MotionServer) via XMLRPC
    #
    rpc = xml.ServerProxy(args.motorproxy)

    #
    # Similarly, we talk to the position sensors via XMLRPC
    #
    rpc2 = xml.ServerProxy(args.sensorproxy)

    #
    # Pick up targets
    #
    tra = args.ra
    tdec = args.dec

    body = None

    #
    # They've asked to track a planetary body
    #
    # The planetary ephemeris in pyEphem is out of date, so we use
    #  SkyField for planets
    #
    # We could use SkyField for everything, and maybe we will one day,
    #   but I think the coordinate transrorms in pyephem are still valid
    #
    if (args.planet is not None):
        planet = args.planet.lower()
        if (planet not in planets):
            print ("No such planet %s" % args.planet)
            return False
        else:
            body = planets[planet]
            body.compute(ephem.now())
            tra = from_ephem_coord(str(body.ra))
            tdec = from_ephem_coord(str(body.dec))
    if (args.galactic is True and args.planet is None):
        equ = ephem.Equatorial(ephem.Galactic(math.radians(args.ra), math.radians(args.dec)))
        #
        # Ephem keeps things in hour-angle (in radians)
        # We prefer decimalized hours-of-RA
        #
        tra = (math.degrees(equ.ra)/360.0)*24.0
        tdec = math.degrees(equ.dec)
        
    ltp = time.gmtime()
    ts = "%04d%02d%02d" % (ltp.tm_year, ltp.tm_mon, ltp.tm_mday)

    fp = open("%s-motion.csv"  % ts ,  "w")
    fp.write("TARGET: %f %f\n" % (tra, tdec))
    fp.flush()

    #
    # Set the global acceleration limits
    #
    if (args.simulate is False and args.posonly is False):
        set_el_acclimit(args.acclimit)
        time.sleep(0.5)
        set_az_acclimit(args.acclimit)

    if (args.trackonly is False):
        try:
            if (moveto(tra, tdec, args.lat, args.lon, args.elev, args.azoffset, args.eloffset,
                fp, args.absolute, args.posonly, body, args.pause, args.sanity, args.linear, args.serror) is not True):
                print ("Problem encountered during slew--exiting prior to tracking")
                if (args.simulate is False and args.posonly is False):
                    set_el_speed(0.0)
                    set_az_speed(0.0)
                    restore_limits()
                if (args.posonly is False and args.simulate is False):
                    my_exit(1,args.serverexit)
        except:
            print ("Exception raised in moveto()--setting motors to zero speed")
            if (args.simulate is False and args.posonly is False):
               set_el_speed(0.0)
               set_az_speed(0.0)
               restore_limits()
            print ("Traceback--")
            print (traceback.format_exc())
            if (args.simulate is False and args.posonly is False):
                my_exit(1,args.serverexit)

    if (args.absolute is False and (args.tracking > 0)):
        try:
            if (track_continuous(tra, tdec, fp, body, args)
                is not True):
                print ("Problem encountered while tracking.  Done tracking")
                if (args.simulate is False):
                    set_el_speed(0.0)
                    set_az_speed(0.0)
                    restore_limits()
                my_exit(1,args.serverexit)
        except:
            print ("Exception raised in track() -- setting motors to zero speed")
            if (args.simulate is False):
                set_el_speed(0.0)
                set_az_speed(0.0)
                restore_limits()
            print ("Traceback--")
            print(traceback.format_exc())
            my_exit(1,args.serverexit)
    fp.close()
    if (args.simulate is False and args.posonly is False):
        my_exit(0,args.serverexit)

if __name__ == '__main__':
    main()
