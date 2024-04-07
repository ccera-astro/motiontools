#!/usr/bin/python3
import math
import os
import sys
import ephem
import datetime
import time
import xmlrpc.client as xml
import argparse
import traceback

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
ELEVATION_LIMITS = (0.5,91.5)
AZIMUTH_LIMITS = (1.0,359.0)

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

#
# Motor server interface
#
def set_az_speed(spd):
    global rpc
    return rpc.Move(1,spd*AZ_SIGN)

def set_el_speed(spd):
    global rpc
    return rpc.Move(0,spd*ELEV_SIGN)
    
def query_el_torque():
    global rpc
    return rpc.QueryTorque(0)

def query_az_torque():
    global rpc
    return rpc.QueryTorque(1)

def move_az_angle(angle):
    global rpc
    
    shaft_angle = angle * AZ_RATIO
    return rpc.AngleMove(1,float(shaft_angle*AZ_SIGN))

def move_el_angle(angle):
    global rpc
    shaft_angle = angle * ELEV_RATIO
    return rpc.AngleMove(0,float(shaft_angle*ELEV_SIGN))

def set_el_acclimit(limit):
    global rpc
    return rpc.AccLimit(0,limit)

def set_az_acclimit(limit):
    global rpc
    return rpc.AccLimit(1,limit)

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
    return (rpc2.query_both_axes())

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
    if (linear == False):
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

#
# We start going into proportional control at this limit in offset between
#  the current and target position (degrees)
# Command line can change this
#
PROP_LIMIT = 2.75
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
def slew_rate(targ_el, targ_az, cur_el, cur_az, linear):
    global gear_spin_max
    global prop_limit
    #
    # Compute for elevation
    #
    if (abs(targ_el - cur_el) > prop_limit):
        el_slew = gear_spin_max
    else:
        el_slew = proportional_speed(abs(targ_el - cur_el), prop_limit, linear)
    #
    # Compute for azimuth
    # Azimuth moves faster than elevation, so adjust the resulting curve
    #  a bit.
    #
    if (abs(targ_az - cur_az) > prop_limit*1.25):
        az_slew = gear_spin_max
    else:
        az_slew = proportional_speed(abs(targ_az - cur_az), prop_limit*1.25, linear)
            
    #
    # Adjust sign
    #
    if (targ_el < cur_el):
        el_slew *= -1.0
    if (targ_az < cur_az):
        az_slew *= -1.0

    return ((el_slew,az_slew))

#
# Pause time between slewing-rate updates
#
# Can be changed by command-line parameter
#
PAUSE_TIME = 0.5

#
# Sanity-checking interval
#
# Can be changed by command-line parameter
#
SANITY_TIME = 5.0

#
# Move dish to target
#
# t_ra - target RA in decimal-float format
# t_dec - target DEC in decimal-float format
# lat - local geo latitude in decimal-float format
# lon - local geo longitude in decimal-float format
# elev - local elevation in decimal/float format  
#
# Returns: True for success False otherwise
#
def moveto(t_ra, t_dec, lat, lon, elev, azoffset, eloffset, lfp, absolute, posonly, body,
    ptime, stime, linear):

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
    if (body == None):
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
    #   (every 5 seconds), to check that if there's a non-zero commanded
    #   motor speed, there is at least *some* motion over that interval.
    #
    last_time_sensors = time.time()
    cmp_axes = get_both_sensors()
    cmp_el = cmp_axes[0]
    cmp_az = cmp_exes[1]
    time.sleep(ptime)
    
    #
    # Forever, until we zero-in on the target
    #
    while True:
        limits = False
        #
        # Looks like down below, we have stopped motion on both
        #  axes--we exit this loop when this happens
        #
        if (az_running == False and el_running == False):
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
        if (absolute == False):
            local.date = ephem.now()
            v.compute(local)
            t_az = from_ephem_coord("%s" % v.az) + azoffset
            t_el = from_ephem_coord("%s" % v.alt) + eloffset
            if (posonly):
                print ("Current LMST: %s"  % cur_sidereal(lon,lat).replace(",", ":"))
                print ("AZ: %f EL %f for equatorial coordinate: RA %f DEC %f" %
                    (t_az, t_el, t_ra, t_dec))
                rv = False
                break
            print ("Converging on AZ %f  EL %f" % (t_az, t_el))
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
        
        if (limits == True):
            break
        
        #
        # Get sensors
        #
        axes = get_both_sensors()
        cur_el = axes[0]
        cur_az = axes[1]
        
        if (cur_el < 0.0 or cur_el > 91.0):
            limits = True
            break
        if (cur_az < 1.0 or cur_az > 359.5):
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
        if (abs(cur_el - t_el) >= 0.14):
            el_running = True

        if (abs(cur_az - t_az) >= 0.14):
            az_running = True
         
        #
        # We haved reached the object in elevation--zero speed
        #
        if (abs(cur_el - t_el) <= 0.07):
            set_el_speed(0.0)
            el_speed = 0.0
            el_running = False
        
        #
        # We have reached the object in azimuth--zero speed
        #
        if (abs(cur_az - t_az) <= 0.07):
            set_az_speed(0.0)
            az_speed = 0.0
            az_running = False
            
        #
        # Determine desired slew-rate based on relative distances on
        #  both axes
        #
        slew_tuple = slew_rate(t_el, t_az, cur_el, cur_az)
        
        #
        # Update commanded motor speed if desired rate is different from
        #   current rate
        #
        if ((el_running == True) and (el_speed != slew_tuple[0])):
            el_speed = slew_tuple[0]
            r = set_el_speed(el_speed)
            if (r != 0):
                print ("Problem during set_el_speed: %08X" % r)
                rv = False
                break
                
        if ((az_running == True) and (az_speed != slew_tuple[1])):
            az_speed = slew_tuple[1]
            r = set_az_speed(az_speed)
            if (r != 0):
                print ("Problem during set_az_speed: %08X" % r)
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
            if (az_running == True and az_speed > 0.0 and abs(cur_az-cmp_az) <= 0.0):
                print ("AZ: apparently not spinning or encoder failure")
                set_az_speed(0.0)
                rv = False
                break
            if (el_running == True and el_speed > 0.0 and abs(cur_el-cmp_el) <= 0.0):
                set_el_speed(0.0)
                print ("EL: apparently not spinning or encoder failure")
                rv = False
                break
            cmp_az = cur_az
            cmp_el = cur_el
            
        
    #
    # No matter how we exit from this loop, make sure things are "safe"
    #
    if (posonly == False):
        set_az_speed(0.0)
        set_el_speed(0.0)
    
    if (limits == True):
        print ("At least one axis limit exceeded--not proceeding")
        rv = False
    
    return rv

#
# Position-based (stuttered) tracking
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
def track(t_ra, t_dec, lat, lon, elev, tracktime, azoffset, eloffset, lfp, body):

    #
    # Measurement interval, seconds
    #
    minterval = 7.5
    
    #
    # Prime ephem to know about our location and time
    #
    local = ephem.Observer()
    local.lat = to_ephem_coord(lat)
    local.lon = to_ephem_coord(lon)
    local.elevation = elev
    local.pressure = 0
    local.epoch = ephem.J2000
    
    #
    # Do intial compute on the target
    #  celestial coordinate
    #
    local.date = ephem.now()
    if (body == None):
        v = ephem.FixedBody()
        v._ra = to_ephem_coord(t_ra)
        v._dec = to_ephem_coord(t_dec)
    else:
        v = body
        t_ra = from_ephem_coord(str(v.ra))
        t_dec = from_ephem_coord(str(v.dec))
    
    #
    # Mark our starting time
    #
    start_time = time.time()
    
    while True:
        #
        # Looks like we're done
        #
        if ((time.time() - start_time) >= tracktime):
            set_az_speed(0.0)
            set_el_speed(0.0)
            break
        
        #
        # Update the ephem sky coordinates
        #
        local.date = ephem.now()
        v.compute(local)
        t_az = from_ephem_coord("%s" % v.az) + azoffset
        t_el = from_ephem_coord("%s" % v.alt) + eloffset
        
        #
        # Get our current actual position
        #
        axes = get_both_sensors()
        cur_el = axes[0]
        cur_az = axes[1]
        
        #
        # If elevation has drifted enough, move through computed angle
        #
        if (abs(cur_el - t_el) >= 0.1):
            move_el_angle(t_el-cur_el)
        
        #
        # If azimuth has drifted enough, move through computed angle
        #
        if (abs(cur_az - t_az) >= 0.1):
            move_az_angle(t_az-cur_az)
        
        ltp = time.gmtime()
        lfp.write ("%02d,%02d,%02d,TRACK,%f,%f,%f,%f\n" % (ltp.tm_hour,
            ltp.tm_min, ltp.tm_sec, t_az, t_el, cur_az, cur_el))
        lfp.flush()
        
        if (cur_el < ELEVATION_LIMITS[0] or cur_el > ELEVATION_LIMITS[1]):
            print ("Elevation position limit exceeded (%f).  Halting tracking" % cur_el)
            rv = False
            break
        
        if (cur_az < AZIMUTH_LIMITS[0] or cur_az > AZIMUTH_LIMITS[1]):
            print ("Azimuth position limit exceeded (%f).  Halting tracking" % cur_az)
            rv = False
            break

        time.sleep(minterval)
    #
    # No matter how we exit from this loop, make sure things are "safe"
    #
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
    args = parser.parse_args()
    
    gear_spin_max = args.speedlimit
    
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
    if (args.planet != None):
        planet = args.planet.lower()
        if (planet not in planets):
            print ("No such planet %s" % args.planet)
            return False
        else:
            body = planets[planet]
            body.compute(ephem.now())
            #print ("body: %s %s" % (str(body.ra), str(body.dec)))
            tra = from_ephem_coord(str(body.ra))
            tdec = from_ephem_coord(str(body.dec))

    ltp = time.gmtime()
    ts = "%04d%02d%02d%02d%02d" % (ltp.tm_year, ltp.tm_mon, ltp.tm_mday,
        ltp.tm_hour, ltp.tm_min)

    fp = open("%s-motion.csv"  % ts ,  "w")
    fp.write("TARGET: %f %f\n" % (tra, tdec))
    fp.flush()
    
    #
    # Set the global acceleration limits
    #
    set_el_acclimit(args.acclimit)
    time.sleep(0.5)
    set_az_acclimit(args.acclimit)
    
    try:    
        if (moveto(tra, tdec, args.lat, args.lon, args.elev, args.azoffset, args.eloffset,
            fp, args.absolute, args.posonly, body, args.pause, args.sanity, args.linear) != True):
            print ("Problem encountered--exiting prior to tracking")
            exit(1)
    except Exception:
        print ("Exception raised in moveto()--setting motors to zero speed")
        set_el_speed(0.0)
        set_az_speed(0.0)
        print ("Traceback--")
        print (traceback.format_exc())
        exit(1)
        
    
    if (args.absolute == False and args.tracking > 0):
        try:
            if (track(tra, tdec, args.lat, args.lon, args.elev, args.tracking, args.azoffset, args.eloffset,
                fp, body)
                != True):
                print ("Problem encountered while tracking.  Done tracking")
                exit(1)
        except Exception:
            print ("Exception raised in track() -- setting motors to zero speed")
            set_el_speed(0.0)
            set_az_speed(0.0)
            print ("Traceback--")
            print(traceback.format_exc())
            exit(1)
    fp.close()

if __name__ == '__main__':
    main()
