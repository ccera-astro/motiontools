#!/usr/bin/python3
import math
import os
import sys
import ephem
import datetime
import time
import xmlrpc.client as xml
import argparse

ENCODER_BITS=14
ENCODER_RESOLUTION=360.0/(2**ENCODER_BITS)

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
# Ratio of the gearbox, up to the output pinion
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
GEAR_SPIN_MAX = 1600.0

#
# An angular rate that is manageable by both axes
#
ELEV_SLEW_RATE_MAX = (GEAR_SPIN_MAX / ELEV_RATIO) * 360.0
AZ_SLEW_RATE_MAX = (GEAR_SPIN_MAX / AZIM_RATIO) * 360.0

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
    #print ("COnverting from ephem to decimal %s" % coord)
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
QUANTUM=15
def proportional_speed(pdiff,lim):
    v1 = lim-pdiff
    v2 = GEAR_SPIN_MAX/(1+1.35*(v1*v1))
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
#
PROP_LIMIT = 2.5

#
# Determine desired slew rate, based on axis position offsets from target
#
# Basically, as we get closer, we slow down, to prevent over-shoot
#
# targ_el - target elevation in decimal-float format
# targ_az - target azimuth in decimal-float format
# cur_el - current elevation in decimal-float format
# cur_az - current azimuth in decimal-float format
#
# Returns:  (el_slew, az_slew)   slew speeds in decimal-float
#
def slew_rate(targ_el, targ_az, cur_el, cur_az):
    
    #
    # Compute for elevation
    #
    if (abs(targ_el - cur_el) > PROP_LIMIT):
        el_slew = GEAR_SPIN_MAX
    else:
        el_slew = proportional_speed(abs(targ_el - cur_el), PROP_LIMIT)
    #
    # Compute for azimuth
    # Azimuth moves faster than elevation, so adjust the resulting curve
    #  a bit.
    #
    if (abs(targ_az - cur_az) > PROP_LIMIT*1.5):
        az_slew = GEAR_SPIN_MAX
    else:
        az_slew = proportional_speed(abs(targ_az - cur_az), PROP_LIMIT*1.5)
            
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
PAUSE_TIME = 1.0

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
def moveto(t_ra, t_dec, lat, lon, elev, azoffset, eloffset, lfp, absolute, posonly, body):

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
    v.compute(local)
    #print ("_RA %s _DEC %s" % (v._ra, v._dec))
    #print ("G_RA %s G_DEC %s" % (v.g_ra, v.g_dec))
    #print ("A_RA %s A_DEC %s" % (v.a_ra, v.a_dec))
    #print ("RA %s DEC %s" % (v.ra ,v.dec))
    #print (str(v))
    #print (str(local))
    
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
    # Init our weirdness counter
    #
    weird_count = 0
    
    #
    # Set return value
    #
    rv = True
    
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
        # Update the ephem sky coordinates
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
        else:
            t_az = t_ra
            t_el = t_dec
        
        if (t_el < ELEVATION_LIMITS[0] or t_el > ELEVATION_LIMITS[1]):
            set_el_speed(0.0)
            limits = True
            
        if (t_az < AZIMUTH_LIMITS[0] or t_az > AZIMUTH_LIMITS[1]):
            set_az_speed(0.0)
            limits = True
        
        if (limits == True):
            break
        
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
        if (abs(cur_el - t_el) >= 0.15):
            el_running = True
        if (abs(cur_az - t_az) >= 0.15):
            az_running = True
            
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
        # Wait PAUSE_TIME second between updates
        #
        ptime = PAUSE_TIME
        
        if (dps(el_speed, ELEV_RATIO) < ENCODER_RESOLUTION*3.0):
            ptime = PAUSE_TIME/2.0
        if (dps(az_speed, AZIM_RATIO) < ENCODER_RESOLUTION*3.0):
            ptime = PAUSE_TIME/2.0

        time.sleep(ptime)
        
        #
        # Hmm, despite there being a 2-second pause the relevant axis
        #   hasn't moved.  Declare some "weirdness"
        #
        new_positions = get_both_sensors()
        new_el = new_positions[0]
        new_az = new_positions[1]
        
        #
        # Compute expected and actual ELEV axis motion
        #
        expected_el = dps(el_speed, ELEV_RATIO)
        expected_el_range = (expected_el * 0.30, expected_el * 3.0)
        actual_el_motion = abs(new_el - cur_el) / ptime

        #
        # Compute expected and actual AZIM axis motion
        #
        expected_az = dps(az_speed, AZIM_RATIO)
        expected_az_range = (expected_az * 0.3, expected_az * 3.0)
        actual_az_motion = abs(new_az - cur_az) / ptime
        
        #
        # Is the actual ELEV speed somewhere in the rough range of expected speed?
        #
        if ((el_running == True) and (actual_el_motion <= expected_el_range[0] or
            actual_el_motion >= expected_el_range[1])):
            #
            # Special case--if we're moving really slow, the encoder may not have
            #  updated in its lowest bit position.
            #
            if (actual_el_motion != 0.0):
                print ("EL MOTION FAULT: actual %f range %f-%f commanded %f" % (actual_el_motion,
                    expected_el_range[0], expected_el_range[1], dps(el_speed, ELEV_RATIO)))
                rv = False
                set_el_speed(0.0)
                set_az_speed(0.0)
                break
            elif(dps(el_speed, ELEV_RATIO) > ENCODER_RESOLUTION*2.25):
                print ("EL MOTION FAULT: encoder or motor system not moving")
                set_el_speed(0.0)
                set_az_speed(0.0)
                rv = False
                
        #
        # Is the actual AZIM speed somewhere in the rough range of expected speed?
        #
        if ((az_running == True) and (actual_az_motion <= expected_az_range[0] or
            actual_az_motion >= expected_az_range[1])):
            
            #
            # Special case here -- if the actual motion is 0.0, it's probably because
            #   we're moving at a rate that is too slow to see an update.
            #
            if (actual_az_motion != 0.0):
                print ("AZ MOTION FAULT: actual %f range %f-%f commanded %f" % (actual_az_motion,
                    expected_az_range[0], expected_az_range[1], dps(az_speed, AZIM_RATIO)))
                set_el_speed(0.0)
                set_az_speed(0.0)
                rv = False
                break
            elif (dps(az_speed, AZIM_RATIO) > ENCODER_RESOLUTION*2.25):
                print ("AZ MOTION FAULT: encoder or motor system not moving")
                set_el_speed(0.0)
                set_az_speed(0.0)
                rv = False
                break
        
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

"""
#
# RATE-BASED TRACKING
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
def track(t_ra, t_dec, lat, lon, elev, tracktime, azoffset, eloffset, lfp):

    #
    # Measurement interval, seconds
    #
    minterval = 10.0
    
    #
    # Prime ephem to know about our location and time
    #
    local = ephem.Observer()
    local.lat = to_ephem_coord(lat)
    local.lon = to_ephem_coord(lon)
    local.elevation = elev
    local.pressure = 0
    
    #
    # Do intial compute on the target
    #  celestial coordinate
    #
    local.date = ephem.now()
    v = ephem.FixedBody()
    v._ra = to_ephem_coord(t_ra)
    v._dec = to_ephem_coord(t_dec)
    
    #
    # Keep track of current speed
    #
    el_speed = 0.0
    az_speed = 0.0
    
    #
    # Init our weirdness counter
    #
    weird_count = 0
    
    #
    # Set return value
    #
    rv = True
    
    #
    # Forever, until we're done tracking
    #
    start_time = time.time()
 
    v.compute(local)
    last_t_el = from_ephem_coord(v.alt) + eloffset
    last_t_az = from_ephem_coord(v.az) + azoffset
    last_el = get_el_sensor()
    last_az = get_az_sensor()
    
    #
    # Set initial speed
    #
    speeds = track_rate(t_dec)
    el_speed = speeds[0]
    az_speed = speeds[1]
    
    #
    # If we're past the meridian, we're heading downwards in elevation
    #
    if (last_az > 180.0):
        el_speed = el_speed * -1.0

    #
    # Start the ball rolling...
    #  at the initial speed, which will get adjusted 10 seconds from now
    #
    if (set_az_speed(az_speed) != 0):
        return False
        
    if (set_el_speed(el_speed) != 0):
        return False
        
    time.sleep (minterval)
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
        cur_el = get_el_sensor()
        cur_az = get_az_sensor()
        
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
        
        #
        # Determine actual machine rates
        #
        rate_el = (cur_el - last_el) / minterval
        rate_az = (cur_az - last_az) / minterval
        
        last_el = cur_el
        last_az = cur_az
        
        #
        # Determine desired rates (from target ephemeris)
        #
        expected_rate_el = (t_el - last_t_el) / minterval
        last_t_el = t_el

        expected_rate_az = (t_az - last_t_az) / minterval
        last_t_az = t_az
        
        #
        # Compute ratios
        # Use to adjust current motor speed
        #
        # Note that we're computing rates based on deg/second,
        #   but motor speed is in RPM.  That's fine, because the
        #   ratio adjust on el_speed/az_speed is linear
        #
        el_ratio = rate_el / expected_rate_el
        el_speed = el_speed / el_ratio

        az_ratio = rate_az / expected_rate_az
        az_speed = az_speed / az_ratio

        if (abs(el_ratio) > 3.0 or abs(el_ratio) < 0.3333):
            print ("Tracking speeds not converging!")
            break
        if (abs(az_ratio) > 3.0 or abs(el_ratio) < 0.3333):
            print ("Tracking speeds not converging!")
            break
        
        #
        # Update speeds if reasonable
        #
        if (abs(el_ratio) < 0.985 or abs(el_ratio) > 1.025):
            r = set_el_speed(el_speed)
            if (r != 0):
                print ("Problem during set_el_speed; %08X" % r)
                rv = False
                break
        
        if (abs(az_ratio) < 0.985 or abs(az_ratio) > 1.025):
            r = set_az_speed (az_speed)
            if (r != 0):
                print ("Problem during set_az_speed: %08X" % r)
                rv = False
                break
        
        #
        # We have a measurement interval for determining what the
        #   current angular rate of the machine is, and what is
        #   actually required based on the ephemeris calculations.
        #
        time.sleep(minterval)
    #
    # No matter how we exit from this loop, make sure things are "safe"
    #
    set_az_speed(0.0)
    set_el_speed(0.0)
    
    return rv
"""

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
    minterval = 8
    
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


from skyfield.api import load

def main():
    global rpc
    global rpc2
    planets = {"sun" : ephem.Sun(), "moon" : ephem.Moon(), "mercury" : ephem.Mercury(),
        "venus" : ephem.Venus(), "jupiter" : ephem.Jupiter(), "saturn" : ephem.Saturn(),
        "neptune" : ephem.Neptune(), "uranus" : ephem.Uranus(), "pluto" : ephem.Pluto()}

    
    parser = argparse.ArgumentParser()
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
    args = parser.parse_args()
    
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
        """
        #
        # Use SkyField to determine RA/DEC of planet
        #
        # Create a timescale and ask the current time.
        ts = load.timescale()
        t = ts.now()

        # Load the JPL ephemeris DE421 (covers 1900-2050).
        planets = load('de421.bsp')
        earth, planet = planets['earth'], planets[args.planet]

        # What's the position of planet, viewed from Earth?
        astrometric = earth.at(t).observe(planet)
        ra, dec, distance = astrometric.radec()

        #
        # Set our target ra/dec accordingly--overriding anything in
        #  args.ra and args.dec
        #
        tra = float(ra.hours)
        tdec = float(dec.degrees)
        if (args.posonly):
            print ("Position for object %s is RA: %f DEC: %f" %
                (args.planet, tra, tdec))
            return (True)
        """
        planet = args.planet.lower()
        if (planet not in planets):
            print ("No such planet %s" % args.planet)
            return False
        else:
            body = planets[planet]
            body.compute(ephem.now())
            print ("body: %s %s" % (str(body.ra), str(body.dec)))
            tra = from_ephem_coord(str(body.ra))
            tdec = from_ephem_coord(str(body.dec))

    ltp = time.gmtime()
    ts = "%04d%02d%02d%02d%02d" % (ltp.tm_year, ltp.tm_mon, ltp.tm_mday,
        ltp.tm_hour, ltp.tm_min)

    fp = open("%s-motion.csv"  % ts ,  "w")
    fp.write("TARGET: %f %f\n" % (tra, tdec))
    fp.flush()
        
    if (moveto(tra, tdec, args.lat, args.lon, args.elev, args.azoffset, args.eloffset,
        fp, args.absolute, args.posonly, body) != True):
        print ("Problem encountered--exiting prior to tracking")
        exit(1)
    
    if (args.absolute == False and args.tracking > 0):
        if (track(tra, tdec, args.lat, args.lon, args.elev, args.tracking, args.azoffset, args.eloffset,
            fp, body)
            != True):
            print ("Problem encountered while tracking.  Done tracking")
            exit(1)
    fp.close()

if __name__ == '__main__':
    main()
