#!/usr/bin/python3
import math
import os
import sys
import ephem
import datetime
import time
import xmlrpc.client as xml
import argparse
import minimalmodbus as mb

#
# Establish some constants
#
LATITUDE = 45.3491
LONGITUDE = -76.0413
ELEVATION = 70.00

#
# Pre-reduction ahead of the main gearboxes
#
ELEV_PREFIX = 5.0
AZ_PREFIX = 7.5

#
# Total ratios from the PREFIX gearing, and the main gearing
#
ELEV_RATIO = ELEV_PREFIX * (500.47 * 16)
AZIM_RATIO = AZ_PREFIX * (500.47 * 10)

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
AZIMUTH_LIMITS = (0.5,359.5)


#
# A rate that is manageable by both axes
#
SLEW_RATE_MAX = 21.0


def set_az_speed(spd):
    global rpc
    rpc.Move(1,spd*AZ_SIGN)
    return True

def set_el_speed(spd):
    global rpc
    rpc.Move(0,spd*EL_SIGN)
    return True


#
# The sensor code may move into a separate XMLRPC server at some point,
#  so that other parts of our overall system at the observatory can
#  get at it.
#
elSensor = None
azSensor = None
sensorFailed = False
SENSORS_PORT = '/dev/ttyUSB0'
SENSOR_BITS = 2**14

def init_sensor_system():
    global elSensor
    global azSensor
    
    if (elSensor == None or azSensor == None):
        try:
            elSensor = mb.Instrument(SENSORS_PORT, 1)
            azSensor = mb.Instrument(SENSORS_PORT, 2)
            elSensor.serial.baudrate = 9600
        except:
            elSensor = None
            azSensor = None
            sensorFailed = True
        
#
# Functions for position sensors
#
def get_el_sensor():
    global elSensor
    init_sensor_system()
    try:
        r = elSensor.read_register(0, 0)
        return (float(r)/float(SENSOR_BITS))*360.0
    except:
        return -1000.0

def get_az_sensor():
    init_sensor_system()
    global azSensor
    try:
        r = azSensor.read_register(0, 0)
        return (float(r)/float(SENSOR_BITS))*360.0
    except:
        return -1000.0

#
# Take a decimal-degrees coordinate, and transform to the HH:MM:SS
#   that ephem uses
#
def to_ephem_coord(decimal):
    longstr = "%02d" % int(decimal)
    longstr = longstr + ":"
    decimal = abs(decimal)
    frac = decimal - int(decimal)
    frac *= 60
    mins = int(frac)
    longstr += "%02d" % mins
    longstr += ":00"
    return longstr

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
    # We want to turn our degrees/min MAX slew rate into a motor RPM
    #
    max_slew = SLEW_RATE_MAX/360.0
    
    #
    # Adjust for the gearing on the two motors
    #
    el_slew = max_slew * ELEV_RATIO
    az_slew = max_slew * AZIM_RATIO
    
    #
    # Adjust az_slew/el_slew based on difference between target and current
    #
    if (abs(targ_el-cur_el) <= 2.5):
        el_slew = el_slew / 2.0
    if (abs(targ_el-cur_el) <= 1.25):
        el_slew = el_slew / 2.0
    if (abs(targ_el-cur_el) <= 0.625):
        el_slew = el_slew / 3.0
    if (abs(targ_el-cur_el) <= 0.3125):
        el_slew = el_slew / 2.0

    if (abs(targ_az-cur_az) <= 2.5):
        az_slew = az_slew / 2.0
    if (abs(targ_az-cur_az) <= 1.25):
        az_slew = az_slew / 2.0
    if (abs(targ_az-cur_az) <= 0.625):
        az_slew = az_slew / 3.0
    if (abs(targ_az-cur_az) <= 0.3125):
        az_slew = az_slew / 2.0

    #
    # Adjust sign
    #
    if (targ_el < cur_el):
        el_slew *= -1.0
    if (targ_az < cur_az):
        az_slew *= -1.0

    return ((el_slew,az_slew))

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
def moveto(t_ra, t_dec, lat, lon, elev):

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
        local.date = ephem.now()
        v.compute(local)
        t_az = from_ephem_coord("%s" % v.az)
        t_el = from_ephem_coord("%s" % v.alt)
        
        if (t_el < ELEVATION_LIMITS[0] or t_el > ELEVATION_LIMITS[1]):
            set_el_speed(0.0)
            limits = True
            
        if (t_az < AZIMUTH_LIMITS[0] or t_az > AZIMUTH_LIMITS[1]):
            set_az_speed(0.0)
            limits = True
        
        if (limits == True):
            break
            
        cur_el = get_el_sensor()
        cur_az = get_az_sensor()
        
        #
        # This accounts for the fact that the two axes will not be
        #  finishing at the same time, and indeed, one may be very
        #  "close" at the start, but if we stop, it will drift further
        #  away.
        #
        if (abs(cur_el - t_el) > 0.05):
            el_running = True
        if (abs(cur_az - t_az) > 0.05):
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
        if (el_running == True and el_speed != slew_tuple[0]):
            el_speed = slew_tuple[0]
            set_el_speed(el_speed)
            
        if (al_running == True  and az_speed != slew_tuple[1]):
            az_speed = slew_tuple[1]
            set_az_speed(az_speed)
        
        #
        # We haved reached the object in elevation--zero speed
        #
        if (abs(cur_el-t_el) < 0.05):
            set_el_speed(0.0)
            el_speed = 0.0
            el_running = False
        
        #
        # We have reached the object in azimuth--zero speed
        #
        if (abs(cur_az - t_az) < 0.05):
            set_az_speed(0.0)
            el_speed = 0.0
            az_running = False
            
        #
        # Wait 1 second between updates
        #
        time.sleep(2)
        
        #
        # Hmm, despite there being a 2-second pause the relevant axis
        #   hasn't moved.  Declare some "weirdness"
        #
        if (el_running == True and (abs(cur_el-get_el_sensor()) < 0.025)):
            weird_count += 1

        if (el_running == True and abs(cur_az-get_az_sensor()) < 0.025):
            weird_count += 1
        
        if (weird_count >= 5):
            print( "Axis not moving!!")
            set_az_speed(0.0)
            set_el_speed(0.0)
            rv = False
            break
    #
    # No matter how we exit from this loop, make sure things are "safe"
    #
    set_az_speed(0.0)
    set_el_speed(0.0)
    
    if (limits == True):
        print ("At least one axis limit exceeded--not proceeding")
        rv = False
    
    return rv

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
def track(t_ra, t_dec, lat, lon, elev, tracktime):

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
    last_t_el = from_ephem_coord(v.alt)
    last_t_az = from_ephem_coord(v.az)
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

    set_az_speed(az_speed)
    set_el_speed(el_speed)

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
        t_az = from_ephem_coord("%s" % v.az)
        t_el = from_ephem_coord("%s" % v.alt)
        
        #
        # Get our current actual position
        #
        cur_el = get_el_sensor()
        cur_az = get_az_sensor()
        
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
        if (abs(el_ratio) < 0.98 or abs(el_ratio) > 1.02):
            set_el_speed(el_speed)
        
        if (abs(az_ratio) < 0.98 or abs(az_ratio) > 1.02):
            set_az_speed(az_speed)
        
        
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
    rpc = xml.ServerProxy("http://localhost:36036")
    
    parser = argparse.ArgumentParser()
    parser.add_argument ("--ra", type=float, default=None, help="RA of object")
    parser.add_argument ("--dec", type=float, default=None, help="DEC of object")
    parser.add_argument ("--tracking", type=float, default=0.0, help="How long to track")
    parser.add_argument ("--lat", type=float, default=45.3497, help="Local latitude")
    parser.add_argument ("--lon", type=float, default=-76.0559, help="Local longitude")
    parser.add_argument ("--elev", type=float, default=96.0, help="Local elevation (m)")
    parser.add_argument ("--planet", type=str, default=None, help="Planetary body")
    
    args = parser.parse_args()
    
    #
    # Pick up targets
    #
    tra = args.ra
    tdec = args.dec
    
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
        
    if (moveto(tra, tdec, args.lat, args.lon, args.elev) != True):
        print ("Problem encountered--exiting prior to tracking")
        exit(1)
    
    if (args.tracking > 0):
        track(tra, tdec, args.lat, args.lon, args.elev, args.tracking)

if __name__ == '__main__':
    main()
