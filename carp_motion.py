#!/usr/bin/python3
import math
import os
import sys
import ephem
import datetime
import time
import pycurl
from io import BytesIO



#
# Establish some constants
#
LATITUDE = 45.3491
LONGITUDE = -76.0413
ELEVATION = 70.00

#
# Pre-reduction ahead of the main gearboxes
#
ELEV_PREFIX = 5
AZ_PREFIX = 5
ELEV_RATIO = ELEV_PREFIX * (500.47 * 16)
AZIM_RATIO = AZ_PREFIX * (500.47 * 10)

DEG_MINUTE = 0.25

SLEW_RATE_MAX = 20.0
RELAY_ADDR = '192.168.1.4'


def send_relay_control(which, state):
    vfile = which * 2
    vfile += state
    buffer = BytesIO()
    c = pycurl.Curl()
    c.setopt(c.URL, 'http://%s/30000/%02d' % (RELAY_ADDR, vfile))
    c.setopt(c.WRITEDATA, buffer)
    c.perform()
    c.close()

body = buffer.getvalue()
# Body is a byte string.
# We have to know the encoding in order to print it to a text file
# such as standard output.
print(body.decode('iso-8859-1'))

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
# Determine desired slew rate
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

    if (abs(targ_az-cur_az) <= 2.5):
        el_slew = az_slew / 2.0
    if (abs(targ_az-cur_az) <= 1.25):
        az_slew = az_slew / 2.0
    if (abs(targ_az-cur_az) <= 0.625):
        az_slew = az_slew / 3.0

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
    # Pop the brakes off!
    #
    disable_az_brake()
    disable_el_brake()
    
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
        #
        # Looks like down below, we have stopped motion on both
        #  axes--we exit this loop when this happens
        #
        if (az_speed == -9999 and el_speed == -9999):
            set_az_speed(0.0)
            set_el_speed(0.0)
            enable_az_brake()
            enable_el_brake()
            break
        
        #
        # Update the ephem sky coordinates
        #
        local.date = ephem.now()
        v.compute(local)
        t_az = from_ephem_coord("%s" % v.az)
        t_el = from_ephem_coord("%s" % v.alt)
        
        cur_el = get_el_sensor()
        cur_az = get_az_sensor()
            
        #
        # Determine desired slew-rate based on relative distances on
        #  both axes
        #
        slew_tuple = slew_rate(t_el, t_az, cur_el, cur_az)
        
        #
        # Update commanded motor speed if desired rate is different from
        #   current rate
        #
        if (el_speed != -9999 and el_speed != slew_tuple[0]):
            el_speed = slew_tuple[0]
            set_el_speed(el_speed)
            
        if (az_speed != -9999 and az_speed != slew_tuple[1]):
            az_speed = slew_tuple[1]
            set_az_speed(az_speed)
        
        #
        # We haved reached the object in elevation--zero speed, brakes ON
        #
        if (abs(cur_el-t_el) < 0.05):
            set_el_speed(0.0)
            enable_el_brake()
            el_speed = -9999
        
        #
        # We have reached the object in azimuth--zero speed, brakes ON
        #
        if (abs(cur_az - t_az) < 0.05):
            set_az_speed(0.0)
            enable_az_brake()
            az_speed = -9999
            
        #
        # Wait 1 second between updates
        #
        time.sleep(1)
        
        #
        # Hmm, despite there being a 1-second pause the relevant axis
        #   hasn't moved.  Declare some "weirdness"
        #
        if (el_speed != -9999 and (abs(cur_el-get_el_sensor()) < 0.025)):
            weird_count += 1

        if (az_speed != -9999 and abs(cur_az-get_az_sensor()) < 0.025):
            weird_count += 1
        
        if (weird_count > 5):
            print( "Axis not moving!!")
            set_az_speed(0.0)
            set_el_speed(0.0)
            enable_az_brake()
            enable_el_brake()
            rv = False
            break
    #
    # No matter how we exit from this loop, make sure things are "safe"
    #
    set_az_speed(0.0)
    set_el_speed(0.0)
    enable_az_brake()
    enable_el_brake()
    
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
    # Pop the brakes off!
    #
    disable_az_brake()
    disable_el_brake()
    
    #
    # Set initial speed
    #
    speeds = track_rate(t_dec)
    el_speed = speeds[0]
    az_speed = speeds[1]
    set_az_speed(az_speed)
    set_el_speed(el_speed)

    time.sleep (minterval)
    while True:
        #
        # Looks like we're done
        #
        if (time.time() - start_time >= tracktime):
            set_az_speed(0.0)
            set_el_speed(0.0)
            enable_az_brake()
            enable_el_brake()
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
        if (abs(el_ratio) < 0.95 or abs(el_ratio) > 1.05):
            set_el_speed(el_speed)
        
        if (abs(az_ratio) < 0.95 or abs(az_ratio) > 1.05):
            set_az_speed(az_speed)
        
        
        time.sleep(minterval)
    #
    # No matter how we exit from this loop, make sure things are "safe"
    #
    set_az_speed(0.0)
    set_el_speed(0.0)
    enable_az_brake()
    enable_el_brake()
    
    return rv
