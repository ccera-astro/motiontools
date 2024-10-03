# motiontools

The scripts in this repository are used to position the dish.   To operate the dish
follow the steps below

1. Operate key switch.
2. Press the "START" button.
3. Log in to motion control machine.
4. `cd motiontools` 
5. `./MotionServer
6. Check that both the Azimuth and Elevation lights are on.   
- If not, press "Exit Motion Server" button; press "STOP" and then "START"; 
and respond "Yes" to "Clear Alerts?"
7. Use `carp_motion.py` to position dish. e.g., to track J0332+5434 for one hour enter

`      ./carp_motion.py --serror 0.05 --RA 3.5333 --dec 54.57 --tracking 3600. `




