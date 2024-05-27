#!/usr/bin/env python

# Example of using the Plane class, which is imported from pymavlink_plane.py

from pymavlink_plane import *
from clrprint import *

#### GLOBALS ####
# Constant altitude for all commands
ALT = 100
# Waypoints to be used
WPS = [
    (51.12067178, -2.18868379),
    (51.12567178, -2.18868379),
    (51.12567178, -2.18468379)
]
# Counter to see how many waypoints we have commanded
WP_COUNTER = 0

def set_speed(plane, speed, type=0):
    '''
        Set commanded speed of mav
        Inputs:
        - plane = Plane object
        - speed = commanded speed [m/s]
        - type: 0 = airspeed, 1 = groundspeed
    '''
    clrprint('Setting speed = ', speed, ' m/s', clr='y')
    plane.send_cmd(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                    type,      # param1: Type = 0 (airspeed), Type = 1 (groundspeed)
                    speed,  # param2: Speed (m/s)
                    -1,     # param3: Throttle
                    0, 0, 0, 0, 0  # param4-7: unused
    )

def new_wp(plane, location, accuracy, speed=25):
    '''
        Creates new flightplan with just home and defined waypoint
        Uploads to mav
        Inputs:
        - plane = Plane object
        - location = (lat, lon) tuple
        - accuracy = int of closest approach to WP [m]
        - speed = commanded airspeed [m/s] (optional, default = 25)
    '''
    clrprint('Adding WP at ', location, clr='y')
    # CLear existing (keep home)
    plane.init_wp()
    # Add the WP
    plane.add_waypoint(location[0], location[1], ALT)
    global WP_COUNTER
    WP_COUNTER += 1
    # Upload the plan
    plane.send_all_waypoints()
    # Set the flight mode
    clrprint('Setting AUTO', clr='y')
    plane.change_mode('AUTO')
    clrprint('AUTO set :)', clr='g')
    clrprint('Current WP# = ', plane.mav.waypoint_current(), ', Total WPs = ', WP_COUNTER, clr='b')
    # Set speed
    set_speed(plane, speed)
    # Get the current target waypoint info: target.alt, target.lat, larget.lng
    target = plane.get_current_target()
    # Wait until we approach the target (within <accuracy> m, wait 1000 sec max)
    plane.wait_location(target, accuracy=accuracy, timeout=1000)
    clrprint('At WP# = ', plane.mav.waypoint_current(), clr='g')

def main(plane):
    # Initialise waypoint list    
    plane.init_wp()

    # Get the home location info: home.alt, home.lat, home.lng
    home = plane.home_position_as_mav_location()

    # Add the first WP and send
    new_wp(plane, WPS[0], 100)
    # Add the second WP and send
    new_wp(plane, WPS[1], 100)
    # Add the third WP and send
    new_wp(plane, WPS[2], 100)

    # No more commands
    clrprint('Finished, actioning RTL default', clr='red')

if __name__ == "__main__":
    # Initialise the object
    plane = Plane()

    # Connect to the specified address
    print('Attempting connect...')
    plane.connect('127.0.0.1:5761')
    print('Connected!')

    # Main script
    main(plane)


############## Test zone: bits of functionality that are tested but not currently used, for reference
'''print("ALT_HOLD_RTL value %f" % plane.get_parameter("ALT_HOLD_RTL"))
    plane.set_parameters({"ALT_HOLD_RTL": 20000})
    print("ALT_HOLD_RTL value %f" % plane.get_parameter("ALT_HOLD_RTL"))'''

'''while True:
        # plane.mav is the connection (see self.connect() definition)
        msg = plane.mav.recv_match()
        if msg != None and msg.msgname == 'GLOBAL_POSITION_INT':
            print(msg._timestamp)
            print()'''