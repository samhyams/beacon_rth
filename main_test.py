#!/usr/bin/env python

# Example of using the Plane class, which is imported from pymavlink_plane.py

from pymavlink_plane import *
from clrprint import *

def main():
    # Constant altitude for all commands
    ALT = 100

    # Initialise the object
    plane = Plane()

    # Connect to the specified address
    print('Attempting connect...')
    plane.connect('127.0.0.1:5761')
    print('Connected!')
    
    plane.init_wp()

    # Get the home location info: home.alt, home.lat, home.lng
    home = plane.home_position_as_mav_location()

    ## Create a new plan for upload
    # Add a waypoint to the plan (west a bit)
    plane.add_waypoint(51.12067178, -2.18868379, ALT)
    # Upload the plan
    plane.send_all_waypoints()
    # Set the flight mode
    clrprint('Setting AUTO', clr='y')
    plane.change_mode('AUTO')
    clrprint('AUTO set :)', clr='g')
    clrprint('Current WP# = ', plane.mav.waypoint_current(), clr='b')
    # Get the current target waypoint info: target.alt, target.lat, larget.lng
    target = plane.get_current_target()
    print(target)

    # Wait until we approach the target (within 250 m, wait 1000 sec max)
    plane.wait_location(target, accuracy=250, timeout=1000)

    plane.wploader.clear()
    # Add a waypoint to the plan (north a bit)
    plane.add_waypoint(51.12567178, -2.18868379, ALT)
    # Upload the plan
    plane.send_all_waypoints()
    # Set the flight mode
    plane.change_mode('AUTO')
    print('*****************************',plane.mav.waypoint_current())
    
    # Get the current target waypoint info: target.alt, target.lat, larget.lng
    target = plane.get_current_target()
    print(target)

    # Wait until we approach the target (within 250 m, wait 1000 sec max)
    plane.wait_location(target, accuracy=50, timeout=1000)


if __name__ == "__main__":
    main()


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