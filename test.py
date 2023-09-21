'''
to use with mavproxy sitl, navigate to arupilot/ArduPlane and run:
sim_vehicle.py --console --map

when open, in the original command window, run:
output add 127.0.0.1:5761
this adds the UDP port for us to listen on here

'''
#


from pymavlink import mavutil
import pprint

master = mavutil.mavlink_connection('127.0.0.1:5761')

print('wait for hb')

master.wait_heartbeat()

print('got')

'''message = master.mav.command_long_encode(
        master.target_system,  # Target system ID
        master.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
        0,  # Confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,  # param1: Message ID to be streamed
        30e7, # param2: Interval in microseconds
        0,       # param3 (unused)
        0,       # param4 (unused)
        0,       # param5 (unused)
        0,       # param5 (unused)
        0        # param6 (unused)
        )'''

# Using the MAVProxy example for DO_CHANGE_SPEED:
# line 176 in MAVProxy/MAVProxy/modules/mavproxy_cmdlong.py
# message ref: mavlink command 178
speed = 15
message = master.mav.command_long_encode(
        master.target_system,  # Target system ID
        master.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,  # ID of command to send
        0,  # Confirmation
        0,  # param1: Type = 0 (airspeed)
        speed, # param2: SPeed (m/s)
        0,       # param3: Throttle
        0,       # param4 (unused)
        0,       # param5 (unused)
        0,       # param5 (unused)
        0        # param6 (unused)
        )


# NOT SUPPORTED (BY PLANE??)
# altitude change
# message ref: mavlink command 186
alt = 200
message = master.mav.command_long_encode(
        master.target_system,  # Target system ID
        master.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,  # ID of command to send
        0,  # Confirmation
        alt,  # param1: Set altitude (m)
        2, # param2: frame ()
        0,       # param3: Throttle
        0,       # param4 (unused)
        0,       # param5 (unused)
        0,       # param5 (unused)
        0        # param6 (unused)
        )


# LOITER TO ALT (at current position)
# message ref: mavlink command 31
alt = 200
message = master.mav.command_long_encode(
        master.target_system,  # Target system ID
        master.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT,  # ID of command to send
        0,  # Confirmation
        1,       # param1: heading required on exit (true/false)
        50,      # param2: Loiter radius (m)
        0,       # param3: (unused)
        0,       # param4: xtrac on exit (true/false)
        0,       # param5: Latitude (0 = current position)
        0,       # param6: Longitude (0 = current position)
        alt      # param7: Altitude (m)
        )

# DO JUMP
# message ref: mavlink command 177
wp_id = 2
message = master.mav.command_long_encode(
        master.target_system,  # Target system ID
        master.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_DO_JUMP,  # ID of command to send
        0,  # Confirmation
        wp_id,       # param1: sequence number
        0,      # param2: repeat #
        0,       # param3: (unused)
        0,       # param4: (unused)
        0,       # param5: (unused)
        0,       # param6: (unused)
        alt      # param7: (unused)
        )

# CHANGE ALT
# message ref: mavlink command 30
alt = 200
message = master.mav.command_long_encode(
        master.target_system,  # Target system ID
        master.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT,  # ID of command to send
        0,  # Confirmation
        0,       # param1: action
        0,      # param2: (unused)
        0,       # param3: (unused)
        0,       # param4: (unused))
        0,       # param5: (unused)
        0,       # param6: (unused)
        alt      # param7: Altitude (m)
        )

# NAV WP
# message ref: mavlink command 16
alt = 200
message = master.mav.command_long_encode(
        master.target_system,  # Target system ID
        master.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # ID of command to send
        0,  # Confirmation
        0,       # param1: (unused)
        20,       # param2: Accept radius (m)
        0,       # param3: Pass radius (m)
        0,       # param4: (unused)
        51.12067178,       # param5: Latitude
        -2.18368379,       # param6: Longitude
        alt      # param7: Altitude (m)
        )

# Send the COMMAND_LONG     
master.mav.send(message)

# Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result
response = master.recv_match(type='COMMAND_ACK', blocking=True)
'''if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("Command accepted")
else:
    print("Command failed")'''


exit()


packettypes = []

while True:
    msg = master.recv_match()
    if msg != None:
        msg = msg.to_dict()
        # Print all packet types
        if False:
            if msg['mavpackettype'] not in packettypes:
                packettypes.append(msg['mavpackettype'])
            print(packettypes)
            
        # Filter by packet types, print full dict
        if True and msg['mavpackettype'] == 'VFR_HUD':
            pprint.pprint(msg)

    
    

    
