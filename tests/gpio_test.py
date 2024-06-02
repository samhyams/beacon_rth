from pymavlink import mavutil
import time

def run(master):
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        # When a message is received
        else:
            # Check for system time updates
            if msg.get_type == 'SYSTEM_TIME':
                TIME = msg.to_dict()['time_unix_usec']
            if msg != None:
                print(msg.to_dict())

if __name__ == '__main__':
    # Make the connection to MAV
    master = mavutil.mavlink_connection('/dev/ttyS0', baud=57600)
    print('Waiting for heartbeat...')
    master.wait_heartbeat()
    print('Got heartbeat!')

    # Run main script
    run(master)


