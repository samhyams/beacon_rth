# Beacon RTH
A basic pymavlink implementation for ArduPlane for control of fixed wing mav types.

## Usage
Open ArduPlane simulator using mavproxy.
```
$ cd ~/ardupilot/ArduPlane/
$ sim_vehicle.py -v ArduPlane -w --console --map -L patch
```
Then, within the mavproxy terminal window, set the flight mode to takeoff and arm it to start the flight:
```
mode takeoff
arm throttle
```
The test script is setup to communicate on port 5761, which is not the default, so add the output in the mavproxy terminal:
```
output add 127.0.0.1:5761
```
In a separate terminal window, now run the pymavlink script:
```
$ cd ~/beacon_rth/
$ python big_test.py
```

## File List
- `main_test.py`: Main script for high-level interactions
- `pymavlink_plane.py`: Plane class implementation
- `test.py`: lower-level testing
- `mavpackettypes.txt`: messages available from mav
