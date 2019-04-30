# duckie_interface
provide a way for the duckiebot to receive velocity commands from QT GUI and output predicted pose based on velocity changes


------------------------------------------------------------
# Implementation
------------------------------------------------------------
An experiment to publish on the "cmd_vel" topic for duckiebot using the wasd keys as a simple mechanism
## Note: Not a part of the duckietown project

### How to Run
1. ``` catkin build ```
2. ``` roslaunch duckie_interface duckie_interface.launch```
3. Note: vicon_track_multiple is a requirement for this
