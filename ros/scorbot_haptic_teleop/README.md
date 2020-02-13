# scorbot_haptic_teleop

## Scorbot Haptic teleop demo

Enable kernel module, connect Phantom Omni and launch driver
```
rosrun omni_common initialize_device.sh
roslaunch omni_common omni.launch
```
Connect Ethernet cable to PC, release emergency button, switch on power supply and launch driver
```
roslaunch scorbot_driver scorbot.launch trajectory_controller:=false
```
Launch teleop nodes
```
roslaunch scorbot_haptic_teleop scorbot_shadow.launch
```