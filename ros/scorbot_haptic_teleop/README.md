# Conectar firewire
# conectar alimetanción phantom

rosrun omni_common initialize_device.sh
roslaunch omni_common omni.launch

---

roslaunch scorbot_haptic_teleop scorbot_shadow.launch

---
# Conectar alimentacion placa USB
# conectar alimetanción puente H
roslaunch scorbot_driver scorbot.launch