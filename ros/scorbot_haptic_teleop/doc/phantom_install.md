Install Geomagic Touch (PHANToM Omni) in Ubuntu 14.04 (ROS Indigo)
------------------------------------------------------------------

```bash
sudo apt-get install --no-install-recommends freeglut3-dev g++ libdrm-dev libexpat1-dev libglw1-mesa-dev libmotif-dev libncurses5-dev libraw1394-dev libx11-dev libxdamage-dev libxext-dev libxt-dev libxxf86vm-dev tcsh unzip x11proto-dri2-dev x11proto-gl-dev x11proto-print-dev libbullet-dev
sudo dpkg -i phantomdevicedrivers_4.3-3_amd64.deb
sudo cp libPHANToMIO.so.4.3 /usr/lib
sudo cp PHANToMConfiguration /usr/sbin
sudo ln -s /usr/lib64/libPHANToMIO.so.4.3 /usr/lib/libPHANToMIO.so.4
sudo ln -s /usr/lib/x86_64-linux-gnu/libraw1394.so.11.1.0 /usr/lib/libraw1394.so.8
sudo chmod 777 /dev/fw*
```



```bash
LANG=en_us /usr/sbin/PHANToMConfiguration
```

PHANToMConfiguration:
* `Hardware > PHANToM` to `Default PHANToM`.
* `Hardware > PHANToM Model` to `Omni`. Press `Defaults` button.
* Press OK

If you need to, exchange all "," with "." in floating point numbers in `/etc/SensAble/PHANToMDeviceDrivers/PHANToM0.ini`. 

```python
# -*- coding: utf-8 -*-
import shutil

print ' *******************************************************'
print ' * Fixing , -> . in floating point numbers             *'
print ' * in /etc/SensAble/PHANToMDeviceDrivers/PHANToM0.ini  *'
print ' * (backup file created)                               *'
print ' * usage: sudo python phantomfix.py                    *'
print ' *******************************************************'

shutil.copyfile('/etc/SensAble/PHANToMDeviceDrivers/PHANToM0.ini','/etc/SensAble/PHANToMDeviceDrivers/PHANToM0.ini.bak')
input = open('/etc/SensAble/PHANToMDeviceDrivers/PHANToM0.ini.bak')
output = open('/etc/SensAble/PHANToMDeviceDrivers/PHANToM0.ini','w')

for s in input:
    s = s.replace(',0','.0');
    s = s.replace(',1','.1');
    s = s.replace(',2','.2');
    s = s.replace(',3','.3');
    s = s.replace(',4','.4');
    s = s.replace(',5','.5');
    s = s.replace(',6','.6');
    s = s.replace(',7','.7');
    s = s.replace(',8','.8');
    s = s.replace(',9','.9');
    output.write(s)
output.close()
input.close()
```

```bash
sudo python phantomfix.py
```

```bash
sudo dpkg -i openhaptics-ae_3.0-2_amd64.deb
sudo ln -s /usr/lib64/libHL.so.3.0.0 /usr/lib/libHL.so.3.0
sudo ln -s /usr/lib64/libHD.so.3.0.0 /usr/lib/libHD.so.3.0
```

```bash
git clone https://github.com/fsuarez6/phantom_omni
```


References
----------

* http://www.csc.kth.se/~jofo02/H3DTrunkGNULinux.html
* http://robotica.unileon.es/index.php/Alvaro-RV-HAPTICROS01
* https://github.com/danepowell/phantom_omni
* https://github.com/fsuarez6/phantom_omni
* https://github.com/ytsutano/raw_omni
* https://github.com/gt-ros-pkg/hrl/tree/master/hrl_hardware_drivers/phantom_omni
* http://wiki.ros.org/phantom_omni
* https://github.com/radarsat1/dimple/blob/master/doc/phantom_howto.md
* https://github.com/MurpheyLab/trep_omni


