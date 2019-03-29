scorbot_description
-------------------

## Create Collada files from SolidWorks

* Create a VM with Windows using VirtualBox
* Install guest tools
* Configure shared folder
* Change MAC address of the VM
* Create a VM snapshot to recover the VM to this state in the future
* [Download Collada Exporter](https://www.simlab-soft.com/3d-plugins/SolidWorks/Collada_exporter_for_SolidWorks-main.aspx#download)
* Create a [disposable email account](https://getinboxes.com)
* Install Collada exporter and request license file
* Check inbox of disposable email and download license file
* Setup Collada exporter license file in SolidWorks

Some known issues:

* DAE files should be rotated in X axis, it's common to use xacro `rpy="${M_PI/2.0} 0 0"`
* Images from SolidWorks should be saved together with DAE file


