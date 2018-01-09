Scorbot ER VII
--------------

Installation
------------

### System requirements: 
* Ubuntu 14.04 

These libraries are required by the Scorbot ER VII: 

* [Simple Open EtherCAT master](https://github.com/OpenEtherCATsociety/SOEM)


### Installation:

Install dependencies:

```shell
sudo apt-get install cmake git git-core
```

Intall SOEM:

```shell
git clone https://github.com/OpenEtherCATsociety/SOEM
cd SOEM
mkdir build
cd build
cmake ..
make
make install
echo "export SOEM_PATH=\""$(dirname $(pwd))"/install\"" >> ~/.bashrc
```

Run
---

The Scorbot ER VII EherCAT driver needs access to the raw ethernet device. Under Linux a normal user does not have access to the raw ethernet device. You can grand this capability to a program by the tool `setcap`. To install `setcap` use:

```shell
sudo apt-get install libcap2-bin
```

To provide a executable with raw access to a ethernet device use:

```shell
sudo setcap cap_net_raw+ep [executable]
```

This have to be done whenever the executable is created or replaced (e.g. after building).


