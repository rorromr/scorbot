#!/bin/bash

workspace="$(dirname "$(dirname "$(rospack find scorbot_driver)")")"
driver="${workspace}/devel/lib/scorbot_driver/${1}"
sudo setcap cap_net_raw+ep "$driver"
