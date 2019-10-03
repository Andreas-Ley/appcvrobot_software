#!/bin/sh
ipaddr="$(ifconfig wlan0 | awk '/inet /{print substr($2,0)}')"
sudo /home/pi/software/software_appcvrobot/buildRel/executables/LCDTest/LCDTest "Wifi connected" ${ipaddr}
