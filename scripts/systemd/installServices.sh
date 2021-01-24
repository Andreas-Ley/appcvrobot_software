#!/bin/sh
cp *.service /etc/systemd/system
systemctl daemon-reload
#systemctl enable shutdownNotification.service --now
cp /home/pi/software/software_appcvrobot/buildRel/executables/Shutdown/Shutdown /usr/lib/systemd/system-shutdown/
systemctl enable bootNotification.service --now
systemctl enable wifiNotification.service --now
systemctl enable RobotHardwareController.service --now
