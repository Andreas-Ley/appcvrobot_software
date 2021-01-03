#!/bin/sh
cp *.service /etc/systemd/system
systemctl daemon-reload
systemctl enable shutdownNotification.service --now
systemctl enable bootNotification.service --now
systemctl enable wifiNotification.service --now
systemctl enable RobotHardwareController.service --now
