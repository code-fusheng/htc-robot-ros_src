#!/bin/bash

# Intel RealSense D435i
echo 'SUBSYSTEM=="video4linux",ATTR{name}=="Intel(R) RealSense(TM) Depth Ca",ATTR{index}=="0",MODE:="0777",SYMLINK+="intel_d435i"' >>/etc/udev/rules.d/htc_camera.rules
# astra_s
echo 'SUBSYSTEM'=="usb", ATTR{idProduct}=="0402", ATTR{idVendor}=="2bc5", MODE:="0666", OWNER:="root", GROUP:="video", SYMLINK+="astra_s" >>/etc/udev/rules.d/htc_camera.rules
# IMU N100
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="imu_n100"' >>/etc/udev/rules.d/htc_imu.rules
# GNSS G60
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0005", MODE:="0777", GROUP:="dialout", SYMLINK+="gps_g60"' >>/etc/udev/rules.d/htc_gps.rules
# wheeltec stm32
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_stm32"' >/etc/udev/rules.d/htc_vcu.rules
echo 'KERNEL=="ttyACM0*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_stm32"' >/etc/udev/rules.d/htc_vcu.rules

sudo service udev reload
sleep 2
service udev restart