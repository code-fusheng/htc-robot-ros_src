# Intel RealSense D435i
echo 'SUBSYSTEM=="video4linux",ATTR{name}=="Intel(R) RealSense(TM) Depth Ca",ATTR{index}=="0",MODE:="0777",SYMLINK+="intel_d435i"' >>/etc/udev/rules.d/camera.rules

echo 'SUBSYSTEM'=="usb", ATTR{idProduct}=="0402", ATTR{idVendor}=="2bc5", MODE:="0666", OWNER:="root", GROUP:="video", SYMLINK+="astra_s"

service udev reload
sleep 2
service udev restart