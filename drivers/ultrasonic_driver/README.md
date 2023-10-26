# 串口ultrasonic_driver

## 参数
盲区: 300mm
最大值5000mm


**Author**: Hongda
1. 根据插口位置绑定
udevadm info -a -n /dev/ttyUSB1 | grep KERNELS
ACTION=="add",KERNELS=="3-7.1.2:1.0",SUBSYSTEMS=="usb",MODE:="0777",SYMLINK+="right_wheel"

2. 根据idVender/idProduct绑定
```
sudo udevadm trigger
sudo udevadm control –reload-rules
sudo service udev restart 
sudo udevadm trigger
```

refer: https://icode.best/i/58479648145259