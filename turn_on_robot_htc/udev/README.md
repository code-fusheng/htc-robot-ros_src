# ubuntu 环境下 的 ROS 工程中 设计绑定设备的串口号

1. 确保设备已经连接到计算机上，并且确定设备文件路径，例如 /dev/ttyUSB0

```shell
# Intel D435i 相机串口
$ lsusb
Bus 001 Device 006: ID 8086:0b3a Intel Corp. Intel(R) RealSense(TM) Depth Camera 435i
```

2. 创建规则文件，例如 htc-robot-camera.rules

3. 在规则文件中添加内容

模版
KERNEL=="ttyUSB*", ATTRS{idVendor}=="<YOUR_VENDOR_ID>", ATTRS{idProduct}=="<YOUR_PRODUCT_ID>", SYMLINK+="ttyACM%n", OWNER="<YOUR_USERNAME>"
参数说明
KERNEL=="ttyUSB*": 指定设备的内核名称以匹配所有 ttyUSB 开头的串口设备
ATTRS{idVendor}=="8086": 指定设备的供应商 ID 为"8086"，用于唯一标识设备的制造商
ATTRS{idProduct}=="0b3a": 指定设备的产品 ID 为"0b3a"，用于唯一标识设备的产品类型
ATTRS{serial}=="0001": 指定设备的序列号为"0001"，用于唯一标识设备的序列号
MODE:="0777": 设置设备文件权限为 0777，允许任何用户对设备进行读写操作
GROUP:="dialout": 将设备文件的所有权组设置为"dialout"，使"dialout"组的用户具有对设备的访问权限
SYMLINK+="htc_camera": 创建一个名为"htc_camera"的符号链接，指向与匹配的设备文件相关联
\> /etc/udev/rules.d/htc_camera.rules: 将规则写入/etc/udev/rules.d/目录下的 htc_camera.rules 文件中。

```shell
# 示例&解析
KERNEL=="ttyUSB*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", SYMLINK+="intel_d435i"
```
