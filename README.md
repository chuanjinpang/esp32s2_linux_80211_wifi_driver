# esp32s2_linux_80211_wifi_driver
my toy diy poroject for esp3s2 linux 802.11 wifi driver, include linux host driver & esp32s2 firmware 

开源软件怎么说了？公用品，方便了用户，但是怎么维护开发者的权益是个问题？问题不因回避而消失。



---

## 1.新手使用说明：

1. 升级esp32s2固件，已经编译好了。直接下载固件到esp32s2中即可。

esp32s2_linux_80211_wifi_driver\test_firmware

2. 安装Linux主机驱动， 默认是centos7.6的主机驱动。

   如果是pi的话，请注释掉main.c 中这行：#define LINUX_VERSION_CODE   KERNEL_VERSION(4, 4, 0)

make    进行编译

modprobe cfg80211

insmod xfz_usb_wifi.ko

驱动安装完毕

3.升级 wpa_supplicant-2.9.tar.gz 请参考网上的编译安装步骤。本主机驱动依赖高版本的wifi认证服务 wpa_supplicant

4.使用系统GUI配置wifi网络，这个和其它usb无线网卡一样，不啰嗦。



## 2.学习Linux wifi 802.11用户使用说明

esp32s2的固件源码，计划分3次发布完。

阶段1：先发一个学习版本，不影响参考学习。性能上可以看720P在线视频，~2.5Mbps的速度。对usb error recovery代码只提供.o二进制。如果esp32s2 usb没这问题，根本就不存在这段代码。

阶段2 ：高性能版本, 性能上大约可以看1080P在线视频，相比前一版本大约提高1Mbps, 即~3.5Mbps,峰值5+Mbps。

阶段3：发布usb error recovery代码。

毕竟乐鑫也没有开源wifi部分的源码，USB部分的资料也没放出来。

这就是开发者对代码有一点掌控力的福利，2022年内发布完。



## 3.已知问题

1. rmmod  host driver时会有一个警告，调查过，不影响功能。 懒得修了。

   

## 4.将来的计划

也许会开发一个SPI接口的80211 wifi给嵌入式Linux用。

