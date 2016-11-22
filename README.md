# Cloudroid (micROS-cloud)
## Introduction
Cloudroid is a cloud robotic platform which supports the direct deployment of ROS software packages onto the cloud. Basically, it can be regarded as a PAAS platform which adopts the ROS application model. A ROS package can be covnerted into a cloud service automatically. The robotic applications can access the cloud service remotely in an on-demand style through a WebSocket protocol.

The service access is purely based on a cloud service paradigm, which means that you need not concern ROS master and other configurations. Multiple robots can access a service simultaneously, for example, to build their own map respectively. The robotic apllications which access the cloud services also need no modification, because Cloudrid can generate a stub ROS package with the same interface of the original ROS package, which acts as a local proxy of the remote cloud service.

By adopting the docker container technology in the back-end, a ROS package which is orignally designed for a single robot can serve multiple robots simultaneously by dynamically instantiation of the servant in the cloud. And by specifying the resource demand of the ROS package (e.g., mem, CPU, etc.), the quaility of a service can be assured by the internal mechanisms of Cloudroid.

Please contact us through dingbo@nudt.edu.cn or bding@msn.com. Any feedback would be greatly appreciated.

## Release notes

v0.10 [2016-08-31]

1. Initial open source release


