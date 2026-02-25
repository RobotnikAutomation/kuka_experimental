# Kuka experimental

[![Build Status](http://build.ros.org/job/Idev__kuka_experimental__ubuntu_trusty_amd64/badge/icon)](http://build.ros.org/job/Idev__kuka_experimental__ubuntu_trusty_amd64)

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

Experimental packages for Kuka manipulators within [ROS-Industrial][].
See the [ROS wiki][] page for more information.


## Contents

This repository contains packages that will be migrated to the [kuka][]
repository after they have received sufficient testing. The contents of 
these packages are subject to change, without prior notice. Any available 
APIs are to be considered unstable and are not guaranteed to be complete 
and / or functional.

## Obuses project notes (Noetic)

- Package `kuka_rsi_cartesian_hw_interface` now exposes:
  - `force_udp_reconnect` (`std_srvs/Trigger`)
- In the default bringup namespace (`kuka_robot`) this is available as:
  - `/kuka_robot/force_udp_reconnect`
- Intended use:
  - request a runtime UDP reconnection to RSI without restarting bringup.


[ROS-Industrial]: http://wiki.ros.org/Industrial
[ROS wiki]: http://wiki.ros.org/kuka_experimental
[kuka]: https://github.com/ros-industrial/kuka
