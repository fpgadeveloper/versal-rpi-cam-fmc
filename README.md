# Reference design for RPi Camera FMC on Versal platforms

## UNDER DEVELOPMENT

This project is still in development and undergoing major changes.

## Description

This project demonstrates the Opsero [RPi Camera FMC](https://camerafmc.com/docs/rpi-camera-fmc/overview/) used to
connect 4x Raspberry Pi cameras (or compatible cameras) to one of the target Versal boards listed below. The 
designs contain:

* 4x MIPI CSI capture pipelines
* [VVAS MultiScaler] kernel for hardware accelerated image processing
* Video Mixer IP based display pipeline to HDMI output

Important links:
* The RPi Camera FMC [datasheet](https://camerafmc.com/docs/rpi-camera-fmc/overview/)
* To [report an issue](https://github.com/fpgadeveloper/versal-rpi-cam-fmc/issues)
* For technical support: [Contact Opsero](https://opsero.com/contact-us)

## Requirements

This project is designed for version 2022.1 of the Xilinx tools (Vivado/Vitis/PetaLinux). 
If you are using an older version of the Xilinx tools, then refer to the 
[release tags](https://github.com/fpgadeveloper/versal-rpi-cam-fmc/tags "releases")
to find the version of this repository that matches your version of the tools.

### License

These designs contain [Xilinx AMD HDMI IP] for driving an HDMI monitor. A license is required to build
designs containing this core. An evaluation license is available.

### Software

* Vivado 2022.1
* Vitis 2022.1
* PetaLinux Tools 2022.1

### Hardware

* Linux PC for build
* One or more [Raspberry Pi Camera Module 2](https://www.raspberrypi.com/products/camera-module-v2/)
* 1x [RPi Camera FMC](https://camerafmc.com/buy/ "RPi Camera FMC")
* 1x HDMI monitor that supports 1080p video
* One of the supported target boards listed below

## Target designs

| Target board             | Target design   | FMC slot used | Cameras |
|--------------------------|-----------------|----------|---------|
| [VCK190]                 | `vck190_fmcp1`  | FMCP1    | 2 (note 1) |
| [VCK190]                 | `vck190_fmcp2`  | FMCP2    | 2 (note 1) |
| [VMK180]                 | `vmk190_fmcp1`  | FMCP1    | 2 (note 1) |
| [VMK180]                 | `vmk190_fmcp2`  | FMCP2    | 2 (note 1) |

Notes:
1. The FMCP1 connector of the VCK190/VMK180 board can only support 2 cameras due to the pin assignment.
   This design supports the `CAM1` and `CAM2` slots as labelled on the RPi Camera FMC.

## Build instructions

This repo contains submodules. To clone this repo, run:
```
git clone --recursive https://github.com/fpgadeveloper/versal-rpi-cam-fmc.git
```

Source Vivado and PetaLinux tools:

```
source <path-to-petalinux>/2022.1/settings.sh
source <path-to-vivado>/2022.1/settings64.sh
```

Build all (Vivado project, accelerator kernel and PetaLinux):

```
cd versal-rpi-cam-fmc/PetaLinux
make petalinux TARGET=vck190_fmcp1
```

## Contribute

We strongly encourage community contribution to these projects. Please make a pull request if you
would like to share your work:
* if you've spotted and fixed any issues
* if you've added designs for other target platforms
* if you've added software support for other cameras

Thank you to everyone who supports us!

### The TODO list

* Software support for more cameras (this will be an ongoing task due to the number of cameras available).

## About us

[Opsero Inc.](https://opsero.com "Opsero Inc.") is a team of FPGA developers delivering FPGA products and 
design services to start-ups and tech companies. Follow our blog, 
[FPGA Developer](https://www.fpgadeveloper.com "FPGA Developer"), for news, tutorials and
updates on the awesome projects we work on.

[3]: https://camerafmc.com/docs/rpi-camera-fmc/overview/
[AMD Xilinx MIPI CSI Controller Subsystem IP]: https://docs.xilinx.com/r/en-US/pg202-mipi-dphy
[RPi Camera FMC]: https://camerafmc.com/docs/rpi-camera-fmc/overview/
[GStreamer]: https://gstreamer.freedesktop.org/
[VVAS MultiScaler]: https://xilinx.github.io/VVAS/2.0/build/html/docs/common/Acceleration-Hardware.html#multiscaler-kernel
[G-Streamer plugins]: https://xilinx.github.io/VVAS/2.0/build/html/docs/common/common_plugins.html
[VCK190]: https://www.xilinx.com/vck190
[VMK180]: https://www.xilinx.com/vmk180
[Xilinx AMD HDMI IP]: https://www.xilinx.com/products/intellectual-property/hdmi.html

