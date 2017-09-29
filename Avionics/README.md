# Avionics
Avionics are the electronic systems used to control the Falcon flight. Avionic systems include communications, navigation, and stability control. 
The term avionics is a portmanteau of the words aviation and electronics.
The Falcon 9 Electric Fun avionics include:
- NAze 32 rev6 Flight Controller
- ESC 60A 
- ...

# Documentation as we go (blog style)

## 1. [Naze 32bit rev 6 Flight Controller Guide](http://www.dronetrest.com/t/naze-32-revision-6-flight-controller-guide/1605)

## 2. Setup/Naze32 
  1. Using USB to connect to the Naze32 requires the installation of the SiLabs CP2102 VCP Drivers. The drivers are available for download directly from the [SiLabs Website](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers)
  2. Force the serial connection to be recognized by running the following line at the command terminal: sudo ln -s /dev/tty.SLAB_USBtoUART /dev/tty.usbserial
  3. Fire up Cleanflight!

## 3. [Naze32 Rev6 manual](http://quadquestions.com/Naze32_rev6_manual_v1.2.pdf)

## 4.[Fucking RC legacy protocols!](http://www.dronetrest.com/t/rc-radio-control-protocols-explained-pwm-ppm-pcm-sbus-ibus-dsmx-dsm2/1357) 
  
## [WTF is Oneshot ESC?](https://oscarliang.com/oneshot125-esc-quadcopter-fpv/)