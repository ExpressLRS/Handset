The current version of the handset uses the TTGO T-Display-GD32 RISC-V development board. Unfortunately since this is a relatively new architecture there are some rough edges in the support for it. This page documents the known issues and work-arounds.

1) Conflict with C++ bool in header file

Problem:

```
/home/testuser/.platformio/packages/framework-gd32vf103-sdk/GD32VF103_standard_peripheral/gd32vf103.h:179:41: error: redeclaration of C++ built-in type 'bool' [-fpermissive]
  179 | typedef enum {FALSE = 0, TRUE = !FALSE} bool;
      |                                         ^~~~
```

Cause:

The header file from the gd32 sdk isn't fully C++ compatible yet

Fix:

Comment out line 179 in .platformio/packages/framework-gd32vf103-sdk/GD32VF103_standard_peripheral/gd32vf103.h

```
// typedef enum {FALSE = 0, TRUE = !FALSE} bool;
```

see: https://github.com/riscv-mcu/GD32VF103_Firmware_Library/issues/1

2) Build fails with missing dfu-suffix tool

(Looks like this may have been fixed in recent platformio versions)

Problem:

```
Adding dfu suffix to firmware.bin
sh: 1: bin/dfu-suffix: not found
```

Cause:

dfu tools don't get automatically installed by platformio.

Fix:

Download dfu-utils from https://bintray.com/platformio/tool-packages (use search for tool-dfu to find the right package for your operating system). Extract the archive and copy it to (for linux)

$HOME/.platformio/packages/tool-dfuutil

see https://community.platformio.org/t/dfu-suffix-not-found-for-longan-nano-build/16710/4

3) dfu-util: Cannot open DFU device 28e9:0189

On Linux, indicates a need to add a rule to udev. Either create a new file in /etc/udev/rules.d or add to an existing one if you already have stuff for other dfu (e.g. betaflight FC controllers)

ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="28e9", ATTRS{idProduct}=="0189", MODE="0664", GROUP="whichever group you want to use for dfu access"
  
I just use my main userid group for access rather than going to the trouble of adding a special one for dfu.

You may need to run
```
udevadm control --reload-rules
udevadm trigger
```
as root after updating the rules for them to take effect.
