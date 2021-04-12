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

