# Known Limitations and Roadmap

## General limitations

- Many newer backends have source-backed basic digital I/O but incomplete pull, alternate-function or hardware PWM validation.
- Direct register access depends on SoC register layout, page mapping and kernel/device permissions.
- GPIO expanders, EC/MCU/RT-core pins and isolated industrial IO may require libgpiod or a device-specific API instead of this mmap-oriented library.
- I2C/SPI metadata can differ by image and overlay. A compiled device path is not proof that the node exists on every distribution.
- Carrier/module confusion is a high-risk failure mode; an incorrect map can drive power, boot or board-owned signals.

## Current roadmap

### Newly implemented, hardware validation required

- BPI-SM10 / SpacemiT K3: detection, J12 map, K3 GPIO direction/read/write, pull and alternate-function register paths build locally; physical pin 18 is deliberately unavailable.
- BPI-CanMV-K230D Zero / Kendryte K230D: detection, JP1 map, two-bank direction/read/write and IOMUX pull paths build locally; physical pin 16 is deliberately unavailable.

### Implemented but hardware validation required

BPI-F4 / Sunplus SP7350 has basic direction/read/write support and a terminal-row map. Pull, exact production detection identity, I2C/SPI device nodes and real hardware behavior remain unverified. SM10 and K230D Zero have stronger public source identity, but still require gpioinfo plus safe input/output/pull tests on exact production images before their capability claims can be widened.

### Needs exact internal board evidence

BPI-M2C, F2, F5, 2K0300, CM2+CM4IO, CM5+CM4IO, RK3588 Stamp-hole/Gold-finger carriers, S64 Core kit and AI2H carrier.

### Needs product scope decision

K3 Pico-ITX FPC/RT24, OpenWrt One mikroBUS, R4 Mini, 5202 module bus and 2K3000 isolated IO.

## Definition of done

A board is not complete until:

- exact board/carrier identity and authoritative mapping are recorded;
- detection and three numbering layers are consistent across both GPIO repositories;
- the code builds;
- safe hardware digital I/O passes on a named image;
- every untested pull/edge/PWM/bus function remains visibly limited;
- documentation and the support matrix are updated in the same change.
