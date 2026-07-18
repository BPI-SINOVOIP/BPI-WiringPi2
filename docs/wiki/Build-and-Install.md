# Build and Install

## Requirements

- A supported Banana Pi running Linux.
- A compiler, `make`, libc development headers and Git.
- Root permission for installation and, depending on the backend/device permissions, runtime GPIO access.

Example Debian/Ubuntu preparation:

```sh
sudo apt update
sudo apt install build-essential git
```

## Build from this repository

```sh
git clone https://github.com/BPI-SINOVOIP/BPI-WiringPi2.git
cd BPI-WiringPi2
./build
sudo ldconfig
```

The `build` script builds and installs `libwiringPi`, `libwiringPiDev` and the `gpio` utility. Review the script before running it on production systems because installation modifies system library and binary paths.

## Non-installing developer build

Use this path when reviewing a port without installing it system-wide:

```sh
make -C wiringPi
make -C devLib
make -C gpio clean
make -C gpio \
  INCLUDE='-I../wiringPi -I../devLib' \
  LDFLAGS='' \
  LIBS='../wiringPi/libwiringPi.so.3.19 ../devLib/libwiringPiDev.so.3.19 -lpthread -lrt -lm -lcrypt'
```

## Verify the installation

```sh
gpio -v
gpio readall
```

`gpio readall` should show the detected board and its exact connector layout. Stop if the board name, header size or power/GND positions are wrong.

## Uninstall

```sh
./build uninstall
```

See [Known Limitations and Roadmap](Known-Limitations-and-Roadmap) before assuming every displayed pin supports pull, alternate functions or PWM.
