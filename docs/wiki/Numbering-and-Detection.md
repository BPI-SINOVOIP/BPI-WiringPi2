# Numbering and Detection

## Numbering layers

BPI-WiringPi2 may expose three different identifiers for one connector pin:

| Layer | Purpose |
| --- | --- |
| Physical | The connector position printed in `gpio readall`; depends on the exact carrier/header. |
| wiringPi | Historical WiringPi application numbering. |
| BCM-style | Compatibility numbering shown in the BCM column; it is a library map, not necessarily the native SoC line number. |

Internally, the backend must also know the native SoC GPIO bank/line or Linux gpiochip offset. Do not mix this controller identifier with the public BCM-style number.

## Board detection

Detection aliases are maintained in `wiringPi/wiringPi_bpi.c`; effective Banana Pi model IDs are declared in `wiringPi/wiringPi_bpi.h`. The implementation may inspect Device Tree model/compatible strings and legacy CPU information, depending on the platform.

For a new board, collect both values from the exact production image:

```sh
cat /proc/device-tree/model; echo
tr '\0' '\n' < /proc/device-tree/compatible
```

Prefer exact `compatible` strings over broad marketing names. Generic model strings such as a shared carrier family must not silently select a different header revision.

## Modules and carriers

A compute module does not define a 26/40-pin user header by itself. The map key must be the exact carrier/development kit. Module names may be detection context, but they cannot justify a carrier mapping without schematic/DTS evidence.

Examples:

- LM7 uses W3 as its documented development kit.
- CM4, CM5, CM5 Pro, CM6, AIM7 and AI2N support claims must name the exact IO board/carrier.

## Detection bug report data

Attach:

```sh
uname -a
cat /etc/os-release
cat /proc/device-tree/model; echo
tr '\0' '\n' < /proc/device-tree/compatible
gpio -v
gpio readall
```

Also provide board, module and carrier revisions and a clear header photograph.
