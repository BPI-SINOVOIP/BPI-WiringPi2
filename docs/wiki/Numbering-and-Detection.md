# Numbering and Detection

## Numbering layers

BPI-WiringPi2 may expose three different identifiers for one connector pin:

| Layer | Purpose |
| --- | --- |
| Physical | The connector position printed in `gpio readall`; depends on the exact carrier/header. |
| wiringPi | Historical WiringPi application numbering. |
| BCM-style | Compatibility numbering shown in the BCM column; it is a library map, not necessarily the native SoC line number. |

Internally, the backend must also know the native SoC GPIO bank/line or Linux gpiochip offset. Do not mix this controller identifier with the public BCM-style number.

BPI-F4 is a documented exception to the usual single-header layout: physical numbers 1-29 are rows in the official combined terminal table across CN8/CN3/CN7/CN6/CN5/CN1/CN4. They are not connector-local pin labels. WiringPi and BCM-style channels 0-19 follow the GPIO-bearing table rows in order and map to native SP7350 lines `84,85,71,70,81,80,83,82,60,61,69,68,72,73,74,75,59,58,56,57`.

BPI-SM10 uses the carrier's J12 40-pin header. Its map is traced through the level shifters and CoM260 SODIMM nets to K3 global GPIO/pad numbers; physical pin 18 is unavailable because no GPIO connection is published. BPI-CanMV-K230D Zero uses JP1 and the GPIO numbers in the official table; physical pin 16 is unavailable because its published row has no GPIO number. Neither gap is inferred from a peripheral label.

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
