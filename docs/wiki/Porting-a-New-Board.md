# Porting a New Board

## Evidence gate

Before editing code, obtain all of the following:

1. Exact product/module/carrier names and PCB revisions.
2. Authoritative schematic or machine-readable pin table.
3. Exact kernel DTS/DTB and board `model`/`compatible` strings.
4. Physical pin → net → SoC pad/bank/line → Linux gpiochip/offset mapping.
5. Voltage, boot-strap, reserved, board-owned and output-unsafe pin annotations.

If any mapping is ambiguous, record a `blocked` row instead of guessing.

## Decide alias or backend

- Use an alias only when official evidence proves identical carrier wiring and backend behavior.
- Add a new internal model when the header map, bus metadata, backend, voltage or limitations differ.
- Treat module + carrier as the target. Never alias modules solely because they share a connector form factor.

## Implementation locations

- Detection aliases: `wiringPi/wiringPi_bpi.c`
- Model IDs and Banana Pi interface declarations: `wiringPi/wiringPi_bpi.h`
- Model names and backend selection: `wiringPi/wiringPi.c`
- Header readout: `gpio/readall.c`
- SoC register/backend code: the relevant source under `wiringPi/`

Keep physical, wiringPi and BCM-style arrays aligned with the companion `RPi.GPIO` repository.

## Build gate

```sh
python3 tools/audit-bpi-support.py --peer ../RPi.GPIO
make -C wiringPi
make -C devLib
make -C gpio clean
make -C gpio \
  INCLUDE='-I../wiringPi -I../devLib' \
  LDFLAGS='' \
  LIBS='../wiringPi/libwiringPi.so.3.19 ../devLib/libwiringPiDev.so.3.19 -lpthread -lrt -lm -lcrypt'
```

## Review checklist

- Canonical product name plus aliases are documented.
- The product row and status in [Complete Board Catalog](Complete-Board-Catalog) are updated.
- Detection tests include real Device Tree strings.
- `gpio readall` power/GND/non-GPIO positions match the exact header.
- Unsafe pins stay unavailable.
- Unsupported pull/alt/PWM paths return a clear limitation and do not pretend success.
- The companion RPi.GPIO map is updated in the same work unit.
- Hardware evidence names image, kernel, DTB, board and carrier revision.

Continue with [Hardware Validation](Hardware-Validation).
