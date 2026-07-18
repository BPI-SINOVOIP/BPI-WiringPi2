# Quick Start

## Inspect before switching a pin

```sh
gpio -v
gpio readall
```

Confirm all of the following before output testing:

- the detected board and carrier are correct;
- the physical header orientation is known;
- the selected pin is a 3.3 V GPIO, not power, ground, a boot strap or a board-owned peripheral;
- the selected pin is documented as output-safe for the exact board revision.

## Numbering modes in the `gpio` command

```text
gpio <command> ...      wiringPi number
gpio -g <command> ...   BCM-style column from gpio readall
gpio -1 <command> ...   physical header number
```

After a hardware owner approves a physical output pin, substitute it for `SAFE_PHYS_PIN`:

```sh
gpio -1 mode SAFE_PHYS_PIN out
gpio -1 write SAFE_PHYS_PIN 1
gpio -1 read SAFE_PHYS_PIN
gpio -1 write SAFE_PHYS_PIN 0
```

Do not copy a pin number from another Banana Pi model. BCM-style numbers in this project are compatibility identifiers from the board map; they are not proof that two SoCs share the same hardware numbering.

## Minimal C program

Choose the setup function that matches the intended numbering mode:

```c
#include <wiringPi.h>

int main(void)
{
    const int approved_bcm_pin = 0; /* replace after checking gpio readall */

    if (wiringPiSetupGpio() < 0)
        return 1;

    pinMode(approved_bcm_pin, OUTPUT);
    digitalWrite(approved_bcm_pin, HIGH);
    delay(100);
    digitalWrite(approved_bcm_pin, LOW);
    return 0;
}
```

Build:

```sh
gcc -Wall -Wextra -o gpio-example gpio-example.c -lwiringPi
```

The placeholder must be replaced with a pin approved for the exact board. A successful compile is not a hardware validation result.
