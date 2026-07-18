# BPI-WiringPi2 Wiki

BPI-WiringPi2 provides WiringPi-compatible C APIs and the `gpio` command for Banana Pi Linux boards. This Wiki is the long-form documentation entry point for users, board maintainers and hardware validation teams.

## Start here

- New user: [Build and Install](Build-and-Install) → [Quick Start](Quick-Start)
- Check a board: [Board Support Matrix](Board-Support-Matrix)
- Understand pin numbers: [Numbering and Detection](Numbering-and-Detection)
- Add a board: [Porting a New Board](Porting-a-New-Board)
- Validate hardware: [Hardware Validation](Hardware-Validation)
- Known gaps: [Known Limitations and Roadmap](Known-Limitations-and-Roadmap)
- Maintain the docs: [Documentation Maintenance](Documentation-Maintenance)

## Support claims

The source currently contains 47 effective internal Banana Pi model IDs and 231 detection names. These numbers describe code coverage, not complete hardware validation. A board can be detected and mapped while pull control, alternate functions, hardware PWM, expander GPIO or exact device nodes remain limited.

The public Banana Pi catalog contains products that are not appropriate targets for this library. Modules without an exact carrier, MCU boards, camera products, isolated industrial DI/DO and boards without an intended user GPIO connector are recorded separately instead of receiving guessed mappings.

## Companion project

Python users should use the [BPI-SINOVOIP/RPi.GPIO Wiki](https://github.com/BPI-SINOVOIP/RPi.GPIO/wiki). Board detection and physical/BCM maps must stay aligned across both repositories, while API-specific behavior is documented separately.

> Safety: never drive a physical pin until the exact board, carrier, revision, voltage domain and pin map have been confirmed.
