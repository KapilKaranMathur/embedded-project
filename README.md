# Arduino Syringe Pump

A single-channel, open-loop syringe pump developed as an embedded-systems
project for non-medical laboratory use.

## Hardware

- Arduino Mega 2560 and RAMPS 1.4
- A4988 driver and NEMA17 stepper motor
- HOME and MAX limit switches
- 20 kg load cell with HX711
- 20 mL and 50 mL syringes
- Laptop-based Chrome Web Serial dashboard

The Arduino owns motion, endstop handling, force protection and communication
watchdog behaviour. The dashboard provides configuration, telemetry,
calibration, hold-to-jog control and visual/audio alarms.

## Final project report

[Download the IEEE conference-format report](output/pdf/Syringe_Pump_Final_Report.pdf).

## Source layout

- `firmware/SyringePumpFirmware/` — Mega firmware and sensor/calibration modules
- `dashboard.html` — Chrome Web Serial dashboard
- `scripts/arduino.sh` — build and board tools

## Compile

```sh
bash scripts/arduino.sh compile
```

The build helper selects the Mega 2560 target and the pinned HX711 library.
The report includes the pin assignments and describes the safeguards and
bench validation still needed for this prototype.

## Current status

V4.5 is the latest verified upload to the Mega. V4.6 and the current dashboard
changes compile and pass simulations but still require controlled hardware
verification. The 20 mL delivery path has been reported accurate; final 50 mL
calibration and repeatability testing remain in progress.

## Contributors

- [Kapil Karan Mathur](https://github.com/KapilKaranMathur)
- [Pranjal](https://github.com/Pranjal9005)
