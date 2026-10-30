# Pott - X9C103S digital potentiometer (ESP32, ESP-IDF, C++)

Control of an X9C103S (10 kOhm, 100 taps) with push buttons.

- Calibration at every power-up: the wiper is driven to the maximum (position 99, ~10 kOhm).
- UP / DOWN buttons change the resistance (hold to auto-repeat); STORE saves the position to the chip's non-volatile memory.

## Wiring
| Signal | ESP32 GPIO |
|---|---|
| CS | 25 |
| INC | 26 |
| U/D | 27 |
| BTN UP | 32 (to GND) |
| BTN DOWN | 33 (to GND) |
| BTN STORE | 14 (to GND) |

## Build
```
idf.py set-target esp32
idf.py build flash monitor
```
