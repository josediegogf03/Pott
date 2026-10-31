# Pott - X9C103S digital potentiometer (ESP32, ESP-IDF, C++)

Control of an X9C103S digital potentiometer with push buttons. The module used here measures as a
32-position pot (~350 ohm per step, ~10.9 kOhm at the top), not the datasheet's 100 positions;
set `POT_TAPS` / `POT_TOTAL_OHM` in `main/main.cpp` for a different chip.

- Calibration at every power-up: the wiper is driven to the maximum (position 31, ~10.9 kOhm).
- UP / DOWN buttons change the resistance (hold to auto-repeat); STORE saves the position to the chip's non-volatile memory.

## Wiring
| Signal | ESP32 GPIO |
|---|---|
| CS | 27 |
| INC | 19 |
| U/D | 18 |
| BTN UP | 32 (to GND) |
| BTN DOWN | 33 (to GND) |
| BTN STORE | 14 (to GND) |

## Build
```
idf.py set-target esp32
idf.py build flash monitor
```
