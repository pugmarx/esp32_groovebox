# ESP32 8-Track Groovebox

8-track, 16-step standalone groovebox running on an ESP32 with 16-bit I2S audio, hardware controls, and a 1.3" OLED interface.

---

## Hardware Modules

- **Microcontroller**: ESP32-WROOM-32 (NodeMCU / DevKit V1)
- **Audio DAC**: PCM5102A 32-bit I2S DAC Module (with 3.5mm stereo line out)
- **Display**: 1.3" SH1106 128x64 Monochrome OLED (SPI)
- **Controls**: 
  - 4x Rotary Encoders / Potentiometers
  - 2x Momentary Push Buttons
  - 1x Beat LED (GPIO 2 onboard or external)

---

## Wiring & Schematic

```
                          ESP32-WROOM-32
                     +----------------------+
                     |                      |
                     |  [PCM5102A DAC]      |
               GPIO26|----> BCK             |
               GPIO25|----> LRCK / WS       |
               GPIO27|----> DIN / DOUT      |
                  GND|----> GND             |
                  VIN|----> VCC (5V / 3.3V) |
                     |                      |
                     |  [SH1106 SPI OLED]   |
               GPIO16|----> DC              |
                GPIO4|----> CS              |
               GPIO17|----> RESET           |
               GPIO18|----> SCK / CLK       |
               GPIO23|----> MOSI / DIN      |
                  3V3|----> VCC             |
                  GND|----> GND             |
                     |                      |
                     |  [INPUTS & CONTROLS] |
               GPIO34|----> NAV Encoder CLK |
               GPIO35|----> NAV Encoder DT  |
                GPIO5|----> KEY Transpose CLK
               GPIO13|----> KEY Transpose DT|
               GPIO33|----> DECAY Knob CLK  |
               GPIO14|----> DECAY Knob DT   |
               GPIO21|----> TEMPO Knob CLK  |
               GPIO19|----> TEMPO Knob DT   |
               GPIO22|----> TOGGLE STEP Btn |
               GPIO32|----> NEXT TRACK Btn  |
                GPIO2|----> BEAT LED        |
                     +----------------------+
```

*Note: All buttons and encoder pins use internal `INPUT_PULLUP` to GND.*

---

## Versions

- **`seq_te_sleek_groovebox/`** *(Recommended / Latest)*:
  - 16-bit 44.1 kHz I2S audio with studio soft-limiter (no clipping).
  - 16 x 256 DMA buffer to prevent underruns during SPI screen refresh.
  - Stereo ping-pong delay line (250ms tempo-synced).
  - MPC-style 58% swing shuffle timing.
  - Natural velocity humanization (±7%) and decay drift (±8%).
  - 8 distinct voices: Kick (pitch-sweep 808), Hi-Hat (band-passed noise), Sub-Bass (harmonics-enriched 55Hz), Saw Bass, Pluck, Lead, Stab, High Chime.
  - Graphic UI: 16-step pads with ghost markers for other tracks, track activity circles, and an idle cassette tape screensaver.
- **`seq_4enc_1_3oled_8x16track_UI_hybrid_GOLDEN_1/`**:
  - Original baseline implementation.

---

## Controls

| Control | Pin(s) | Function |
|---|---|---|
| **NAV** | GPIO 34, 35 | Select active step (1–16) |
| **KEY** | GPIO 5, 13 | Semitone transpose (-12 to +12, Concert Pitch) |
| **DECAY** | GPIO 33, 14 | Adjust note decay / tightness (50ms to 600ms) |
| **TEMPO** | GPIO 21, 19 | Adjust BPM (60 to 200 BPM) |
| **TOGGLE** | GPIO 22 | Toggle current step on/off |
| **TRACK** | GPIO 32 | Cycle through tracks 1 to 8 |

---

## Arduino IDE Setup

- **Board**: `ESP32 Dev Module`
- **CPU Frequency**: 240MHz
- **Upload Speed**: 921600 (or 115200)
- **Required Libraries**:
  - `Adafruit_GFX`
  - `Adafruit_SH110X`
