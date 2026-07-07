# Dual-Radar-Board-TEST

Dual-radar presence-detection firmware for PSoC 6. An Infineon
CY8CPROTO-062S3-4343W (CY8C6245LQI-S3D72, Cortex-M4) drives two XENSIV
BGT60TR13C 60 GHz FMCW radar sensors over a shared SPI bus and runs an
on-device CMSIS-DSP presence pipeline (Range FFT, macro inter-frame delta
detection, Doppler FFT micro-motion scoring, RX2/RX3 phase-based azimuth
angle) under FreeRTOS. Results are emitted as a compact 20-byte binary frame
over UART, decoded to JSON by an Arduino UNO R4 Minima bridge, and validated
offline by a host-side Python replay of the same algorithm.

See [PROJECT.md](PROJECT.md) for the detailed, auto-generated project map
(architecture overview and per-module tables in [project-table/](project-table/)).

## Repository layout

| Path | Contents |
|------|----------|
| `src/` | PSoC 6 application firmware: FreeRTOS tasks, dual-radar SPI driver (`radar_task.c`), CMSIS-DSP presence pipeline (`presence_detection.c`), V3 frame serialization over UART (`usb_logging.c`), wire protocol (`radar_protocol.h`), generated chirp registers (`presence_radar_settings.h`) |
| `A7-RADAR-TEST/` | PlatformIO sketch for an Arduino UNO R4 Minima: UART→USB bridge that parses V3 frames and prints one JSON object per frame |
| `bgt60-configurator-cli/` | Infineon BGT60 register configurator CLI + JSON chirp configs (`config_1400mhz.json`, `config_2ghz.json`) |
| `data_test/` | Host-side Python tools: serial capture, binary frame decoding, and an offline NumPy/SciPy replay of the presence algorithm |
| `bsps/` | Board Support Package for `APP_CY8CPROTO-062S3-4343W` |
| `deps/`, `libs/` | ModusToolbox dependency manifests (`.mtb`) and resolved library closure |
| `templates/` | Archived BSP configs for alternative PSoC 6 boards + `TEST-000` single-radar baseline |
| `imports/` | Legacy emUSB-Device CDC artifacts (excluded from the active build) |

## Build and flash (PSoC 6 firmware)

The firmware is a ModusToolbox `COMBINED` make application
(`TARGET=APP_CY8CPROTO-062S3-4343W`, `TOOLCHAIN=GCC_ARM`,
`APPNAME=Dual-Radar-Board-TEST`). Shared libraries are resolved into
`../mtb_shared`. From a ModusToolbox shell (modus-shell on Windows) in the
repo root:

```sh
make getlibs      # fetch/refresh dependencies (first build only)
make build        # compile; output in build/
make program      # build and flash via KitProg3
```

VS Code debugging uses the Cortex-Debug launch configs in
`.vscode/launch.json` (OpenOCD via `openocd.tcl`), which load
`build/last_config/Dual-Radar-Board-TEST.elf`.

The A7 bridge is a separate PlatformIO project — build/upload it from
`A7-RADAR-TEST/` with the `uno_r4_minima` environment.

## V3 binary protocol

Defined in `src/radar_protocol.h` (and mirrored in the A7 bridge). Fixed
20-byte little-endian frame, emitted every 3 s:

```
[0xAA][0x55][LEN:u16=14][TYPE:u8=0x03][SEQ:u8]
[record0: 6 bytes][record1: 6 bytes]
[CRC16:u16]
```

- One 6-byte record per sensor: `sensor_id (u8)`, `flags (u8)` (bit 0 macro
  presence, bit 1 micro presence), `distance_mm (u16)`, `angle_cdeg (i16)`.
- Sentinels mark "no target": `distance_mm = 0xFFFF`,
  `angle_cdeg = INT16_MIN`.
- CRC is CRC-16/CCITT-FALSE over bytes 0–17 (sync through last payload byte).
- `LEN` counts TYPE + SEQ + payload; re-sync by scanning for `0xAA 0x55`.

The A7 bridge decodes each valid frame to JSON, e.g.
`{"seq":N,"s":[{"id":0,"present":true,...},{...}]}`.

## Radar chirp configuration

The BGT60TR13C register set embedded in `src/presence_radar_settings.h`
(`register_list[]`, 38 registers) is generated offline — do not hand-edit it.
Edit one of the JSON configs in `bgt60-configurator-cli/` and regenerate:

```sh
cd bgt60-configurator-cli
./bgt60-configurator-cli.exe -c config_1400mhz.json
```

- `config_1400mhz.json` — current config: 60.55–61.95 GHz (1.4 GHz bandwidth,
  ~0.107 m range resolution, ~6.85 m span), 128 samples/chirp, 32
  chirps/frame, 2 MHz ADC sample rate, ~150 ms frame repetition.
- `config_2ghz.json` — earlier 2.0 GHz-bandwidth config (~0.075 m resolution).

Copy the generated register list into `presence_radar_settings.h`.

## Host-side analysis (`data_test/`)

Python 3.13+ project (numpy, scipy, matplotlib, pyserial — see
`data_test/pyproject.toml`):

- `src/serial_logger.py` — capture raw serial output from the board
- `src/format_binary_frames.py` — decode captured V3 binary frames
- `src/radar_process.py`, `src/radar_process_test_001.py` — offline replay of
  the Range FFT → Doppler FFT → presence algorithm against captured frame
  fixtures, for threshold tuning and regression checks
