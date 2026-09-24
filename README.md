# On-device keyword spotting on STM32F407

A microcontroller that listens through its own microphone and recognizes eight spoken
English commands (`yes, no, up, down, left, right, go, stop`), entirely on-device:
no cloud, no PC in the loop. The result is shown on the board's LEDs and logged over UART.

**Board:** STM32F4DISCOVERY (STM32F407VGT6, Cortex-M4F @ 168 MHz, 1 MB flash, 192 KB RAM)

## Pipeline

```
MEMS mic (PDM, 1 MHz) -> I2S + DMA (double buffer) -> PDM2PCM (16 kHz)
  -> VAD (adaptive noise floor, pre/post-silence gates)
  -> MFCC (49 x 10) -> int8 DS-CNN (X-CUBE-AI, 10 classes) -> decision -> LED / UART
```

Audio capture and PDM-to-PCM conversion run continuously; MFCC and the model only run
when speech is detected, once per word. The VAD looks back 250 ms in a ring buffer so the
word lands in the middle of the 1 s analysis window.

## Current model (v3): open-set DS-CNN

10 classes: the 8 commands + `_silence_` + `_unknown_`. Trained on the full Google Speech Commands v2
(other 27 words as "unknown", background noise as "silence") plus board-microphone recordings
(119 commands, 100 non-command utterances), with time-shift, gain and background-noise augmentation.

| | v2 (8-class CNN) | v3 (10-class DS-CNN) |
|---|---|---|
| Weights (flash) | 14.2 KB | 14.8 KB |
| Activations (RAM) | 9.2 KB | 12.6 KB |
| MACs | 0.86 M | 1.62 M |
| Latency @ 168 MHz | ~22 ms | 32.5 ms (3.4 cycles/MAC) |
| False accepts on Speech Commands v2 test negatives (unknown + silence) | 40.1 % | 4.3 % |
| Command recall on Speech Commands v2 test | 74.1 % | 83.0 % |
| Speaker's held-out commands (24) | 95.8 % | 100 % |

Decision rule for both: top class is a command, confidence >= 70 % (v2: 75 %), top-2 margin >= 40 points.

Live test with v3 and the timing gates: 0 false accepts in ~2.5 minutes of continuous Turkish speech.
Known remaining confusions: isolated "L" / "les" -> `left`, "top" -> `stop` (near minimal pairs).

## First measurements: FP32 vs int8 (v1, 8-class CNN)

| | FP32 | int8 |
|---|---|---|
| Test accuracy (mini Speech Commands, 8 words) | 88.25 % | 88.5 % |
| Model weights | 55.9 KB | 14.2 KB |
| Inference latency @ 168 MHz (DWT cycle counter) | 45.3 ms | 20.4 ms |
| Cycles per MAC | 8.8 | 4.0 |
| Total flash incl. runtime library | 89.0 KB | 95.5 KB |

- PDM-to-PCM conversion: 0.57 ms per 16 ms of audio (3.6 % CPU)
- MFCC, 49 frames: 148 ms; max difference vs. the TensorFlow reference: 4e-6
- Generated C model is bit-exact with the reference TFLite model (X-CUBE-AI desktop validation)
- Better data capture (VAD-centred windows) alone raised accuracy on board-microphone recordings
  from ~65 % to 88 % with the same model; fine-tuning with 119 board recordings reached 24/24 on a
  small speaker-specific held-out set.

## Repository layout

```
firmware/mic_test/   STM32CubeIDE project (mic_test.ioc, Core/, X-CUBE-AI/, CMSIS_DSP/, Middlewares/)
notebooks/           01 data + MFCC, 02 speaker fine-tuning, 03 open-set training (unknown/silence, DS-CNN)
tools/               kayit_al.py (labelled recording), otomatik_kayit.py (bulk negatives),
                     mfcc_dogrula.py (board vs. PC MFCC check)
models/              kws_int8.tflite (v1 baseline), kws_int8_v2.tflite (speaker-adapted),
                     kws_int8_v3.tflite (current, 10-class open-set DS-CNN)
```

Key source files: `Core/Src/mfcc.c` (MFCC, CMSIS-DSP FFT), `Core/Src/kws_model.c`
(normalization, int8 quantization, inference), `Core/Src/main.c` (audio pipeline, VAD, gates, UART),
`Core/Inc/mfcc_consts.h` (window, mel filters, DCT, normalization; exported from TensorFlow).

## Build and run

1. Install STM32CubeIDE (tested with 2.2.0). Import `firmware/mic_test` as an existing STM32CubeIDE project.
2. Build and flash. The model, runtime libraries and CMSIS-DSP sources are included.
3. Serial log: connect a 3.3 V USB-TTL adapter (adapter RX -> PA2, TX -> PA3, GND -> GND),
   then `screen /dev/tty.usbserial-* 115200`.
4. Reset the board, stay silent for 1 s (noise floor calibration), then speak a command.

`RECORD_MODE` in `main.c`: `0` = live recognition, `1` = data collection (press the blue button,
speak when the orange LED turns on; the clip is sent over UART to `tools/kayit_al.py`).

To regenerate CubeMX code, open `mic_test.ioc` in STM32CubeMX with X-CUBE-AI 10.2.1.
User code lives in `USER CODE` blocks and separate files and is preserved.

## LED mapping

`up` orange (top) · `down` blue (bottom) · `left` green (left) · `right` red (right) ·
`go` green + red · `stop` orange + blue · `yes` all four · `no` none.
`_silence_` and `_unknown_` never light an LED (logged as `KOMUT DEGIL`).

## False-accept defence (layered)

1. **Timing gates (firmware):** a command needs ~250 ms of near-silence before it and must end
   within the 1 s window; continuous speech is rejected before the model runs.
2. **Open-set model:** `_unknown_` and `_silence_` classes let the model reject non-commands.
3. **Decision rule:** confidence and top-2 margin thresholds.

## Limitations and next steps

- Near minimal pairs ("L"/"les" vs `left`, "top" vs `stop`) can still be accepted when spoken in
  isolation. Next: per-class thresholds, targeted hard-negative recordings.
- Speaker-adapted; test sets are small. A wake word or confirmation is needed for critical loads.
- Next: MFCC profiling and a sparse mel filter bank; 4-bit quantization-aware training (HADQ-Net)
  evaluated on real hardware as a senior capstone project.

## Licenses

- The author's own code (`Core/Src/mfcc.c`, `Core/Src/kws_model.c`, user code in `main.c`,
  `Core/Inc/mfcc_consts.h`, `notebooks/`, `tools/`) is MIT licensed, see `LICENSE`.
- STMicroelectronics components (HAL, CMSIS device files, X-CUBE-AI runtime and generated code,
  PDM2PCM library) are distributed under their own licenses, included next to each component
  (e.g. `firmware/mic_test/LICENSE_X-CUBE-AI.txt`).
- CMSIS-DSP (`firmware/mic_test/CMSIS_DSP/`) is under the Apache 2.0 license (Arm).
- Training data: Google Speech Commands (CC BY 4.0).
