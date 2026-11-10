# ADALM-PLUTO support — branch `adalm-pluto`

This branch carries the work to make `ogn-rf` run on an **ADALM-PLUTO** (PlutoSDR, an AD9363
transceiver behind libiio) instead of an RTL-SDR dongle. Design rationale, measurements and the
hardware bring-up log live in the companion repository
[`thread3d/ADALM-Pluto`](https://github.com/thread3d/ADALM-Pluto); this file is the part that belongs
next to the code.

**Status: the radio works and its numbers are measured; the backend is still to be written.**

As of 2026-11-10 the ADALM-PLUTO on `xavieragx.local` receives: it opens through libiio over USB,
tunes, captures 12-bit I/Q at 4 MSPS, and reads FM broadcast at 100 MHz **+23 dB** above the noise
floor it sees on the 868.4 MHz OGN channel. Everything this design was waiting on is now measured
rather than inferred — the rate floor, the gain range, the sample format and the frequency trim.

Two things still keep `plutosdr.h` out of this branch: the backend should be written once against
those measurements rather than as another draft, and the one remaining unknown that changes code —
how ogn-rf's own DSP behaves at the decimated rate — cannot be answered until the backend exists.

## How it fits in

`master` knows one radio. `ogn-rf.cc` declares `RTLSDR SDR;` and calls `SDR.Open()`, `SDR.Read()`,
`SDR.setCenterFreq()`, `SDR.setTunerGain()`; `rtlsdr.h` implements all of it by forwarding to
librtlsdr. Each device branch adds one file between those two things — `sdr.h`, a `SDR_Device` class
that keeps the identical method names and forwards to whichever backend is selected — and then a
header per radio:

| Branch | Adds |
|---|---|
| `limesdr` | `sdr.h`, `limesdr.h` (SoapySDR), `docs/limesdr.md` |
| `rspduo` | `sdr.h`, `limesdr.h`, `rspduo.h` (SDRplay API), `docs/rspduo.md`, `docs/limesdr.md` |
| `krakensdr` | `docs/krakensdr.md`, an FFTW change |
| **`adalm-pluto`** | `sdr.h`, `plutosdr.h` (libiio), this file |

The dispatch layer is a hand-written switch, not virtual inheritance — the file explains why, and
the reasoning still holds: the classes are cheap, only one is ever open, and every call site keeps
saying `SDR.something(...)` with no indirection to reason about. A backend that is not compiled in is
`typedef`'d to `RTLSDR` as an inert placeholder, so a plain RTL-SDR build is unaffected.

Five places know the list of radios, and they all have to learn about a fourth:

| Place | Today | The Pluto adds |
|---|---|---|
| `enum` | `{ RTL_SDR=0, LIME_SDR=1, RSP_DUO=2 }` | `PLUTO_SDR=3` |
| members | `RTLSDR RTL; LimeSDR Lms; RSPDuo Rsp;` | `PlutoSDR Pluto;` |
| guard | `HaveLimeSDR()`, `HaveRSPDuo()` | `HavePlutoSDR()`, true iff `USE_PLUTOSDR` |
| `getTypeName()` | `"RTL-SDR"`, `"LimeSDR"`, `"RSPduo"` | `"PlutoSDR"` — this is what the status page prints |
| `getTypeByString()` | parses `RF.DeviceType` | `"pluto"`, `"plutosdr"`, `"adalm-pluto"` |

The Makefile block follows `USE_RSPDUO` literally, including its reason for keeping the defines out of
`FLAGS` (a `make FLAGS=...` on the command line would otherwise drop them and produce a binary that
cannot select the radio):

```make
# ADALM-PLUTO support through libiio (see plutosdr.h and sdr.h). Auto-detected from the libiio
# headers, so any other build is unaffected; force it either way with USE_PLUTOSDR=1 or 0.
ifndef USE_PLUTOSDR
ifneq ("$(wildcard /usr/include/iio.h)","")
USE_PLUTOSDR = 1
endif
endif

ifeq ($(USE_PLUTOSDR),1)
PLUTO_FLAGS = -DUSE_PLUTOSDR
PLUTO_LIBS  = -liio
endif
```

## The three things that make this backend different

### 1. It is a libiio device, not a driver

There is no `rtlsdr_open()` here and no vendor SDK to install. The Pluto exposes itself as an IIO
device and `libiio` is the entire interface: a context, then devices, channels and *attributes*.
The consequence for the code is that nothing is a function call with a typed result — you write a
string to an attribute and read another back to find out what the hardware actually did. Two habits
follow from that:

- **Read back what was set.** `sampling_frequency` and `hardwaregain` after writing them, because the
  driver snaps both to what the chip can do and does not complain.
- **Prefer the device's own answer to the datasheet's.** `sampling_frequency_available` is the truth,
  which is why the rate question below is stated as an open question and not a conclusion.

The two devices that matter are `ad9361-phy` (the transceiver's own controls) and `cf-ad9361-lpc`
(the RX DMA that hands over samples).

### 2. 12 bits in, 8 bits out

`ogn-rf`'s DSP was written for the RTL-SDR's stream: unsigned 8-bit I/Q, interleaved, `128` = zero.
The AD9363 produces 12-bit signed samples, delivered in 16-bit containers.

So `Read()` is a conversion, not a copy, and the scale is a **calibration with real consequences** —
exactly the lesson the `rspduo` branch already paid for, where the conversion scale turned out to be
a shift of 8 and had to be measured (peak −3.0 dBFS, zero clipping, full 16-bit range in use) rather
than derived. Too small a shift and the OGN pulses fold at the peaks; too large and the quantisation
noise costs the weak ones. The right value here is to be found by injecting a known level and
sweeping gain with `pluto_probe`, and written down where the constant is defined.

### 3. The rate: the chip produces exactly 2.000 MSPS, with an FFT that divides by a power of two

**`ad9361_set_bb_rate(2000000)` makes the chip output 1 999 999 Hz with no host-side resampling.**
The backend calls that, and nothing else touches the rate.

What this replaced is worth recording, because this branch's own text argued the opposite for a while.
The plain `sampling_frequency` attribute bottoms out at **2 083 333 Hz** (ADC 25 MHz, HB stages ÷12),
and 1.0 and 2.048 MSPS are rejected through it. `ad9361_set_bb_rate()` is a different path: it designs
and loads an **RX FIR** with decimation up to ÷4 on top of the HB stages, and reaches far below that
floor. Measured on the target, libad9361 0.2-1:

| `ad9361_set_bb_rate()` | Applied | ADC clock | Chain |
|---|---|---|---|
| 1 000 000 | 999 999 Hz | 32 MHz | ÷2 ÷2, FIR ÷4 |
| **2 000 000** | **1 999 999 Hz** | **64 MHz** | ÷2 ÷2, FIR ÷4 |
| 2 083 333 | 2 083 332 Hz | 66.7 MHz | ÷2 ÷2, FIR ÷2 |

**Exactly 2.000 rather than 2.083 decides itself, and not for any reason about USB.** `Inp_FFT::Preset()`
computes `FFTsize = (8*8*SampleRate)/15625`:

| SampleRate | OGN FFTsize | factorisation |
|---|---|---|
| 1 000 000 | 4096 | 2¹² |
| **2 000 000** | **8192** | **2¹³** |
| 2 048 000 | 8388 | 2²·3²·233 |
| 2 083 333 | 8533 | 7·23·53 |

At 2.0833 MSPS the demodulator would run a 8533-point FFT whose size has 23 and 53 in it, which FFTW
handles with generic codelets; at 2.000 it is a pure power of two. (Note `8*8`, not `8` — the `8*`
form is `GSM_FFT`, the GSM calibration spectrogram.)

**Bandwidth: 4 MHz.** Unaided, `ad9361_set_bb_rate` picks 1 MHz; 4 MHz measured about 5 dB stronger
at 100 MHz (rms 11.5 vs 6.5 %FS), and the driver's 18 MHz default is worse still. There is no reason
to open the analogue filter wide to "encourage a high sample rate" — the ADC already runs at 64 MHz.

Measured end to end: **99.9 % delivered, +2 ms overhead, 1.998 Msps sustained, 8.0 MB/s, rms 0.7 %FS,
crest 5.34, nothing clipped**, with `filter_fir_en = 1`.

Two implementation notes: call `ad9361_set_bb_rate()` and **do not** then write `sampling_frequency`,
which undoes the FIR setup it just performed; and the FIR costs a few samples of group delay, which
matters when computing the timestamp of a time slot.

USB remains the ceiling for anything faster — it saturates near 31 MB/s / 8 MSPS — but at 2.0 MSPS
that is 8 MB/s and irrelevant.

## The attribute map

Every method below is already called by `ogn-rf.cc` or by the status page, so the backend has to
answer all of them or the branch does not link.

| Method | IIO equivalent |
|---|---|
| `Open(idx, Freq, SampleRate)` | create the context, find `ad9361-phy` and `cf-ad9361-lpc`, tune, set rate/bandwidth, create the buffer |
| `Close()` | destroy buffer, destroy context |
| `isOpen()` | context != NULL and buffer exists |
| `Read(uint8_t*, Samples)` | `iio_buffer_refill()`, then convert 12-bit to 8-bit unsigned — see above |
| `ResetBuffer()` | refill and discard until empty, so a retune does not hand over stale data |
| `setCenterFreq()` / `getCenterFreq()` | `altvoltage0` → `frequency` on `ad9361-phy` |
| `setSampleRate()` / `getSampleRate()` | `voltage0` → `sampling_frequency`; the floor is **2 083 333 Hz** and the value to request is 2 083 334 |
| `setFreqCorrection()` / `getFreqCorrection()` | device attribute `xo_correction` — a **real** trim, measured ±200 ppm at 1 Hz resolution on a 40 MHz XO (`xo = 40e6 + 40 x ppm`) |
| `setTunerGain()`, `getTunerGain()`, `getTunerGains()` | `voltage0` → `hardwaregain`, with `gain_control_mode` = `manual` |
| `setTunerGainMode()` | `gain_control_mode` — **a channel attribute of `voltage0` (input)**, not a device attribute; values `manual fast_attack slow_attack hybrid` |
| `setOversample()` | chip rate = `Ratio ×` requested rate — the core of the rate design above |
| `getTime()` | `clock_gettime()`, unchanged from `rtlsdr.h` |
| `getTunerType()`, `getDeviceName()`, `getUsbStrings()` | from the context and the device's own strings; the status page prints these |
| `getDeviceIndexBySerial()` | index the `usb:` contexts by the ADI serial (here `104473ae73930001fdff27009d682d6d06`) |

## Options that do nothing here

The RSPduo branch has a section for these and the Pluto needs one for the same reason — a config key
that is silently ignored is worse than one that is rejected:

- **`BiasTee`** — accepted and ignored. There is no bias tee: a Pluto is a transceiver on a bare
  board, with no antenna switch and no LNA supply.
- **`OfsTune`** — accepted and ignored. There is no offset-tuning mode.
- **`FreqCorr`** — a real hardware correction. An earlier version of this document said the AD9363
  has no ppm register; that was wrong. `xo_correction` is a device attribute, measured at
  39 999 890 Hz with a range of 39 991 891 to 40 007 889 Hz: ±200 ppm at 1 Hz (0.025 ppm)
  resolution on the 40 MHz XO.
- **`FreqRaster`** — worth setting to 1 (no rounding). The AD9363 tunes to well under 1 Hz.

## Building it on this board

Verified on the Xavier AGX, because it is not what one would assume:

| Dependency | Where it is |
|---|---|
| `libiio` 0.19 | **system** — `/usr/include/iio.h`, `/usr/lib/aarch64-linux-gnu/libiio.so`; `-liio` works |
| `libconfig`, `librtlsdr` | **not system packages** — built from source by the KrakenSDR work into `~/projects/KrakenSDR/krakensdr/deps/prefix` |
| `libfftw3-dev`, `libjpeg-dev` | system, installed |

So a Pluto build still needs the prefix, and it still needs librtlsdr's *headers* even though it will
never open a dongle, because `sdr.h` includes `rtlsdr.h` unconditionally and every device
configuration links `-lrtlsdr`:

```bash
P=$HOME/projects/KrakenSDR/krakensdr/deps/prefix
make USE_PLUTOSDR=1 \
     FLAGS="-Wall -O3 -ffast-math -I$P/include" \
     LIBS="-L$P/lib -Wl,-rpath,$P/lib -lpthread -lm -ljpeg -lconfig -lrt"
```

## What hardware has already settled

| Question | Answer | Effect on the design |
|---|---|---|
| Rate floor | `[2083333 1 61440000]`; 2083333 rejected, 2083334 applied as 2 083 333; 1.0 and 2.048 MSPS rejected; 4.0 MSPS → 3 999 999 | decimation is mandatory, and 4 MSPS ÷ 2 = 2.0 MSPS is the shape |
| Gain | `[-1 1 73]`, 1 dB steps, and **not** linear in dB; nothing clips at 73 dB on a quiet band | use the measured table, not a formula; the shipped config asks for 73 |
| Format | 12-bit signed, shift 0 | shift by 4 for 8-bit, then calibrate |
| USB face | interface 5 is IIO and opens reliably through libusb; `rndis_host`/`cdc_acm` are not loaded, so the network gadget is not an option | USB transport, and a `usb:` context found **by serial** because the address in the URI changes on every replug |
| Attribution | `gain_control_mode` and `hardwaregain` are channel attributes; `xo_correction`/`ensm_mode` are device attributes | write to the channel, or fail silently |

## What is still open

1. **ogn-rf's DSP at the decimated rate** — `SpectrogramFFTsize=(8*SampleRate)/15625` is not a power
   of two at 2.0 or 2.083 MSPS, and has never been exercised. Most likely to need a change here
   rather than in the backend.
2. **The 8-bit conversion shift** — a calibration against a known level, as the `rspduo` branch had to
   do when it settled on a shift of 8.
3. **`ResetBuffer()` after a retune** — how much stale data a libiio refill can hand over, which is
   what turns into a "Half time slot" warning when it is wrong.
4. **Sustained throughput** — 0.5 s at 4 MSPS (16 MB) is fine; a whole `SensTime` window every
   30 seconds over 480 Mbit/s behind two hubs is untested.

## Why the backend is not in this branch yet

Not for want of a radio. The measurements above are the ones that decide its shape, and they arrived
late; writing `plutosdr.h` now means writing it once, against real numbers, instead of the four
drafts this document went through while the answers kept moving. That is the whole reason the branch
has a design document and no code.

The repository that produced the measurements is [`thread3d/ADALM-Pluto`](https://github.com/thread3d/ADALM-Pluto):
`scripts/probe-pluto.sh`, `src/pluto_probe.c` and `docs/hardware.md` hold the tooling and the tables.

## Licence
`ogn-rf` is **GPL v3**. `plutosdr.h` will be part of it and carries the same licence.
