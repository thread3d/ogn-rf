# ADALM-PLUTO support — branch `adalm-pluto`

This branch carries the work to make `ogn-rf` run on an **ADALM-PLUTO** (PlutoSDR, an AD9363
transceiver behind libiio) instead of an RTL-SDR dongle. Design rationale, measurements and the
hardware bring-up log live in the companion repository
[`thread3d/ADALM-Pluto`](https://github.com/thread3d/ADALM-Pluto); this file is the part that belongs
next to the code.

**Status: design only. No backend is in this branch yet, and nothing has been received.**

That is a deliberate choice rather than an accident of time. The radio currently falls off the USB
bus four seconds after it enumerates, so every number that would decide the shape of the backend —
the chip's real rate floor, the gain range, the working conversion scale — is unread. A backend
written now would be a set of guesses wearing a `.h` extension, and the pattern from the other device
branches is that the expensive part is not writing the code, it is discovering which guess was wrong.
The companion repository's `docs/status-log.md` records exactly what is known and what is blocked.

When the link is stable, `plutosdr.h` goes in and this branch changes from a document into a
backend. Until then, this file is the contract it will have to meet.

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

### 3. The chip cannot go where ogn-rf wants to go

This is the constraint that decides the architecture of the backend, and it is not negotiable
software-wise. The AD9361/AD9363 family's decimating filters bottom out at roughly **2.08 MSPS**,
while `ogn-rf` asks for 1.0 MSPS by default (`ogn-rf.cc`) and 2.048 MSPS by inheritance from the
RTL-SDR (`rtlsdr.h`, `sdr.h`). Both are below the floor.

`sdr.h` already has the hook for this — `setOversample(Ratio)`, added for the RSPduo's fixed 2/6/8
MSPS ADC. The Pluto wants the same treatment:

| ogn-rf asks for | chip runs at | software decimation | USB traffic | note |
|---|---|---|---|---|
| 2 000 000 | 4 000 000 | ÷2 | 16 MB/s | **intended default** — exact integer ratio, a quarter of USB2 |
| 2 000 000 | 8 000 000 | ÷4 | 32 MB/s | if the ÷2 decimation turns out to alias |
| 1 000 000 | 8 000 000 | ÷8 | 32 MB/s | only if 1 Msps is genuinely wanted |
| 2 083 333 | 2 083 333 | ÷1 | 8 MB/s | ask ogn-rf to match the chip; least USB load, but the rate is unrelated to the OGN clock |

The first row is the recommendation, for a reason beyond arithmetic: 2.0 MSPS is **the same rate the
`rspduo` branch settled on**, so the frequency plan and the gain measurements from that work stay
directly comparable. The decimation itself is a two-tap box filter and `boxfilter.h` is already in
the tree.

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
| `setSampleRate()` / `getSampleRate()` | `voltage0` → `sampling_frequency` |
| `setFreqCorrection()` / `getFreqCorrection()` | no ppm register: applied as an offset on the LO |
| `setTunerGain()`, `getTunerGain()`, `getTunerGains()` | `voltage0` → `hardwaregain`, with `gain_control_mode` = `manual` |
| `setTunerGainMode()` | device attribute `gain_control_mode` (`manual`, `slow_attack`, `fast_attack`) |
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
- **`FreqCorr`** — a real correction, but not the RSPduo's kind. The AD9363 has no ppm register, so
  the value is applied as an arithmetic offset on the LO. It is honest, but it is ours, not the
  chip's, and it should be measured with `gsm_scan` before it is trusted.
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

## What has to be settled on hardware

In order of how much they change the design:

1. **The real rate floor** — `sampling_frequency_available` on `voltage0`. If it permits 2.0 MSPS
   directly, the decimation design collapses into a pass-through and the backend gets simpler.
2. **The conversion scale** — sweep gain against RMS and crest factor, and pick the shift that keeps
   the noise floor near 20-25 % of full scale with a crest factor around 9-11.
3. **The gain law** — the RSPduo's turned out to be `total = 115.8 − gRdB`; this one has to be
   measured the same way before any dB figure in a config file means anything.
4. **Which USB face the firmware offers**, and whether USB or the network gadget is the better
   transport for a one-second burst every thirty seconds. USB avoids depending on a DHCP lease.
5. **`ResetBuffer()` semantics** — libiio is a blocking read, which is closer to the RTL-SDR model
   than the RSPduo's callback API, but how much stale data a refill can deliver after a retune is
   precisely what produces a "Half time slot" warning when it is wrong.

## Why there is no backend in this branch yet

Because the radio does not stay on the bus. The Pluto enumerated once on 2026-11-10 at 10:44:55 as
`0456:B673`, presented its firmware volume as mass storage, reported `Power-on or device reset
occurred`, and disconnected at 10:44:59. Until that is fixed — the leading suspect is a brown-out,
since the device is bus-powered and sits behind three hubs — items 1, 2, 3 and 5 above are unreadable,
and each of them changes code.

What exists instead: the companion repository holds a read-only link diagnosis
(`scripts/probe-pluto.sh`), a libiio receive checker that compiles and behaves correctly against this
board's libiio 0.19 (`src/pluto_probe.c`), the udev rule for non-root access, and a configuration
whose every value either has a reason or is marked as unmeasured.

## Licence

`ogn-rf` is **GPL v3**. `plutosdr.h` will be part of it and carries the same licence.
