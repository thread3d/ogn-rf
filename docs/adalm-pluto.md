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

### 3. The chip decimates on-chip already, and USB is the ceiling

The constraint that decides the backend's architecture, now measured rather than reasoned about.

`ogn-rf` asks for 1.0 MSPS by default (`ogn-rf.cc`) and 2.048 MSPS by inheritance from the RTL-SDR
(`rtlsdr.h`, `sdr.h`). The chip's floor is **2 083 333 Hz**, so both are unreachable — and so is
2.000 MSPS. One software step is therefore unavoidable to land on exactly 2.0.

What is worth stating plainly, because the opposite is easy to assume: **the chip decimates on every
setting, and a higher sample rate does not put more work on it.** `rx_path_rates` shows the chain: at
4 MSPS the ADC runs at 32 MHz and the filters are ÷8; at 61.44 MSPS the ADC runs at 491.52 MHz with
the *same* ÷8. Only the minimum data rate gets the extra ÷12. And **USB saturates near 8 MSPS /
31 MB/s** — requests for 15.36, 30.72 and 61.44 MSPS all delivered about 8 Msps and 31 MB/s, so the
link, not the chip, is what limits a high rate.

| Chip data rate | On-chip | Software | Into ogn-rf | USB | Measured (3 repeats, >=2 s) |
|---|---|---|---|---|---|
| **2 083 334** | **÷12** | **none** | **2 081 065** | **8.3 MB/s** | **99.8-99.9 %, +2-4 ms — clean** |
| 4 000 000 | ÷8 | ÷2 box | 1 997 777 | 15.0 MB/s | 99.9 %, +2-3 ms — clean |
| 6 000 000 | ÷8 | ÷3 box | 1 997 986 | 23.0 MB/s | 99.9 %, +2-3 ms — clean |
| 8 000 000 | ÷8 | ÷4 box | ~1 950 000 | 31.0 MB/s | 95.2-95.6 % — **drops 2-5 % of samples** |
| 10 000 000 | ÷8 | ÷5 box | 1 580 000 | over capacity | 78.8 % — real output collapses |
| 16 000 000 | ÷8 | ÷8 box | 990 000 | over capacity | 49.5 % — real output collapses |

The three clean rows are indistinguishable on delivery, so the choice is not about throughput at all —
it is about **where the last factor happens**. Having the chip do all of it (`2 083 334` / ÷1) puts
2.081 MSPS on the wire from 8.3 MB/s and asks nothing of the host, at the cost of the rate being
2.0833 rather than 2.000, which makes `SpectrogramFFTsize` 1066 instead of 1024. Taking the last
factor in software lands exactly on 2.000 MSPS for twice the USB traffic.

The shipped configuration takes the first option and the backend should implement both, because the
difference between them is one config key, not one code path.

`sdr.h` already has the hook — `setOversample(Ratio)`, added for the RSPduo's fixed 2/6/8 MSPS ADC —
and this is precisely what it was for: `setOversample(3)` at a 6 MSPS chip rate.

One caveat the backend should keep: the ~31 MB/s link is shared with any other SDR on the board. The
device-decimation configuration uses 8.3 MB/s and is the friendliest to a busy bus; the software
options use 15-23 MB/s for the same ~2 MSPS output.

The tie-breaker is arithmetic that is not arbitrary: **8 x 2 000 000 / 15 625 = 1024 exactly**, so
ogn-rf's spectrogram is the power-of-two FFT its author wrote it against. At 2 083 333 the same
expression gives 1066. And 2.0 MSPS is the rate the `rspduo` branch already settled on, so the
frequency plan and gain measurements from that work stay comparable.

Validated on hardware: `-r 6000000 -D 3 -n 12000000` produced 4 000 000 samples at a nominal
2 000 000 Hz over 2.000 s, 99.5 % delivered, rms 0.6 %FS, crest 5.33, nothing clipped. The filter is
the same shape as ogn-rf's own `boxfilter.h`.

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
