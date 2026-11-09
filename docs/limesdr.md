# LimeSDR support — branch `limesdr`

This branch makes `ogn-rf` able to drive a **LimeSDR-USB** (LMS7002M) instead of an RTL-SDR dongle,
selectable at run time through the configuration file. The hardware measurements, bring-up notes and
standalone verification tooling live in the companion repository
[`thread3d/LimeSDR`](https://github.com/thread3d/LimeSDR); this file is the part that belongs next
to the code.

**Status: the radio works.** On a Jetson AGX Xavier with a LimeSDR-USB on USB 2.0 the backend opens
the device, tunes 868.300 MHz to within 2 Hz, streams 1 MSps at 100 % of real time with no overruns,
and hands ogn-rf correctly scaled 8-bit I/Q (mean 127.5, no clipping). Time-slot acquisition runs
indefinitely. **Live OGN decoding is not yet demonstrated** — see "What is not proven" below.

## Changes

Two new headers, plus a small amount of plumbing:

| File | What it is |
|---|---|
| `limesdr.h` | `LimeSDR` — a SoapySDR backend mirroring the `RTLSDR` interface |
| `sdr.h` | `SDR_Device` — holds both backends, forwards to the selected one |
| `ogn-rf.cc` | `RTLSDR SDR` → `SDR_Device SDR`; parses `RF.DeviceType`; status page shows the radio |
| `Makefile` | auto-detects SoapySDR, adds `-DUSE_LIMESDR` / `-lSoapySDR` |
| `fft.h` | FFTW planning flag overridable, default `FFTW_ESTIMATE` (taken from `krakensdr`) |

`ogn-rf`'s acquisition code did not change: the whole point of `sdr.h` is that `RF_Acq` keeps
calling `SDR.Open()`, `SDR.Read()`, `SDR.setCenterFreq()` and so on, and neither knows nor cares
which radio is behind them. There are no virtual calls — `SDR_Device` owns an `RTLSDR` and a
`LimeSDR` and forwards to whichever type was selected.

An RTL-SDR build is unaffected: without the SoapySDR headers `USE_LIMESDR` is empty, `sdr.h` uses
`RTLSDR` as an inert placeholder, and no SoapySDR library is linked.

## Build

```bash
make USE_LIMESDR=1 FLAGS="-Wall -O3 -ffast-math -DVERSION=0.2.6 -I$PREFIX/include" \
     LIBS="-L$PREFIX/lib -Wl,-rpath,$PREFIX/lib -lpthread -lm -ljpeg -lconfig -lrt" ogn-rf
```

`-DUSE_LIMESDR` and `-lSoapySDR` are kept in the Makefile's own `SDR_FLAGS` / `LIME_LIBS` variables
rather than in `FLAGS`/`LIBS`, because a `make FLAGS=...` on the command line **replaces** `FLAGS`
outright. Put the define in `FLAGS` and the build line above would quietly produce a binary that
cannot select a LimeSDR.

Requirements: `libsoapysdr-dev` and `soapysdr0.7-module-lms7` (or any SoapySDR with an LMS7 module).
The LimeSuite command-line tools are *not* needed.

## Verified working configuration

```c
RF = {
  DeviceType   = "limesdr";
  DeviceSerial = "00090726074D1731";
  FreqPlan     = 1;              // Europe/Africa -> 868.200/868.400 MHz, centred on 868.300
  FreqRaster   = 1;              // no raster rounding; the LMS7002M synthesiser does not need it
  BiasTee      = 0;
  OfsTune      = 0;
  FreqCorr     = 0;
  SampleRate   = 1000000;
  Oversample   = 32;             // on-chip capture/decimate ratio; 0 = driver decides
  OGN = { Gain = 61.0; GainMode = 1; SampleRate = 1000000;
          SensTime = 0.85; StartTime = 0.375; };
};
Position = { Latitude = 51.5; Longitude = -0.1; };
APRS = { Call = "TEST0"; };
HTTP = { Port = 8095; };
```

`RF.OGN.Gain` is in **dB** (SoapySDR's own units), not an RTL-SDR tuner's discrete steps. The board
offers −12 … +61 dB, and **61 dB is the right value**, measured: at 50 dB the noise floor sits
0.7 dB above the receiver's own converter floor (it is listening to itself), at 61 dB ~9 dB above
it. That is too much gain for a busy band — at 800 MHz the ADC is already at 92 % of full scale —
but OGN's hopping plan only spans 868.2–868.4 MHz, which is quiet.

Startup:

```
HTTP_Server.Exec() ... Listening on port 8095
LimeSDR::SelectAntenna() => LNAL
LimeSDR::Open(0,868300000,1000000) => LimeSDR-USB,  868.300 MHz, 1.000 Msps, MTU=2040
LimeSDR::Gain[64] = -12.0 -10.8 ... +59.8 +61.0 [dB]
RF_Acq.Exec() ... Half time slot
```

`LimeSDR::Gain[64]` is synthesised from the device's own gain range, since an LMS7002M has no
discrete tuner gain table. `MTU=2040` is what SoapyLMS7 negotiates; a 0.85 s slot is about 417
`readStream()` calls.

## How the differences were handled

| RTL-SDR concept | On a LimeSDR |
|---|---|
| 8-bit unsigned I/Q | CS16 from SoapySDR, converted with `(int16 >> 8) + 128`, clamped |
| tuner gain steps | `setGain()` in dB; a gain table is synthesised for the status page |
| `setFreqCorrection(ppm)` | applied in software by retuning the LO to `freq × (1 + ppm·1e-6)` |
| `ResetBuffer()` | drain `readStream()` with a zero timeout (see below) |
| bias tee | accepted and ignored — a LimeSDR-USB has none |
| offset tuning | accepted and ignored — an RTL-SDR/E4000 concept |

`RF.Oversample` maps to SoapyLMS7's `OVERSAMPLING`: with a ratio of N the LMS7002M runs its
converter at N × the host rate and decimates on chip, so `ogn-rf` still receives 1 MSps. It
**defaults to the highest meaningful ratio, 32** — the converter tops out near 61.44 MSps and the
host rate is 1 MSps — and is set before `setSampleRate()`, which is what configures the chain. If a
device cannot deliver the host rate with that ratio, the open retries once with auto instead of
failing, because the host rate matters to the rest of ogn-rf and the ratio does not.

Two measured caveats, so nobody expects a win that is not there. First, oversampling buys nothing
for OGN on this link: a native 1 MSps capture and a 4 MSps capture normalise to the same in-band
noise density (−88.1 vs −88.9 dBFS/Hz), because at 868 MHz with an antenna the receiver is limited
by sky noise, not converter resolution. Second, on this LimeSuite build the setting appears to be a
**no-op entirely** — every ratio from 0 to 4096 is accepted and produces identical output, which a
4096× ratio at a 1 MSps host rate obviously must be. The status page reports `RF.Oversample  32
(auto) / applied 32`, and the open banner reports `oversampling 32`, so what happened is visible.

Two of those are load-bearing:

**`ResetBuffer()` must not re-calibrate.** The obvious implementation, deactivate/re-activate the
stream, re-runs the LMS7002M's automatic DC/IQ calibration — slow enough to miss the start of the
next slot — and ogn-rf calls `ResetBuffer()` before *every* slot. Draining pending samples with a
zero timeout does what ogn-rf actually wants and costs nothing.

**The `>> 8` shift is measured, not guessed.** With the LMS7002M's `TSG_NCO` digital test tone
enabled, the CS16 words reach ±25472, so the 16-bit words carry the signal at 16-bit scale and
`>> 8` lands exactly on the 8-bit range. `>> 4` clips, `>> 12` discards the signal.

## Why CS16 and SoapySDR rather than liblimesuite directly

`liblimesuite-dev` was not installed (only the shared library), while `libsoapysdr-dev` and the
SoapyLMS7 module were. SoapySDR is also the interface the surrounding tooling already speaks, and
it leaves a second radio (Airspy, bladeRF, …) behind the same `sdr.h` switch for free.

## What is not proven

- **Over-the-air OGN decoding.** Not demonstrated, because there is nothing to demodulate: an
  868 MHz omni antenna is connected to `RX1_L` and the front end is provably live — the noise floor
  is 17–20 dB higher at 780/800 MHz (DVB-T2, LTE800) than at 868 MHz, which an unconnected port
  cannot do — but both OGN channels measure at the noise floor (+0.4 dB, +0.3 dB). No transmitter is
  in range at this location yet. `scripts/verify-reception.sh` in the companion repository says
  within a minute whether signal has appeared.
- **GSM frequency calibration.** The retune calls are implemented, but with no RF there has never
  been a burst to lock to.
- **RX1 channel.** Only `RX0` is used. Using both channels coherently is not implemented.
- **Gateware match.** The board runs gateware 2.23 while the installed LimeSuite 20.01 expects 2.22
  and warns at open. Everything above was measured with that mismatch present.

`Half time slot` / `Dropped a slot` messages while nothing is attached to the output FIFO are
expected and are not a LimeSDR problem — the same pair appears on the RTL-SDR/KrakenSDR build. Drain
`ogn-rf.fifo` and the live time recovers immediately.

## Two runtime traps inherited from upstream

- **stdin**: `main()` exits when `fgets(stdin)` hits EOF, so run it as `sleep infinity | ./ogn-rf …`.
- **stdout buffering**: `printf` is block-buffered into a pipe, so use `stdbuf -oL -eL` or a `timeout`
  kill will throw the log away.

Both are handled by `scripts/run-ogn-rf.sh` in the companion repository.
