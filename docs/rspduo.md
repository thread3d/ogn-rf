# The RSPduo backend

This branch adds an SDRplay RSPduo as a third selectable radio. It works the same way the LimeSDR
backend does: `RF.DeviceType` in the configuration picks the radio, and everything downstream — the
pulse filter, the FFT, the OGN demodulator, the status page — is unchanged.

```ini
RF.DeviceType = "rspduo"
RF.Device     = 0
RF.SampleRate = 2000000
RF.Freq       = 868300000
```

## How it fits in

`rspduo.h` mirrors the public interface of `RTLSDR` in `rtlsdr.h`, exactly as `limesdr.h` does, and
`sdr.h` switches between the three. This branch is built on the multi-radio layer that the `limesdr`
branch introduced; the `rspduo` backend is an addition to that layer, not a parallel one.

| File | Role |
|---|---|
| `rspduo.h` | the RSPduo backend (`class RSPDuo`) |
| `sdr.h` | adds `RSP_DUO` and dispatches to it |
| `Makefile` | `USE_RSPDUO=1` → `-DUSE_RSPDUO -lsdrplay_api` |
| `ogn-rf.cc` | parses `RF.DeviceType = "rspduo"` |

Build support is auto-detected from `/usr/local/include/sdrplay_api.h`, so an ordinary RTL-SDR
build is unaffected. Force it either way with `make USE_RSPDUO=1` or `make USE_RSPDUO=0`.

## The three things that make this backend different

### 1. The API is callback-driven, ogn-rf is synchronous

`sdrplay_api_Init()` starts the stream and hands sample buffers to `StreamACbFn` on the API's own
thread; it returns immediately. ogn-rf, by contrast, calls `SDR.Read()` in a loop and expects it to
block until the samples are there.

So the callback pushes into a ring buffer and `Read()` drains it:

```
sdrplay_apiService (USB)  →  API thread  →  StreamACbFn  →  ring buffer (2 MiB)
                                                                  ↓
                                            ogn-rf acquisition loop: Read()
```

`Read()` waits on a condition variable and is woken by the callback. That bridge is the heart of
`rspduo.h`; the rest is configuration.

If the reader ever falls behind, the callback drops the whole backlog rather than delivering a torn
buffer, counts the event and resynchronises. A stream `reset` (which the API raises after a tuner or
rate change) also flushes the ring, because the queued samples belong to the old configuration and
splicing them onto the new one would fake a continuous stream across two different frequencies.

### 2. Two tuners, two streams, two separate I/Q arrays

The API delivers I and Q as two separate `short` arrays per stream (`xi[]`, `xq[]`), not
interleaved, and an RSPduo has an A and a B tuner.

**Both tuners can run at once.** `RF.RSPduo.DualTuner = 1` selects `Dual_Tuner` mode, and the
status page then shows a second row, `Tuner B (dual mode)`, carrying tuner B's own frequency.
`Single_Tuner` (the default) uses tuner A alone.

Why single-tuner is the default: ogn-rf needs one window of spectrum around 868.300 MHz, and one
tuner's 2 MHz of bandwidth already spans both European OGN channels (868.200 and 868.400). The
second tuner buys nothing for that and doubles the sample traffic over USB 2.0. Dual mode exists
because it is the RSPduo's distinguishing feature and the reason its reference tools
(DuoTools, SDRduoDisplay, DuoDX-recorder, dual-tuner-recorder) are written the way they are.

Dual mode is much more constrained than it looks:

| | Single_Tuner | Dual_Tuner |
|---|---|---|
| IF | `IF_Zero` | **`IF_Zero` is not valid** — 450 kHz at a 2 MHz ADC |
| Bandwidth | 1.536 MHz | 0.200 / 0.300 / 0.600 MHz |
| Output rate | 2 Msps | 500 ksps (bw 0.2/0.3) or 1 Msps (bw 0.6) |

Only a fixed set of (ADC rate, bandwidth, IF, internal decimation) combinations is accepted;
anything else makes `sdrplay_api_Init()` fail. Those are the combinations the backend allows.

And the trap the reference implementations warn about, which is real: **`sdrplay_api_Init()`
resets channel B's settings to channel A's.** Tuner B's own frequency must be written *after*
`Init()` returns and pushed down with a `sdrplay_api_Update()` on `sdrplay_api_Tuner_B`. Doing it
before `Init()` silently leaves both tuners on the same band — it looks like it works while
receiving the same spectrum twice. `ReconfigureTunerB()` does this, and it is verified on hardware
by tuning the two tuners to bands far apart and confirming their spectra differ (+16.9 dB of FM
content on one, +1.3 dB of nothing on the other).

### 3. The device is a daemon client

`libsdrplay_api` is a client library. All hardware access lives in `sdrplay_apiService`, which runs
as root under systemd; nothing in this backend touches USB. When `Open()` reports
`ServiceNotResponding`, the daemon is not running:

```bash
sudo systemctl status sdrplay
sudo systemctl start sdrplay
```

This is also why the RSPduo needs a system-wide install and cannot be driven from a user prefix the
way the LimeSDR work built its own dependencies.

## Sample format and the conversion scale

`RTLSDR` hands over interleaved 8-bit **unsigned** I/Q with 128 as zero. The RSPduo delivers 12-bit
samples in `int16` arrays. The conversion is the same one the LimeSDR backend uses:

```c
Val = (xi[i] >> Shift) + 128;   // Shift = 8, clamped to 0..255
```

`Shift` is a member rather than a constant so the scale can be measured on new hardware and
corrected without touching the conversion code.

**`Shift = 8` is measured, not assumed.** Two independent checks on an RSPduo agreed:

- At maximum gain with an antenna connected, the stream peaked at **21761 (−3.0 dBFS)** with **zero
  clipping** out of 4 M values — so the sample scale really does reach the 16-bit rails.
- The quantisation check reports that values are **not** all multiples of 2, 4 or 16, so no low
  bits are unused and no narrower ADC is being left-justified into 16 bits.

A weak-signal capture can look misleading here: at 40 dB of gain the same band peaked at only 1923,
which looks like a scale error but is simply a signal 24 dB below full scale. The quantisation check
is the one that settles it; a peak alone does not.

## Decimation

The API exposes a decimation factor, and it matters more than it sounds: **the ADC cannot run below
2 MHz at all**. Ask for a lower `fsHz` and `sdrplay_api_Init()` fails outright — verified on
hardware at 1 MHz and 500 kHz. So decimation is not an optimisation here, it is the *only* way to
reach any rate below 2 MHz.

**The highest decimation is 32, and only powers of two do anything.** That second half is the trap:
the API accepts a factor of 3, 6, 7 or 12 without complaint and then delivers the full 2 MHz
anyway — no error, no warning, no decimation. Measured by comparing the delivered rate against the
ADC rate:

| Factor | Expected | Delivered | Verdict |
|---|---|---|---|
| 2 | 1.0000 | 1.026 | applied |
| 4 | 0.5000 | 0.508 | applied |
| **6** | 0.3333 | **2.077** | **silently ignored** |
| **7** | 0.2857 | **3.311** | **silently ignored** |
| 8 | 0.2500 | 0.258 | applied |
| 16 | 0.1250 | 0.129 | applied |
| 32 | 0.0625 | 0.063 | applied |
| 33 | — | — | `Init()` **fails** |

So the validity check is on range, not on form: anything up to 32 is accepted, only powers of two
take effect, and 33 and above is rejected outright. `SnapSampleRate()` only ever produces powers of
two, so the backend cannot fall into the silent case — but anything driving the API directly can.

**The decimation is not done on the RSPduo silicon.** That was an assumption in earlier revisions of
this document and it is wrong; two independent measurements contradict it:

- The `sdrplay_apiService` daemon burns the *same* CPU at factor 1 and factor 32 — 0.61 s against
  0.56 s over ten seconds, for 2.004 Msps against 0.063 Msps. If the chip decimated, the daemon
  would be handling 32x fewer samples and its work would collapse. It doesn't move.
- At an 8 MHz ADC with a factor of 32, the delivered rate is 168 ksps, not the 250 ksps the
  arithmetic gives. The shortfall matches the bus-limited 5.48 Msps divided by 32 — so the stream is
  limited at full rate *before* the factor is applied.

The practical consequence matters on a shared bus: **decimation does not save USB bandwidth.** It
reduces only the rate handed to the client, while the full-rate stream still crosses USB. It cannot
be used to relieve bus contention, and it cannot recover the shortfall of an under-running ADC rate.
A USB-level capture (`usbmon`, which needs root) would confirm this directly.

`SnapSampleRate()` returns the **ADC** rate to program, and the delivered rate is that divided by
the factor — once, by the caller. Getting this wrong is easy and expensive: returning the
already-decimated rate programs a sub-2 MHz ADC rate (which fails to start) and then divides a
second time, halving the reported rate. A unit test pins the whole mapping down.

Dual-tuner mode uses the same mechanism but the API chooses the factor itself, as part of the fixed
combinations described above.

## Sample rate

In single-tuner mode the RSPduo's ADC produces **2, 6 or 8 MHz** and nothing else. A requested rate
is snapped to the nearest of those and the rate actually applied is reported at `Open()`, because a
silent 2 % rate error would slide every OGN channel off its FFT bin and quietly break decoding.

ogn-rf's own default is 2048000 Hz (an RTL-SDR rate). For an RSPduo, configure
`RF.SampleRate = 2000000`: that is the one rate at which both OGN channels fall where the frequency
plan expects them. Rates below 2 MHz are reached with the API's decimation factor, which
`SnapSampleRate()` sets up automatically.

## Gain

The RSPduo has no discrete tuner gain table. The API expresses gain as a pair: an IF gain reduction
(`gRdB`, 20..59 dB) and an LNA state step.

Measured on this unit at 868.3 MHz, with the LNA at state 0, the law is simply:

```
total gain [dB] = 115.8 - gRdB        95.8 dB at gRdB 20, 56.8 dB at gRdB 59
```

`setTunerGain()` therefore treats ogn-rf's single gain number as a **total gain** and spends it on
`gRdB`, clamping to the reachable 56.8–95.8 dB range. `115.8` is not hard-coded: the API reports
the total gain it actually applied on every gain change, and the backend recalibrates its offset
from that report, so the model corrects itself across units and bands.

LNA state is held at 0 deliberately. It is the lowest-noise setting, and on a weak, noise-dominated
band — which 868 MHz is here — noise figure limits reception, not gain. Measured at `gRdB` 20,
stepping the LNA state down did not lower the noise floor at all; it only moved the reported gain.

Because this is measured rather than assumed, the numbers are re-checkable with
`rspduo_probe`; see the RSPduo repository's `docs/hardware.md` for the sweep.

## Frequency correction

Unlike the LimeSDR backend — where ppm correction is emulated by shifting the LO in software — the
RSPduo has a real ppm control, so `setFreqCorrection()` writes it straight into `devParams->ppm`.
`getCenterFreq()` therefore reports the frequency actually tuned.

## Options that do nothing here

| Option | Behaviour |
|---|---|
| `RF.BiasTee` | accepted and ignored — the RSPduo has no bias tee (the RSP2 and RSPdx do) |
| `RF.OffsetTuning` | accepted and ignored — an RTL-SDR/E4000 technique with no meaning here |
| `RF.Oversample` | ignored — the RSPduo decimates internally, driven by the sample rate |
| `RF.ClockSource`, `RF.ReferenceRate` | ignored — RSPduo-only external reference is via the API, not wired up |

## DC offset and IQ imbalance correction

Left at the API default (enabled), but made explicit in `Open()` so the behaviour is visible. The
correction notches a few kHz at the exact centre frequency — 868.300 MHz — which sits **between**
the two OGN channels, so it does no harm to either. `DcOffsetCorrect` is a member if that ever needs
turning off for spectrum work.

## Verified on

See the RSPduo repository (`thread3d/RSPduo`) for the bring-up record, the measurements behind
`Shift = 8`, and the scripts that re-check reception.
