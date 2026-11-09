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
interleaved, and an RSPduo has an A and a B tuner. This backend runs **tuner A only**, in
`Single_Tuner` mode.

That is a deliberate choice. ogn-rf needs one window of spectrum around 868.300 MHz, and the
RSPduo's 2 MHz of bandwidth already spans both European OGN channels (868.200 and 868.400) with
room to spare — so the second tuner would buy nothing for OGN and would double the sample traffic
over a USB 2.0 link. `sdrplay_api_RspDuoMode_Dual_Tuner` is wired through `setRspDuoMode()` for
anyone who wants it, but it takes effect at the next `Open()` and is not exercised by ogn-rf.

Worth knowing if dual-tuner mode is ever enabled: **`sdrplay_api_Init()` resets channel B's
settings to channel A's.** Channel B must be reconfigured with `sdrplay_api_Update()` *after*
`Init()` returns, not before. This is documented in the reference implementations
(`dual-tuner-recorder` notes it explicitly) and is easy to miss.

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

`Shift` is a member rather than a constant so the scale can be measured against a known signal and
corrected without touching the conversion code. `Shift = 8` is the value to verify first on new
hardware: if every level reads 12 dB low, the samples are left-justified differently than assumed
and the shift belongs at 6.

## Sample rate

In single-tuner mode the RSPduo's ADC produces **2, 6 or 8 MHz** and nothing else. A requested rate
is snapped to the nearest of those and the rate actually applied is reported at `Open()`, because a
silent 2 % rate error would slide every OGN channel off its FFT bin and quietly break decoding.

ogn-rf's own default is 2048000 Hz (an RTL-SDR rate). For an RSPduo, configure
`RF.SampleRate = 2000000`: that is the one rate at which both OGN channels fall where the frequency
plan expects them. Rates below 2 MHz are reached with the API's on-chip decimation, which
`SnapSampleRate()` sets up automatically.

## Gain

The RSPduo has no discrete tuner gain table. The API expresses gain as a pair: an IF gain reduction
(`gRdB`, 20..59 dB) and an LNA state step. `setTunerGain()` therefore picks the LNA state that lands
closest to the gain ogn-rf asks for, holding `gRdB` at 40 dB, and publishes a table built from the
range the device itself reports. `getTunerGain()` reports what the API says is actually in use.

Because the LNA step size is band dependent, the table is rebuilt on every `Open()` and after every
gain change.

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
