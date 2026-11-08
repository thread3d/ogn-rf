# KrakenSDR support — branch `krakensdr`

This branch carries the work to make `ogn-rf` run on a **KrakenSDR** (a five-channel coherent
RTL-SDR array) instead of a single RTL-SDR dongle. Design rationale, measurements and the hardware
bring-up log live in the companion repository
[`thread3d/KrakenSDR`](https://github.com/thread3d/KrakenSDR); this file is the part that belongs
next to the code.

**Status: design only.** No source files have been modified yet. The branch exists so that
modifications have somewhere to land and so the design can be reviewed before code is written.

## Why this is not a one-line change

`ogn-rf` talks to the radio through one class, `RTLSDR` in `rtlsdr.h`, which is a thin wrapper over
`librtlsdr`. That works for one dongle. A KrakenSDR is not one dongle:

- Its five RTL2832U receive chains share a single clock so their samples stay phase-coherent. They
  must be started, tuned and read in lockstep, which `librtlsdr`'s per-dongle async API does not do.
- The vendor therefore puts the USB devices under one acquisition daemon, **HeIMDALL**
  ([`krakenrf/heimdall_daq_fw`](https://github.com/krakenrf/heimdall_daq_fw)), which publishes IQ
  frames and leaves application software as a *consumer*.
- The packaged `librtlsdr` is also too old for HeIMDALL: it lacks `rtlsdr_set_dithering` and
  `rtlsdr_set_sample_freq_correction_f`, both of which `rtl_daq.c` calls. HeIMDALL needs the
  [krakenrf fork](https://github.com/krakenrf/librtlsdr).

## The seam

`RF_Acq::Exec()` in `ogn-rf.cc` uses only this method surface on its `SDR` object:

```
isOpen()               getTime()                ResetBuffer()
Read(SampleBuffer<uint8_t>&, int)               Open(index, freq, rate)
Close()                setCenterFreq(Hz)        getFreqCorrection()
setTunerGainMode(m)    setTunerGain(gain)       setFreqCorrection(ppm)
setOffsetTuning(on)    setBiasTee(on)           getDeviceIndexBySerial(serial)
```

Nothing in that list is intrinsically RTL-SDR-specific. So the plan is an alternative source class
with the same method names and signatures, chosen by a config key, leaving `ogn-rf.cc` calling
`SDR.*` unchanged:

```
rtlsdr.h                  unchanged — reference implementation of the interface
krakensdr_source.h        NEW  HeIMDALL frame source, same interface
krakensdr_source.cc       NEW  shm attach, double-buffer handshake, de-interleave
ogn-rf.cc                 small diff: pick the source class from config
```

Proposed configuration key:

```
RF.Source = "rtlsdr"        # default, current behaviour
RF.Source = "heimdall"      # read IQ frames from HeIMDALL
```

## The frequency plan decides the architecture

`freqplan.h` derives four `(slot, OGN)` frequencies per second. `calcCenterFreq()` then tries to
find a single tuner centre covering all four:

| Plan | Region | Span of the four frequencies |
|---|---|---|
| 1 | Europe / Africa (default) | **0.2 MHz** — both channels fit at 868.3 MHz |
| 4 | New Zealand | 0 — single channel at 869.25 MHz |
| 2 | USA / Canada | **26.0 MHz** — hash-hopped across 65 channels |
| 3 | Australia / S. America | **9.6 MHz** — hash-hopped across 24 channels |

**In Europe and New Zealand `ogn-rf` never retunes.** That is the easy and most common case: the
coherent array can sit on 868.3 MHz indefinitely, and KrakenSDR's ~1–3.5 s recalibration-on-retune
never comes into play.

**In the USA and Australia it really does retune every 850 ms slot.** Coherent mode cannot follow
that: each retune costs more than a slot. But this is where the array becomes an *advantage* rather
than a problem — in KrakenSDR's **Independent mode** each of the five tuners has its own frequency,
so the four current slot channels can be received **simultaneously** instead of in turn. That
removes `ogn-rf`'s dead time instead of adding to it.

## Recommended phasing

**Phase 1 — OGN reception, single channel, no DoA.**
Run one tuner at a fixed frequency (Europe: 868.3 MHz), sourced either directly from the krakenrf
`librtlsdr` fork or from HeIMDALL. This validates antenna → tuner → slot alignment → decoder handoff
against a known-good receiver, and needs the fewest moving parts.

**Phase 2 — exploit the array.**
Add the HeIMDALL source and use **Independent** mode to receive several OGN slot channels at once,
or **Coherent** mode on a fixed channel where direction-finding a static OGN/FLARM transmitter is
actually meaningful.

## Things to resolve before writing the adapter

1. **Sample rate.** `ogn-rf` defaults to 1 000 000 sps and derives FFT bin counts from it. Set
   HeIMDALL's decimator (`daq_chain_config.ini`) to 1 MS/s rather than resampling in `ogn-rf`.
2. **Sample format.** `ogn-rf` works in 8-bit unsigned I/Q. Check the decimator's
   `data_type` / `sample_bit_depth` in the frame header before writing the adapter; an int16 or
   float32 path needs conversion.
3. **Slot alignment.** `RF_Acq::Exec()` aligns on 0.375 s using `SDR.getTime()`, i.e. the host
   clock. HeIMDALL frames carry their own `time_stamp`. Align on the frame timestamp instead, or the
   two clocks will drift within a slot.
4. **Calibration vs the slot.** In coherent mode the noise-source calibration blanks reception for
   roughly a second — longer than an 850 ms OGN slot. Either calibrate between acquisition passes or
   use Independent mode, which does not calibrate per retune.
5. **The GSM frequency reference.** `ogn-rf` can lock its frequency against a nearby GSM base station
   (`GSM_CenterFreq`), which costs a tuner and a retune away from 868 MHz. On a five-channel radio
   that is affordable — dedicate one tuner — but it has to be coordinated with the mode choice.

## Hardware notes from the bring-up

On the test unit (Jetson Xavier AGX, Ubuntu 20.04, `aarch64`):

- All five chains enumerate as `0bda:2838` behind an internal Microchip USB2517 hub, with EEPROM
  serials 1000–1004 and R820T tuners.
- **Enumeration order is not channel order:** SoapySDR lists them `1004, 1000, 1001, 1002, 1003`.
  Address tuners by serial wherever possible.
- There is **no CP210x serial bridge** and no `/dev/ttyUSB*`; this board switches its noise source
  and antenna matrix over the RTL2832U GPIOs (`rtlsdr_set_bias_tee_gpio`). A missing `ttyUSB` is
  expected, not a fault.
- The RF path is live: output power scales with gain and the five channels differ from each other as
  independent receivers should.

## Licence

`ogn-rf` is **GPL v3**. HeIMDALL is **GPL v3**. Any code shared between the trees stays GPL v3.
