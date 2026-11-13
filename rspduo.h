/*
    OGN - Open Glider Network - http://glidernet.org/
    Copyright (c) 2015 The OGN Project

    A detailed list of copyright holders can be found in the file "AUTHORS".

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this software.  If not, see <http://www.gnu.org/licenses/>.
*/

// ==================================================================================================
//
// SDRplay RSPduo backend, through the SDRplay API 3.x (libsdrplay_api).
//
// This class mirrors the public interface of RTLSDR (rtlsdr.h) so that the acquisition code in
// ogn-rf.cc can drive an RSPduo without knowing which radio it has. The contract that matters
// downstream is the sample format: RTLSDR hands over interleaved 8-bit UNSIGNED I/Q with 128 as
// zero, and this class produces exactly that, so the pulse filter, the FFT and the OGN demodulator
// are untouched.
//
// Three things differ from every other backend here, and they drive the design:
//
//   1. THE API IS CALLBACK-DRIVEN, OGN-RF IS SYNCHRONOUS.
//      sdrplay_api_Init() starts a stream and hands samples to StreamACbFn on the API's own
//      thread; it returns immediately. ogn-rf, by contrast, calls SDR.Read() in a loop and
//      expects it to block until samples are there. So the callback pushes into a ring buffer
//      and Read() drains it. That bridge is the heart of this file.
//
//   2. THE TWO TUNERS ARRIVE AS TWO SEPARATE STREAMS.
//      The API delivers I and Q as two separate short arrays per stream (xi[]/xq[]), not
//      interleaved, and the RSPduo has an A and a B tuner. This backend runs tuner A only:
//      ogn-rf needs a single window around 868.300 MHz and the RSPduo's 2 MHz of bandwidth
//      already spans both European OGN channels. Dual_Tuner mode is available in the API
//      (see RspDuoMode below) but is deliberately not enabled - see docs/rspduo.md.
//
//   3. THE DEVICE IS A CLOSED-SOURCE DAEMON CLIENT.
//      libsdrplay_api is a client library: all hardware access lives in sdrplay_apiService,
//      which must be running as root. Nothing here talks to USB. If Open() reports
//      "ServiceNotResponding", the daemon is not up - see scripts/install-sdrplay-api.sh.
//
// Notes on the mapping, because several RTL-SDR concepts have no RSPduo equivalent:
//
//   * Frequency correction (ppm) - the RSPduo does have a real ppm control, so unlike the
//     LimeSDR backend this is NOT emulated in software: it goes straight into devParams->ppm.
//     getCenterFreq() therefore reports the true tuned frequency.
//   * Sample rate - the RSPduo's ADC runs at 2, 6 or 8 MHz in single-tuner mode. A requested
//     rate is snapped to the nearest supported value and the rate actually applied is reported,
//     because a silent 2 % rate error would slide every OGN channel off its bin.
//   * Bias tee - the RSPduo DOES have one, and this backend used to ignore it on the mistaken
//     belief that it did not. It is on the TUNER 2 PORT ONLY, 4.7 V at 100 mA, so it can power an
//     antenna-mounted LNA on that port and nothing on the other. Enabled through
//     rspDuoTunerParams.biasTEnable with sdrplay_api_Update_RspDuo_BiasTControl.
//     That reason sits in the API's per-tuner section, not the master-only one, so it must be
//     applied to TUNER B - the port it physically feeds. Applying it to tuner A silently does
//     nothing, which is what this backend did at first.
//   * Offset tuning - an RTL-SDR/E4000 trick with no meaning here; accepted and ignored.
//   * getTunerGains() - the RSPduo has no discrete tuner gain table. The API exposes gain as an
//     IF gain reduction (gRdB) plus an LNA state, so a table is synthesised from the device's
//     own reported range and the LNA state count, so the status page and config validation work.
//
// ==================================================================================================

#ifndef __RSPDUO_H__
#define __RSPDUO_H__

#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <time.h>
#include <sys/time.h>

#include <math.h>
#include <pthread.h>

#include <sdrplay_api.h>

#include "thread.h"

// ==================================================================================================
//
// One tuner's sample ring: the bridge from the API's callback thread to Read().
//
// Factored out of RSPDuo so dual-tuner mode uses the same logic twice instead of a second copy.
// Each ring is independent - its own lock, its own condition variable, its own overrun count - so
// a slow reader on one tuner cannot stall the other.
//
// ==================================================================================================

class RSPDuo_Ring
{ public:
   uint8_t        *Buffer;         // interleaved 8-bit unsigned I/Q
   size_t          Size;           // capacity in bytes
   size_t          Head;           // read position
   size_t          Full;           // bytes currently stored
   pthread_mutex_t Lock;
   pthread_cond_t  Cond;           // signalled by the callback, waited on by Pop()
   int             Stopped;        // set by Stop() to unblock a waiting Pop()

   uint64_t        Overflows;      // buffers dropped because the ring was full
   uint64_t        Buffers;        // stream buffers received
   uint64_t        BytesRead;      // bytes delivered (1 sample = 2 bytes: I/Q)

   RSPDuo_Ring()
   { Buffer=0; Size=0; Head=0; Full=0; Stopped=0; Overflows=0; Buffers=0; BytesRead=0;
     pthread_mutex_init(&Lock, 0); pthread_cond_init(&Cond, 0); }

  ~RSPDuo_Ring()
   { Free(); pthread_mutex_destroy(&Lock); pthread_cond_destroy(&Cond); }

   int Allocate(size_t Bytes)
   { Free();
     Buffer=(uint8_t *)malloc(Bytes);
     if(Buffer==0) return -1;
     Size=Bytes; Head=0; Full=0; Stopped=0; Overflows=0; Buffers=0; BytesRead=0;
     return 0; }

   void Free(void)
   { if(Buffer) { free(Buffer); Buffer=0; } Size=0; Head=0; Full=0; }

   bool isOpen(void) const { return Buffer!=0; }

   void Flush(void)
   { pthread_mutex_lock(&Lock); Head=0; Full=0; pthread_mutex_unlock(&Lock); }

   void Stop(void)
   { pthread_mutex_lock(&Lock); Stopped=1; pthread_cond_broadcast(&Cond); pthread_mutex_unlock(&Lock); }

   // Convert one API buffer (separate int16 I and Q arrays) into interleaved u8, and append it.
   void Push(const short *xi, const short *xq, unsigned int N, int Shift, int Reset)
   { if(N==0 || Buffer==0) return;
     size_t Bytes=2*(size_t)N;

     pthread_mutex_lock(&Lock);
     if(Stopped) { pthread_mutex_unlock(&Lock); return; }

     // Reset!=0 means the API restarted the stream (tuner or rate change). The queued samples
     // belong to the old configuration, so discard them: keeping them would splice two different
     // centre frequencies into one continuous stream.
     if(Reset) { Head=0; Full=0; }

     // Not enough room: drop the whole backlog rather than deliver a torn buffer. A real overrun,
     // so it is counted and reported.
     if(Full+Bytes>Size) { Head=0; Full=0; Overflows++; }

     size_t Tail=(Head+Full)%Size;
     size_t First=Bytes;
     if(First>Size-Tail) First=Size-Tail;
     size_t Second=Bytes-First;

     for(size_t Idx=0; Idx<First/2; Idx++)
     { int vi=(xi[Idx]>>Shift)+128; if(vi<0) vi=0; else if(vi>255) vi=255;
       int vq=(xq[Idx]>>Shift)+128; if(vq<0) vq=0; else if(vq>255) vq=255;
       Buffer[Tail+2*Idx]  =(uint8_t)vi;
       Buffer[Tail+2*Idx+1]=(uint8_t)vq; }
     for(size_t Idx=0; Idx<Second/2; Idx++)
     { size_t Src=First/2+Idx;
       int vi=(xi[Src]>>Shift)+128; if(vi<0) vi=0; else if(vi>255) vi=255;
       int vq=(xq[Src]>>Shift)+128; if(vq<0) vq=0; else if(vq>255) vq=255;
       Buffer[2*Idx]  =(uint8_t)vi;
       Buffer[2*Idx+1]=(uint8_t)vq; }

     Full+=Bytes;
     Buffers++;
     BytesRead+=Bytes;
     pthread_cond_signal(&Cond);
     pthread_mutex_unlock(&Lock); }

   // Block until Samples interleaved u8 samples are available, or the ring is stopped.
   int Pop(uint8_t *Dst, int Samples)
   { Samples&=0xFFFFFF00;                    // multiple of 256, as in rtlsdr.h
     if(Samples<=0) return 0;
     size_t Want=2*(size_t)Samples;
     size_t Done=0;

     pthread_mutex_lock(&Lock);
     while(Done<Want)
     { if(Stopped) { pthread_mutex_unlock(&Lock); return -1; }
       if(Full==0) { pthread_cond_wait(&Cond, &Lock); continue; }
       size_t Take=Want-Done; if(Take>Full) Take=Full;
       size_t First=Take;
       if(First>Size-Head) First=Size-Head;
       memcpy(Dst+Done, Buffer+Head, First);
       if(Take>First) memcpy(Dst+Done+First, Buffer, Take-First);
       Head=(Head+Take)%Size;
       Full-=Take;
       Done+=Take; }
     pthread_mutex_unlock(&Lock);
     return (int)(Done/2); }

} ;

// ==================================================================================================

// ==================================================================================================

class RSPDuo
{ public:
   MutEx         Lock;              // for multi-threading, as in RTLSDR

   uint32_t      DeviceIndex;       // index of the device among the enumerated RSP devices

   sdrplay_api_DeviceT          Device;   // the API's device descriptor: SerNo, hwVer, dev handle
   sdrplay_api_DeviceParamsT   *Params;   // only valid while the stream is initialised
   int                          ApiOpen;  // sdrplay_api_Open() succeeded
   int                          Streaming;// sdrplay_api_Init() succeeded

   int           Gains;             // number of possible gain settings
   int           Gain[64];          // [0.1 dB] list of possible gain settings

   uint32_t      FreqRaster;        // [Hz] use only multiples of this base value to avoid tuning errors

   // --- sample rings: the bridge from the API's callback thread to our blocking Read() --------
   // One per tuner. In single-tuner mode only [0] exists; in dual-tuner mode both do, and they
   // are independent so neither can stall the other.
   RSPDuo_Ring   Ring[2];

   int           TunerBEnabled;     // 1 when running in Dual_Tuner mode
   double        FreqBApplied;      // [Hz] tuner B centre frequency
   uint64_t      BOverflows;        // tuner B ring overruns, for the status page

   int         (*Callback)(uint8_t *Buffer, int Samples, double SampleTime, double SamplePeriod, void *Contex);
   void         *CallbackContext;

   clockid_t     RefClock;          // CLOCK_REALTIME, CLOCK_MONOTONIC or CLOCK_MONOTONIC_RAW

   double        SampleTime;         // [sec] time when a batch of samples starts
   double        StartTime;          // [sec] time when acquisition started
   double        AverPeriod;         // [sec] averaging period for TimeRef and SamplePeriod
   double        SamplePeriod;       // [sec] time per sample
   double        PrevTime;           // [sec]
   double        SampleTime_DMS;     // [sec^2] mean square variation of SampleTime

   // --- configuration that the caller sets before Open() ---------------------------------------
   // Gain model, measured on this unit at 868.3 MHz: total gain = GainOffset - gRdB at LNA 0.
   // GainOffset is recalibrated from the API's own gain-change reports (see ClassEventCb).
   double        GainOffset;         // [dB] added to (0 - gRdB) to give the total gain
   int           GRdB_set;           // [dB] IF gain reduction currently programmed, 20..59
   int           LNAstate;           // LNA step; held at 0, the lowest-noise setting
   static const int GR_DB_MIN = 20;  // API "NORMAL" minimum gain reduction
   static const int GR_DB_MAX = 59;  // API maximum gain reduction (MAX_BB_GR)
   int           BiasTee;            // [bool] requested bias tee state (no such hardware)
   int           DcOffsetCorrect;    // 1 = let the API notch DC and correct IQ imbalance
   int           RspDuoMode;         // sdrplay_api_RspDuoModeT: Single_Tuner is the default
   int           DualMode;           // 1 when the open used Dual_Tuner (both tuners streaming)

   // --- selection diversity ------------------------------------------------------------------
   // Both tuners on the SAME frequency, decided per read by whichever has more signal energy.
   // This is the cheap end of diversity: it needs no phase calibration and no combining maths,
   // only that the two antennas fade independently. Maximal-ratio combining would need the
   // ~117 deg inter-tuner phase offset (measured, stable) tracked continuously.
   int           Diversity;          // 1 = selection diversity between the two tuners
   int           DivLastTuner;       // tuner chosen last time, for hysteresis
   double        DivSwitchdB;        // only switch when the other tuner is this much better
   uint8_t      *DivBuf[2];          // scratch: both streams drained before choosing
   size_t        DivBufBytes;
   uint64_t      DivChoseA;          // times tuner A won
   uint64_t      DivChoseB;          // times tuner B won
   uint64_t      DivSwitches;        // times the choice actually changed
   double        DivPowerA;          // [dB] last measured energy, tuner A
   double        DivPowerB;          // [dB] last measured energy, tuner B

   // --- what the device actually did, for the status page --------------------------------------
   double        SampleRateApplied;  // [Hz]
   double        FreqApplied;        // [Hz] nominal centre frequency
   double        FreqB;              // [Hz] tuner B centre frequency (dual mode only)
   int           FreqCorr;           // [ppm] pushed straight into devParams->ppm
   double        GaindBApplied;      // [dB] total gain the API reports as in use
   char          DeviceSerNo[SDRPLAY_MAX_SER_NO_LEN];
   unsigned char HardwareVer;        // SDRPLAY_RSPduo_ID for an RSPduo
   float         ApiVersion;         // as reported by sdrplay_api_ApiVersion()

  public:
   RSPDuo()
   { DeviceIndex=0; Params=0; ApiOpen=0; Streaming=0; DeviceSerNo[0]=0; HardwareVer=0; ApiVersion=0;
     memset(&Device, 0, sizeof(Device));
     Gains=0; FreqRaster=0;
     TunerBEnabled=0; FreqBApplied=0; BOverflows=0;
     Diversity=0; DivLastTuner=0; DivSwitchdB=1.0; DivBuf[0]=DivBuf[1]=0; DivBufBytes=0;
     DivChoseA=0; DivChoseB=0; DivSwitches=0; DivPowerA=0; DivPowerB=0;
     Callback=0; CallbackContext=0;
     #ifndef __MACH__
     RefClock=CLOCK_REALTIME;
     #endif
     AverPeriod=100.0;
     SamplePeriod=1.0/2000000.0; SampleTime=0; StartTime=0; PrevTime=0; SampleTime_DMS=0.0001*0.0001;
     // Start at a MID gain, not the maximum. ogn-rf calls Open() and only then setTunerGain(),
     // so the receiver streams briefly at whatever this default is. At GR_DB_MIN (maximum gain)
     // that window overloads the ADC whenever an LNA is in front - measured as a burst of
     // "ADC overload detected" at every open. Mid-range is a far safer starting point.
     GainOffset=115.8; GRdB_set=50; LNAstate=0; BiasTee=(-1); DcOffsetCorrect=1;
     RspDuoMode=sdrplay_api_RspDuoMode_Single_Tuner; DualMode=0;
     DualBw=sdrplay_api_BW_0_600; DualIf=sdrplay_api_IF_0_450; DualDecimation=2;
     FreqB=0;
     SampleRateApplied=0; FreqApplied=0; FreqCorr=0; GaindBApplied=0; }

  ~RSPDuo()
   { Close(); }

   bool isOpen(void) const { return Streaming!=0; }

   double getTime(void) const                                             // read the system time now
#ifndef __MACH__
   { struct timespec now; clock_gettime(RefClock, &now); return now.tv_sec + 1e-9*now.tv_nsec; }
#else
   { struct timeval now; gettimeofday(&now, 0); return now.tv_sec + 1e-6*now.tv_usec; }
#endif

   double SampleTimeJitter(void) { return sqrt(SampleTime_DMS); }

   // -----------------------------------------------------------------------------------------------
   // Enumeration
   //
   // Each call opens and closes the API service connection: enumeration is rare (Open(), the
   // status page, the -L listing) and holding the connection open would lock out the API service
   // between calls for no benefit.
   // -----------------------------------------------------------------------------------------------

   // Fill Devices[] with what the service can see. Returns the number found, or -1 on failure.
   static int enumerate(sdrplay_api_DeviceT *Devices, unsigned int MaxDevices, const char *&Error)
   { Error=0;
     sdrplay_api_ErrT Err=sdrplay_api_Open();
     if(Err!=sdrplay_api_Success)
     { Error=sdrplay_api_GetErrorString(Err); return -1; }
     Err=sdrplay_api_LockDeviceApi();
     if(Err!=sdrplay_api_Success)
     { Error=sdrplay_api_GetErrorString(Err); sdrplay_api_Close(); return -1; }
     unsigned int Count=0;
     Err=sdrplay_api_GetDevices(Devices, &Count, MaxDevices);
     sdrplay_api_UnlockDeviceApi();
     sdrplay_api_Close();
     if(Err!=sdrplay_api_Success) { Error=sdrplay_api_GetErrorString(Err); return -1; }
     return (int)Count; }

   // How many RSP devices (of any model) are attached. An RSPduo counts once.
   static int getNumberOfDevices(void)
   { sdrplay_api_DeviceT Devices[SDRPLAY_MAX_DEVICES]; const char *Error;
     return enumerate(Devices, SDRPLAY_MAX_DEVICES, Error); }

   static const char *modelName(unsigned char HwVer)
   { switch(HwVer)
     { case SDRPLAY_RSP1_ID:      return "RSP1";
       case SDRPLAY_RSP1A_ID:     return "RSP1A";
       case SDRPLAY_RSP2_ID:      return "RSP2";
       case SDRPLAY_RSPduo_ID:    return "RSPduo";
       case SDRPLAY_RSPdx_ID:     return "RSPdx";
       case SDRPLAY_RSP1B_ID:     return "RSP1B";
       case SDRPLAY_RSPdxR2_ID:   return "RSPdxR2";
       default:                   return "RSP-unknown"; } }

   const char *getDeviceName(void) const { return modelName(HardwareVer); }

   // Index of the device with the given serial, -1 when there is no such device
   int getDeviceIndexBySerial(const char *Serial)
   { if(Serial==0) return -1;
     sdrplay_api_DeviceT Devices[SDRPLAY_MAX_DEVICES]; const char *Error;
     int Count=enumerate(Devices, SDRPLAY_MAX_DEVICES, Error);
     if(Count<0) return -1;
     for(int Idx=0; Idx<Count; Idx++)
       if(strncmp(Devices[Idx].SerNo, Serial, SDRPLAY_MAX_SER_NO_LEN)==0) return Idx;
     return -1; }

   // -----------------------------------------------------------------------------------------------
   // The stream callback: convert, timestamp, publish
   //
   // Runs on the API's own thread. It must be quick and must not block, so it does the minimum:
   // convert to the 8-bit interleaved layout, push into the ring, and wake one reader.
   // -----------------------------------------------------------------------------------------------

   static void StaticStreamACb(short *xi, short *xq, sdrplay_api_StreamCbParamsT *params,
                               unsigned int numSamples, unsigned int reset, void *cbContext)
   { RSPDuo *This=(RSPDuo *)cbContext;
     if(This) This->ClassStreamCb(0, xi, xq, params, numSamples, reset); }

   static void StaticStreamBCb(short *xi, short *xq, sdrplay_api_StreamCbParamsT *params,
                               unsigned int numSamples, unsigned int reset, void *cbContext)
   { RSPDuo *This=(RSPDuo *)cbContext;
     if(This) This->ClassStreamCb(1, xi, xq, params, numSamples, reset); }

   static void StaticEventCb(sdrplay_api_EventT eventId, sdrplay_api_TunerSelectT tuner,
                             sdrplay_api_EventParamsT *params, void *cbContext)
   { RSPDuo *This=(RSPDuo *)cbContext;
     if(This) This->ClassEventCb(eventId, tuner, params); }

   // RSP samples arrive as int16 and are shifted onto the 8-bit range with the same full scale as
   // an RTL-SDR dongle (128 = zero), which is what everything downstream expects.
   //
   // Shift = 8 was measured on an RSPduo rather than assumed, in two independent ways. At maximum
   // gain with an antenna connected the stream peaked at 21761 (-3.0 dBFS) with zero clipping out
   // of 4 M values, i.e. the scale really does reach the 16-bit rails. And the quantisation check
   // in rspduo_probe reports that sample values are not all multiples of 2, 4 or 16, so the full
   // 16-bit range is in use and there are no unused low bits from a narrower ADC. It stays a
   // member so it can be re-measured on other units without touching this code.
   int           Shift;

   // One entry point for both tuners. The conversion and buffering live in RSPDuo_Ring; what is
   // left here is the timestamp bookkeeping, which is shared state and must therefore be updated
   // from one stream only.
   void ClassStreamCb(int Tuner, short *xi, short *xq, sdrplay_api_StreamCbParamsT *params,
                      unsigned int numSamples, unsigned int reset)
   { (void)params;                               // firstSampleNum/grChanged are not needed here
     if(numSamples==0 || Tuner<0 || Tuner>1) return;
     Ring[Tuner].Push(xi, xq, numSamples, Shift, reset);

     // Timestamps follow the first tuner only. Both tuners in dual mode run off the same clock at
     // the same rate, so feeding this from both would simply double the sample count and make the
     // averaged sample period nonsense.
     if(Tuner!=0) return;

     // The timing model copied from RTLSDR::ClassCallback, so that ogn-rf's sample timestamps
     // keep the same meaning they have with an RTL-SDR dongle.
     double ReadTime=getTime();
     double AcqTime=numSamples*SamplePeriod;
     double AverWeight=AcqTime/AverPeriod;
     SampleTime += numSamples*SamplePeriod;
     double TimeDiff=ReadTime-SampleTime;
     SampleTime += 0.125*TimeDiff;
     SampleTime_DMS += AverWeight*(TimeDiff*TimeDiff-SampleTime_DMS);
     double PeriodDiff=(ReadTime-PrevTime)-AcqTime;
     SamplePeriod += AverWeight*(PeriodDiff/numSamples);
     PrevTime=ReadTime; }

   void ClassEventCb(sdrplay_api_EventT eventId, sdrplay_api_TunerSelectT tuner,
                     sdrplay_api_EventParamsT *params)
   { (void)tuner;                                // gain reports are shared by both tuners
     switch(eventId)
     { case sdrplay_api_GainChange:
         if(params)
         { GaindBApplied=params->gainParams.currGain;
           // The API reports the total gain it actually applied, so the model's offset can be
           // corrected from the device itself instead of relying on a constant measured once.
           // total = offset - gRdB  =>  offset = total + gRdB.
           double Offset=GaindBApplied + (double)params->gainParams.gRdB;
           if(Offset>40.0 && Offset<200.0) GainOffset=Offset; }
         break;
       case sdrplay_api_PowerOverloadChange:
         // Report it, but do not act: a momentary overload is normal in a busy band and the
         // OGN demodulator tolerates it. Acting here would mean acknowledging the message,
         // which requires an Update() call from this thread - not safe.
         printf("RSPduo: ADC overload %s\n",
                (params && params->powerOverloadParams.powerOverloadChangeType==sdrplay_api_Overload_Detected)
                ? "detected" : "corrected");
         break;
       case sdrplay_api_DeviceRemoved:
         printf("RSPduo: device removed from the bus, stopping\n");
         CancelAsync();
         break;
       case sdrplay_api_RspDuoModeChange:
         printf("RSPduo: mode change event\n");
         break;
       case sdrplay_api_DeviceFailure:
         printf("RSPduo: device failure, stopping\n");
         CancelAsync();
         break;
       default: break; } }

   // -----------------------------------------------------------------------------------------------
   // Sample rate: snap to what the RSPduo can actually generate
   //
   // In single-tuner mode the RSPduo's ADC produces 2, 6 or 8 MHz. Anything else is snapped to
   // the nearest, and lower rates are obtained with the API's decimation factor.
   // -----------------------------------------------------------------------------------------------

   // Returns the ADC rate to program into fsFreq, and sets Decimation to the factor to request.
   // The delivered output rate is the return value DIVIDED by Decimation - that division belongs
   // to the caller, and only once.
   //
   // This distinction is load-bearing. The ADC cannot run below 2 MHz: asking for a lower fsFreq
   // makes sdrplay_api_Init() fail outright, which is verified on hardware. So decimation is not
   // an optimisation here, it is the only way to reach any rate below 2 MHz. Returning the already
   // decimated rate would program a sub-2 MHz ADC rate (failing to start) and then divide a second
   // time (halving the reported rate).
   static double SnapSampleRate(double Wanted, int &Decimation)
   { static const double Rate[3]={2000000.0, 6000000.0, 8000000.0};
     Decimation=1;
     if(Wanted<=Rate[0])
     { // Largest power-of-two factor that still leaves at least the rate asked for.
       while( (Decimation<32) && (Wanted<=Rate[0]/(2.0*Decimation)) ) Decimation*=2;
       return Rate[0]; }
     double Best=Rate[0]; double BestErr=fabs(Wanted-Rate[0]);
     for(int Idx=1; Idx<3; Idx++)
       if(fabs(Wanted-Rate[Idx])<BestErr) { Best=Rate[Idx]; BestErr=fabs(Wanted-Rate[Idx]); }
     return Best; }

   // The rate the host will actually be given for a requested rate, worked out from the settings
   // alone - no hardware, no API service, safe to call before Open().
   //
   // This has to be answerable before Open(), because ogn-rf sizes everything that depends on the
   // sample rate - how many samples make up one time slot, the demodulator's FFT size, the width
   // of the hopping band - from a single number read at startup. In single-tuner mode that number
   // is the ADC rate snapped to 2/6/8 MHz and divided by the decimation. In dual-tuner mode it is
   // not negotiable at all: the API fixes the delivered rate from its (ADC rate, bandwidth, IF)
   // combination, which for the default 2 MHz / 0.6 MHz / IF 450 kHz is 1 Msps per tuner - half
   // the rate RF.SampleRate asks the ADC for. Assuming the requested rate instead makes ogn-rf ask
   // for twice the samples the radio can deliver in a slot, so every read takes twice its slot and
   // the demodulator plans for a stream 2x faster than the one it is fed.
   uint32_t predictSampleRate(uint32_t Wanted)
   { // RspDuoMode, not DualMode: the latter is only derived from it inside Open(), and the whole
     // point of this function is to be answerable before Open() has ever run.
     if(RspDuoMode==sdrplay_api_RspDuoMode_Dual_Tuner)
     { double Rate=2000000.0; int Dec=1;
       DualCombo(Rate, Dec);
       return (uint32_t)lround(Rate/(double)(Dec>1 ? Dec : 1)); }
     int Dec=1;
     double Rate=SnapSampleRate((double)Wanted, Dec);
     return (uint32_t)lround(Rate/(double)(Dec>1 ? Dec : 1)); }

   // -----------------------------------------------------------------------------------------------
   // Dual-tuner mode: a much stricter set of valid combinations
   //
   // In Dual_Tuner mode the two tuners share one ADC clock, so the API imposes a fixed
   // relationship between the ADC rate, the analog bandwidth, the IF and the internal
   // decimation. Unlike single-tuner mode, IF_Zero is NOT among the valid combinations, and a
   // combination outside this table makes sdrplay_api_Init() fail. The pairs come from the
   // reference dual-tuner-recorder implementation and are checked against them here.
   //
   // At a 2 MHz ADC rate the usable settings are:
   //     bw 0.200 or 0.300  ->  IF 450 kHz, decimate by 4  ->  500 ksps out per tuner
   //     bw 0.600           ->  IF 450 kHz, decimate by 2  -> 1000 ksps out per tuner
   //
   // 500 ksps with a 200 kHz filter is ample for one OGN channel, whose signal is about 100 kHz
   // wide, so the default is bw 0.200. The trade against single-tuner mode is real: one tuner
   // there gives 2 MSps covering both channels at once, whereas dual mode gives each channel its
   // own tuner and its own narrow filter at a quarter of the rate.
   // -----------------------------------------------------------------------------------------------

   sdrplay_api_Bw_MHzT  DualBw;          // analog bandwidth per tuner in dual mode
   sdrplay_api_If_kHzT  DualIf;          // IF, which dual mode cannot set to zero
   int                  DualDecimation;  // internal decimation the API applies

   void DualCombo(double &Rate, int &Decimation)
   { Rate=2000000.0;
     switch(DualBw)
     { case sdrplay_api_BW_0_600: DualIf=sdrplay_api_IF_0_450; Decimation=2; break;
       case sdrplay_api_BW_0_300: DualIf=sdrplay_api_IF_0_450; Decimation=4; break;
       case sdrplay_api_BW_0_200:
       default:                    DualBw=sdrplay_api_BW_0_200;
                                   DualIf=sdrplay_api_IF_0_450; Decimation=4; break; }
     DualDecimation=Decimation; }

   // -----------------------------------------------------------------------------------------------
   // Open / Close
   // -----------------------------------------------------------------------------------------------

   int Open(uint32_t DeviceIndex=0, uint32_t Frequency=868300000, uint32_t SampleRate=2000000)
   { Close();

     this->DeviceIndex=DeviceIndex;
     Shift=8;                                  // measured, not assumed; see the note above

     // --- connect to the API service -----------------------------------------------------------
     sdrplay_api_ErrT Err=sdrplay_api_Open();
     if(Err!=sdrplay_api_Success)
     { printf("RSPduo::Open() => sdrplay_api_Open() failed: %s\n", sdrplay_api_GetErrorString(Err));
       if(Err==sdrplay_api_ServiceNotResponding)
         printf("RSPduo::Open() => the API service is not running: sudo systemctl start sdrplay\n");
       return -1; }
     ApiOpen=1;

     float Version=0;
     if(sdrplay_api_ApiVersion(&Version)==sdrplay_api_Success)
     { ApiVersion=Version;
       if(fabsf(Version-SDRPLAY_API_VERSION)>0.001f)
         printf("RSPduo::Open() => API %4.2f, headers %4.2f\n", Version, SDRPLAY_API_VERSION); }

     // --- pick the device ----------------------------------------------------------------------
     Err=sdrplay_api_LockDeviceApi();
     if(Err!=sdrplay_api_Success)
     { printf("RSPduo::Open() => LockDeviceApi() failed: %s\n", sdrplay_api_GetErrorString(Err));
       Close(); return -1; }

     sdrplay_api_DeviceT Devices[SDRPLAY_MAX_DEVICES];
     unsigned int Count=0;
     Err=sdrplay_api_GetDevices(Devices, &Count, SDRPLAY_MAX_DEVICES);
     if(Err!=sdrplay_api_Success)
     { printf("RSPduo::Open() => GetDevices() failed: %s\n", sdrplay_api_GetErrorString(Err));
       sdrplay_api_UnlockDeviceApi(); Close(); return -1; }

     if(Count==0)
     { printf("RSPduo::Open() => the API service sees no RSP device\n");
       sdrplay_api_UnlockDeviceApi(); Close(); return -1; }

     if(DeviceIndex>=Count)
     { printf("RSPduo::Open() => device #%d requested, only %d present\n", DeviceIndex, Count);
       sdrplay_api_UnlockDeviceApi(); Close(); return -1; }

     Device=Devices[DeviceIndex];
     HardwareVer=Device.hwVer;
     strncpy(DeviceSerNo, Device.SerNo, SDRPLAY_MAX_SER_NO_LEN-1); DeviceSerNo[SDRPLAY_MAX_SER_NO_LEN-1]=0;

     // Single_Tuner uses tuner A alone; Dual_Tuner drives both tuners off one clock, which is
     // the RSPduo's distinguishing feature and needs no second device. In dual mode the device
     // must be declared as Tuner_Both and rspDuoSampleFreq must carry the shared rate before the
     // device is selected - the API rejects the init otherwise.
     DualMode = ((RspDuoMode==sdrplay_api_RspDuoMode_Dual_Tuner) ? 1 : 0);
     if(DualMode)
     { double Rate; int Dec;
       DualCombo(Rate, Dec);
       Device.tuner       = sdrplay_api_Tuner_Both;
       Device.rspDuoMode  = sdrplay_api_RspDuoMode_Dual_Tuner;
       Device.rspDuoSampleFreq = Rate; }
     else
     { Device.tuner       = sdrplay_api_Tuner_A;
       Device.rspDuoMode  = (sdrplay_api_RspDuoModeT)RspDuoMode;
       Device.rspDuoSampleFreq = 0.0; }

     Err=sdrplay_api_SelectDevice(&Device);
     sdrplay_api_UnlockDeviceApi();
     if(Err!=sdrplay_api_Success)
     { printf("RSPduo::Open() => SelectDevice() failed: %s\n", sdrplay_api_GetErrorString(Err));
       Close(); return -1; }

     printf("RSPduo::Open(%d,%d,%d) => %s, S/N %s\n",
            DeviceIndex, Frequency, SampleRate, modelName(HardwareVer), DeviceSerNo);

     // --- parameters ---------------------------------------------------------------------------
     Err=sdrplay_api_GetDeviceParams(Device.dev, &Params);
     if(Err!=sdrplay_api_Success || Params==0)
     { printf("RSPduo::Open() => GetDeviceParams() failed: %s\n", sdrplay_api_GetErrorString(Err));
       Close(); return -1; }

     int Decimation=1;
     double Rate;
     sdrplay_api_Bw_MHzT BwType;
     sdrplay_api_If_kHzT IfType;

     if(DualMode)
     { // Dual mode picks its own rate/bandwidth/IF/decimation: the API only accepts the fixed
       // combinations documented above, so nothing here is negotiable.
       //
       // 0.6 MHz is the default bandwidth because the two European OGN channels are 200 kHz
       // apart and a 0.2 MHz filter centred between them would sit on neither. It gives 1 Msps
       // per tuner instead of 500 ksps, which is still well above what one OGN channel needs.
       //
       // Tuner B defaults to the second OGN channel, Frequency + 100 kHz, which for a 868.300
       // centre is 868.400 - so out of the box the two tuners carry the two OGN channels.
       // Selection diversity needs both tuners looking at the same channel; without it the second
       // tuner defaults to the neighbouring OGN channel.
       if(Diversity)      FreqB=(double)Frequency;
       else if(FreqB==0.0) FreqB=(double)Frequency+100000.0;
       DualCombo(Rate, Decimation);
       BwType=DualBw; IfType=DualIf;
       printf("RSPduo::Open() => dual-tuner mode: %5.3f MHz ADC, bw %d kHz, IF %d kHz, decimate by %d "
              "=> %5.3f Msps per tuner\n",
              1e-6*Rate, (int)BwType, (int)IfType, Decimation,
              1e-6*Rate/(double)Decimation); }
     else
     { Rate=SnapSampleRate((double)SampleRate, Decimation);
       double Delivered=Rate/(double)(Decimation>1 ? Decimation : 1);
       if((uint32_t)fabs(Delivered-SampleRate)>1)
         printf("RSPduo::Open() => %d Hz requested, the RSPduo gives %8.1f Hz%s\n",
                SampleRate, Delivered,
                Decimation>1 ? " (2 MHz ADC, decimation factor 2)" : "");
       BwType=sdrplay_api_BW_1_536; IfType=sdrplay_api_IF_Zero; }

     Params->devParams->fsFreq.fsHz    = Rate;
     Params->devParams->ppm            = (double)FreqCorr;
     Params->devParams->mode           = sdrplay_api_BULK;   // bulk is the robust choice on USB

     if(Params->rxChannelA==0)
     { printf("RSPduo::Open() => the API returned no channel A parameters\n"); Close(); return -1; }

     // Channel A. Both channels are configured identically here; in dual mode the API then
     // overwrites B with A's settings during Init(), and the differences are re-applied
     // afterwards by ReconfigureTunerB().
     for(int Ch=0; Ch<2; Ch++)
     { sdrplay_api_RxChannelParamsT *Chan=(Ch==0) ? Params->rxChannelA : Params->rxChannelB;
       if(Chan==0) continue;
       sdrplay_api_TunerParamsT   &Tuner=Chan->tunerParams;
       sdrplay_api_ControlParamsT &Ctrl =Chan->ctrlParams;

       Tuner.rfFreq.rfHz      = (Ch==0) ? (double)Frequency : FreqB;
       Tuner.rfFreq.syncUpdate= 1;
       Tuner.bwType           = BwType;
       Tuner.ifType           = IfType;
       Tuner.loMode           = sdrplay_api_LO_Auto;
       Tuner.gain.gRdB        = GRdB_set;
       Tuner.gain.LNAstate    = (unsigned char)LNAstate;
       Tuner.gain.minGr       = sdrplay_api_NORMAL_MIN_GR;

       // DC offset / IQ imbalance correction. Left at the API default, but made explicit so the
       // behaviour is visible: the correction notches a few kHz at the exact centre frequency
       // (868.300), which sits BETWEEN the two OGN channels and therefore does no harm.
       Ctrl.dcOffset.DCenable = DcOffsetCorrect ? 1 : 0;
       Ctrl.dcOffset.IQenable = DcOffsetCorrect ? 1 : 0;
       // In dual mode the decimation belongs to the API: the dual-tuner combination (ADC rate,
       // bandwidth, IF) already fixes it, and the host-side decimation controls have to stay OFF.
       // Programming DualCombo's factor on top decimates a second time - measured with the probe,
       // dual mode, bw 0.6 MHz, 2 MHz ADC: 1.008 Msps delivered with the controls off, 0.504 Msps
       // with factor 2 set. So the stream arrived at half the rate the combination says and at
       // half the rate this file reported, which is why every slot read in dual mode took twice
       // as long as the sample count implied. Single-tuner mode has no such combination and does
       // need the controls; there the factor is the only way below 2 MHz.
       int HostDecimation = DualMode ? 1 : Decimation;
       Ctrl.decimation.enable = (HostDecimation>1) ? 1 : 0;
       Ctrl.decimation.decimationFactor = (unsigned char)(HostDecimation>1 ? HostDecimation : 1);
       Ctrl.agc.enable        = sdrplay_api_AGC_DISABLE;   // manual gain: ogn-rf owns the gain

       // The bias tee feeds the TUNER 2 port, and its update reason is per-tuner, so the flag
       // belongs to channel B. Only fall back to A when there is no channel B (single-tuner mode).
       if( (Params->rxChannelB!=0 && Ch==1) || (Params->rxChannelB==0 && Ch==0) )
         Chan->rspDuoTunerParams.biasTEnable = (BiasTee>0) ? 1 : 0;
     }

     SampleRateApplied = Rate/(double)(Decimation>1 ? Decimation : 1);
     SamplePeriod=1.0/SampleRateApplied;
     FreqApplied=(double)Frequency;
     FreqBApplied=DualMode ? FreqB : 0.0;
     FreqRaster=0;

     // --- sample rings -------------------------------------------------------------------------
     // Size by time, not by samples, so dual mode (a quarter of the rate) still buffers about half
     // a second per tuner.
     // A ring has to hold MORE than one ogn-rf read window (RF.OGN.SensTime, 0.85 s by default).
     // In selection diversity both rings are drained in one call, and they must be drained
     // sequentially, so while tuner A is being read for 0.85 s tuner B is still filling. If the
     // ring were smaller than the window, B would overflow during A's read, discard its samples
     // and force a second full wait - which measured as 2 reads in 22 s and a 9 % Live Time.
     // Two seconds of headroom costs 4 MB per tuner and removes that entirely.
     size_t RingBytes=(size_t)(SampleRateApplied*2.0*2.0);
     if(RingBytes<(1u<<20)) RingBytes=(1u<<20);        // never smaller than 1 MiB
     if(Ring[0].Allocate(RingBytes)<0)
     { printf("RSPduo::Open() => cannot allocate the tuner A ring\n"); Close(); return -1; }
     if(DualMode && Ring[1].Allocate(RingBytes)<0)
     { printf("RSPduo::Open() => cannot allocate the tuner B ring\n"); Close(); return -1; }
     TunerBEnabled = DualMode;
     BOverflows=0;

     // --- start the stream ---------------------------------------------------------------------
     sdrplay_api_CallbackFnsT CbFns;
     CbFns.StreamACbFn=StaticStreamACb;
     CbFns.StreamBCbFn=DualMode ? StaticStreamBCb : 0;   // tuner B only exists in dual mode
     CbFns.EventCbFn  =StaticEventCb;

     StartTime=SampleTime=PrevTime=getTime();

     Err=sdrplay_api_Init(Device.dev, &CbFns, this);
     if(Err!=sdrplay_api_Success)
     { printf("RSPduo::Open() => Init() failed: %s\n", sdrplay_api_GetErrorString(Err));
       Close(); return -1; }
     Streaming=1;

     // sdrplay_api_Init() RESETS channel B to channel A's values. Anything tuner B needs that
     // differs from A - here its centre frequency - must therefore be written AFTER Init returns
     // and pushed down with a tuner-B update. Doing it before Init silently leaves both tuners on
     // tuner A's frequency, which looks like it works and receives the same band twice.
     if(DualMode) { if(ReconfigureTunerB()<0) { Close(); return -1; } }

     BuildGainTable();

     printf("RSPduo::Open() => tuner A %8.3f MHz%s, %8.3f Msps, gain reduction %4.1f dB, LNA state %d\n",
            1e-6*FreqApplied, DualMode ? " + tuner B both streaming" : "",
            1e-6*SampleRateApplied, (double)GRdB_set, LNAstate);
     return 1; }

   void Close(void)
   { // Stop both rings first, so a Read() blocked on either wakes up instead of hanging.
     for(int T=0; T<2; T++) Ring[T].Stop();

     if(Streaming && Device.dev) sdrplay_api_Uninit(Device.dev);
     Streaming=0; Params=0;

     if(Device.dev)
     { sdrplay_api_ReleaseDevice(&Device); memset(&Device, 0, sizeof(Device)); }

     if(ApiOpen) { sdrplay_api_Close(); ApiOpen=0; }

     for(int T=0; T<2; T++) Ring[T].Free();
     for(int T=0; T<2; T++) { free(DivBuf[T]); DivBuf[T]=0; }
     DivBufBytes=0;
     TunerBEnabled=0; Gains=0; }

   // -----------------------------------------------------------------------------------------------
   // Gain
   //
   // -----------------------------------------------------------------------------------------------
   // Dual mode: push tuner B's differences after Init()
   // -----------------------------------------------------------------------------------------------

   int ReconfigureTunerB(void)
   { if(Params==0 || Params->rxChannelB==0) return 0;

     sdrplay_api_ReasonForUpdateT Reason=sdrplay_api_Update_None;

     if(fabs(Params->rxChannelB->tunerParams.rfFreq.rfHz - FreqB)>0.5)
     { Params->rxChannelB->tunerParams.rfFreq.rfHz=FreqB;
       Params->rxChannelB->tunerParams.rfFreq.syncUpdate=1;
       Reason=(sdrplay_api_ReasonForUpdateT)(Reason|sdrplay_api_Update_Tuner_Frf); }

     if(Reason==sdrplay_api_Update_None) return 0;

     sdrplay_api_ErrT Err=sdrplay_api_Update(Device.dev, sdrplay_api_Tuner_B,
                                             Reason, sdrplay_api_Update_Ext1_None);
     if(Err!=sdrplay_api_Success)
     { printf("RSPduo::ReconfigureTunerB() => Update(%08x) failed: %s\n",
              (unsigned)Reason, sdrplay_api_GetErrorString(Err));
       return -1; }
     printf("RSPduo::Open() => tuner B tuned to %8.3f MHz (channel B re-applied after Init)\n",
            1e-6*FreqB);
     return 0; }

   // The RSPduo has no gain table: gain is the pair (IF gain reduction, LNA state). What it does
   // have is a simple, measurable law, established on this unit at 868.3 MHz (docs/hardware.md):
   //
   //     total gain [dB] = GainOffset - gRdB          with LNA state 0
   //
   // where GainOffset measured 115.8 dB, so gRdB 20 gives 95.8 dB and gRdB 59 gives 56.8 dB.
   // GainOffset is not hard-coded: the API reports the resulting total gain on every gain change,
   // and ClassEventCb recalibrates the offset from that report. So the model is self-correcting
   // across units and bands rather than trusting a constant measured once.
   //
   // LNA state is held at 0 deliberately. It is the lowest-noise setting, and on a weak, noise-
   // dominated band - which 868 MHz is here - noise figure is what limits reception, not gain.
   // Measured at gRdB 20, stepping the LNA state down did not lower the noise floor at all, it
   // only moved the reported gain, so ogn-rf's single gain number is spent on the IF stage.
   // -----------------------------------------------------------------------------------------------

   void BuildGainTable(void)
   { Gains=0;
     // What ogn-rf can ask for: the totals reachable by sweeping gRdB at LNA state 0.
     double Max=GainOffset-GR_DB_MIN;
     double Min=GainOffset-GR_DB_MAX;
     if(!(Max>Min)) return;
     int Steps=32;
     for(int Idx=0; Idx<Steps && Idx<64; Idx++)
     { double dB=Max + (Min-Max)*((double)Idx/(double)(Steps-1));
       Gain[Idx]=(int)lround(10.0*dB); }
     Gains=Steps<64 ? Steps : 64; }

   void PrintGains(void) const
   { printf("RSPduo::Gain[%d] =", Gains);
     for(int Idx=0; Idx<Gains; Idx++) printf(" %+5.1f", 0.1*Gain[Idx]);
     printf(" [dB]\n"); }

   int getTunerGains(int *Gain=0)
   { if(Gain) for(int Idx=0; Idx<Gains; Idx++) Gain[Idx]=this->Gain[Idx];
     return Gains; }

   int getTunerGain(void) { return (int)lround(10.0*GaindBApplied); }

   // Accepts the gain ogn-rf asks for in 0.1 dB, treats it as a total gain, and spends it on the
   // IF gain reduction at LNA state 0. A request above the maximum is clamped to maximum gain
   // rather than rejected: on a quiet band the useful setting is the top of the range, and
   // silently doing something quieter than asked would be the more surprising behaviour.
   int setTunerGain(int Requested)
   { if(Params==0 || Params->rxChannelA==0) return -1;

     double Want=0.1*(double)Requested;
     double GRdB=GainOffset-Want;
     if(GRdB<GR_DB_MIN) GRdB=GR_DB_MIN;
     if(GRdB>GR_DB_MAX) GRdB=GR_DB_MAX;

     GRdB_set=(int)lround(GRdB);
     LNAstate=0;

     Params->rxChannelA->tunerParams.gain.gRdB     = GRdB_set;
     Params->rxChannelA->tunerParams.gain.LNAstate = (unsigned char)LNAstate;

     if(Streaming)
     { sdrplay_api_ErrT Err=sdrplay_api_Update(Device.dev, sdrplay_api_Tuner_A,
                                               sdrplay_api_Update_Tuner_Gr, sdrplay_api_Update_Ext1_None);
       if(Err!=sdrplay_api_Success)
       { printf("RSPduo::setTunerGain(%d) => Update() failed: %s\n", Requested, sdrplay_api_GetErrorString(Err));
         return -1; } }

     BuildGainTable();
     return 0; }

   int setTunerGainMode(int Manual=1) { (void)Manual; return 0; }   // AGC is disabled in Open()
   int setTunerGainManual(int Manual=1) { return setTunerGainMode(Manual); }
   int setTunerGainAuto(void)           { return setTunerGainMode(0); }

   // -----------------------------------------------------------------------------------------------
   // Frequency
   // -----------------------------------------------------------------------------------------------

   int setCenterFreq(uint32_t Frequency)
   { if(FreqRaster) { uint32_t Div=(Frequency+(FreqRaster>>1))/FreqRaster; Frequency=Div*FreqRaster; }
     FreqApplied=(double)Frequency;
     if(Params==0 || Params->rxChannelA==0) return -1;

     // Retuning is the most expensive thing the acquisition loop asks of this backend, and asking
     // for the frequency it is already on is the common case rather than an edge case: the
     // European OGN plan puts 868.200 and 868.400 inside one capture window, so ogn-rf works out
     // the same centre frequency for ever and still calls this once per slot.
     //
     // What an unnecessary Update() costs was measured with the acquisition loop traced: 0.40 s
     // per call in single-tuner mode, 0.80 s in dual - up to half of a one-second slot spent
     // restarting a stream that did not need to move. It also truncates the stream: the API
     // restarts it and the callback returns with reset=1, so the slot's buffered samples are
     // discarded as well as the time. In dual mode the ring was measured taking 500 ksps while
     // the API itself provides 1.008 Msps, which is the signature of half the stream being lost
     // to those restarts.
     //
     // The LimeSDR backend guards its own 60 ms retune the same way, for the same reason.
     bool BFollows=(Diversity && TunerBEnabled && Params->rxChannelB!=0);
     if( (Params->rxChannelA->tunerParams.rfFreq.rfHz==(double)Frequency)
      && (!BFollows || (Params->rxChannelB->tunerParams.rfFreq.rfHz==(double)Frequency)) )
     { if(BFollows) { FreqB=(double)Frequency; FreqBApplied=FreqB; }
       return 0; }

     Params->rxChannelA->tunerParams.rfFreq.rfHz=(double)Frequency;
     Params->rxChannelA->tunerParams.rfFreq.syncUpdate=1;
     if(Streaming)
     { sdrplay_api_ErrT Err=sdrplay_api_Update(Device.dev, sdrplay_api_Tuner_A,
                                               sdrplay_api_Update_Tuner_Frf, sdrplay_api_Update_Ext1_None);
       if(Err!=sdrplay_api_Success)
       { printf("RSPduo::setCenterFreq(%d) => Update() failed: %s\n", Frequency, sdrplay_api_GetErrorString(Err));
         return -1; } }

     // Diversity is only meaningful if both tuners see the same channel, so tuner B follows A.
     if(BFollows)
     { FreqB=(double)Frequency; FreqBApplied=FreqB;
       Params->rxChannelB->tunerParams.rfFreq.rfHz=(double)Frequency;
       Params->rxChannelB->tunerParams.rfFreq.syncUpdate=1;
       if(Streaming)
         sdrplay_api_Update(Device.dev, sdrplay_api_Tuner_B,
                            sdrplay_api_Update_Tuner_Frf, sdrplay_api_Update_Ext1_None); }
     return 0; }

   uint32_t getCenterFreq(void) { return (uint32_t)FreqApplied; }

   // Tuner B, dual mode only. Setting it before Open() also sets the frequency Open() will use
   // for channel B; setting it afterwards retunes channel B live.
   void setCenterFreqB(uint32_t Frequency)
   { FreqB=(double)Frequency;
     if(!DualMode || Params==0 || Params->rxChannelB==0) return;
     FreqBApplied=(double)Frequency;
     ReconfigureTunerB(); }

   uint32_t getCenterFreqB(void) { return (uint32_t)(DualMode ? FreqBApplied : 0); }
   bool     haveTunerB(void)     { return TunerBEnabled!=0; }

   // Real ppm control, straight into the API - no software shift, unlike the LimeSDR backend.
   int setFreqCorrection(int PPM)
   { FreqCorr=PPM;
     if(Params==0) return 0;                 // recorded, applied at the next Open()
     Params->devParams->ppm=(double)PPM;
     if(Streaming)
     { sdrplay_api_ErrT Err=sdrplay_api_Update(Device.dev, sdrplay_api_Tuner_A,
                                               sdrplay_api_Update_Dev_Ppm, sdrplay_api_Update_Ext1_None);
       if(Err!=sdrplay_api_Success) return -1; }
     return 0; }

   int getFreqCorrection(void) { return FreqCorr; }

   // -----------------------------------------------------------------------------------------------
   // Sample rate
   // -----------------------------------------------------------------------------------------------

   int setSampleRate(uint32_t SampleRate)
   { int Decimation=1;
     double Rate=SnapSampleRate((double)SampleRate, Decimation);
     if(Params==0) { SampleRateApplied=Rate/(double)Decimation; SamplePeriod=1.0/SampleRateApplied; return 0; }
     Params->devParams->fsFreq.fsHz=Rate;
     if(Params->rxChannelA)
     { Params->rxChannelA->ctrlParams.decimation.enable=(Decimation>1)?1:0;
       Params->rxChannelA->ctrlParams.decimation.decimationFactor=(unsigned char)(Decimation>1?Decimation:1); }
     SampleRateApplied=Rate/(double)Decimation;
     SamplePeriod=1.0/SampleRateApplied;
     if(Streaming)
     { sdrplay_api_ErrT Err=sdrplay_api_Update(Device.dev, sdrplay_api_Tuner_A,
                                               sdrplay_api_Update_Dev_Fs, sdrplay_api_Update_Ext1_None);
       if(Err!=sdrplay_api_Success)
       { printf("RSPduo::setSampleRate(%d) => Update() failed: %s\n", SampleRate, sdrplay_api_GetErrorString(Err));
         return -1; } }
     return 0; }

   uint32_t getSampleRate(void) { return (uint32_t)lround(SampleRateApplied); }

   // -----------------------------------------------------------------------------------------------
   // Options with no RSPduo hardware behind them
   // -----------------------------------------------------------------------------------------------

   // Bias tee: 4.7 V at 100 mA, on the TUNER 2 port only. Live changes go through
   // sdrplay_api_Update_RspDuo_BiasTControl. That reason is in the API's per-tuner group, so it
   // is applied to TUNER B - the port the 4.7 V comes out of - and NOT to tuner A.
   int setBiasTee(int On=1)
   { BiasTee=On;
     if(Params==0) return 0;                              // recorded; applied at the next Open()
     sdrplay_api_RxChannelParamsT *Port=(Params->rxChannelB!=0) ? Params->rxChannelB
                                                                : Params->rxChannelA;
     if(Port==0) return 0;
     Port->rspDuoTunerParams.biasTEnable = (On>0) ? 1 : 0;
     if(!Streaming) return 0;
     sdrplay_api_TunerSelectT Which=(Params->rxChannelB!=0) ? sdrplay_api_Tuner_B
                                                            : sdrplay_api_Tuner_A;
     sdrplay_api_ErrT Err=sdrplay_api_Update(Device.dev, Which,
                                             sdrplay_api_Update_RspDuo_BiasTControl,
                                             sdrplay_api_Update_Ext1_None);
     if(Err!=sdrplay_api_Success)
     { printf("RSPduo::setBiasTee(%d) => Update() failed: %s\n", On, sdrplay_api_GetErrorString(Err));
       return -1; }
     printf("RSPduo::setBiasTee(%d) => 4.7 V %s on the Tuner 2 port (tuner %s)\n",
            On, On>0 ? "ON" : "OFF", (Params->rxChannelB!=0) ? "B" : "A");
     return 0; }
   int setOffsetTuning(int ON=1) { (void)ON; return 0; }

   int setRspDuoMode(int Mode) { RspDuoMode=Mode; return 0; }   // takes effect at the next Open()

   // -----------------------------------------------------------------------------------------------
   // Identification, for the status page
   // -----------------------------------------------------------------------------------------------

   const char *getTunerTypeName(void) { return modelName(HardwareVer); }
   int         getTunerType(void)     { return (int)HardwareVer; }

   int getUsbStrings(char *Manufacturer, char *Product, char *Serial)
   { if(Manufacturer) strcpy(Manufacturer, "SDRplay");
     if(Product)      strcpy(Product, modelName(HardwareVer));
     if(Serial)       strcpy(Serial, DeviceSerNo);
     return 0; }

   // The RSPduo has no crystal-trim interface of its own; the ppm control is the API's.
   int getXtalFreq(uint32_t &RtlFreq, uint32_t &TunerFreq) { RtlFreq=0; TunerFreq=0; return -1; }

   // -----------------------------------------------------------------------------------------------
   // Streaming: drain the ring
   // -----------------------------------------------------------------------------------------------

   // Enable selection diversity. Only meaningful in dual-tuner mode; the caller is expected to
   // keep both tuners on one frequency (setCenterFreq does that once this is on).
   void setSelectionDiversity(int On)
   { Diversity = On ? 1 : 0; DivLastTuner=0; }

   // Effective only when the open really is dual-tuner; the flag itself is just the request.
   bool haveDiversity(void) { return Diversity!=0 && TunerBEnabled!=0; }
   int  getDivLastTuner(void) { return DivLastTuner; }
   double getDivPower(int Tuner) { return Tuner ? DivPowerB : DivPowerA; }
   uint64_t getDivChoseA(void) { return DivChoseA; }
   uint64_t getDivChoseB(void) { return DivChoseB; }
   uint64_t getDivSwitches(void) { return DivSwitches; }

   // Energy of a buffer of interleaved 8-bit unsigned I/Q. The decision metric is the STRONGEST
   // short block rather than the whole-buffer mean, because an OGN packet is a burst: averaging
   // over 0.85 s of mostly noise washes out exactly the thing being selected for.
   static double BlockPeakPowerdB(const uint8_t *Buf, int Samples, int BlockSamples)
   { if(Buf==0 || Samples<=0) return -1e9;
     if(BlockSamples<=0) BlockSamples=1024;
     double Best=0;
     for(int Start=0; Start<Samples; Start+=BlockSamples)
     { int End=Start+BlockSamples; if(End>Samples) End=Samples;
       double Acc=0;
       for(int Idx=Start; Idx<End; Idx++)
       { int vi=(int)Buf[2*Idx]-128, vq=(int)Buf[2*Idx+1]-128;
         Acc += (double)vi*vi + (double)vq*vq; }
       if(Acc>Best) Best=Acc; }
     return 10.0*log10(Best + 1e-30); }

   // Flush anything already queued, for one tuner (default A) or both.
   int ResetBuffer(int Tuner=0)
   { // Selection diversity reads both rings each call, so both have to be flushed together or the
     // losing tuner's stale samples would be compared against the winner's fresh ones.
     if(Tuner<0 || Diversity) { Ring[0].Flush(); Ring[1].Flush(); }
     else if(Tuner<2) Ring[Tuner].Flush();
     return 0; }

   // Blocks until Samples interleaved 8-bit unsigned I/Q are available. Tuner 0 is A, 1 is B;
   // ogn-rf reads tuner A, and tuner B is available through the same call.
   int Read(uint8_t *Buffer, int Samples, int Tuner=0)
   { if(!Streaming || Tuner<0 || Tuner>1) return -1;
     if(!Ring[Tuner].isOpen()) return -1;
     return Ring[Tuner].Pop(Buffer, Samples); }

   // Selection diversity: drain both tuners over the same window and hand back whichever saw more
   // signal. Both rings are drained every call - the loser cannot be left to overflow, or it could
   // never win a later slot when it is the one in a fade-free position.
   int ReadDiversity(SampleBuffer<uint8_t> &Out, int Samples)
   { if(!Streaming || !Ring[0].isOpen() || !Ring[1].isOpen()) return -1;
     int Use=Samples & 0xFFFFFF00;
     if(Use<=0) return 0;
     size_t Want=2*(size_t)Use;
     if(DivBufBytes<Want)
     { for(int T=0; T<2; T++) { free(DivBuf[T]); DivBuf[T]=0; }
       DivBuf[0]=(uint8_t *)malloc(Want);
       DivBuf[1]=(uint8_t *)malloc(Want);
       if(DivBuf[0]==0 || DivBuf[1]==0) { DivBufBytes=0; return -1; }
       DivBufBytes=Want; }

     // A first: it blocks until the slot has been captured. Both tuners stream off the same clock,
     // so by the time A is complete B already holds the same window and its read returns at once.
     int GotA=Ring[0].Pop(DivBuf[0], Use);
     int GotB=Ring[1].Pop(DivBuf[1], Use);

     DivPowerA=(GotA>0) ? BlockPeakPowerdB(DivBuf[0], GotA, 1024) : -1e9;
     DivPowerB=(GotB>0) ? BlockPeakPowerdB(DivBuf[1], GotB, 1024) : -1e9;

     int Choose;
     if(GotA<=0)            Choose=1;
     else if(GotB<=0)       Choose=0;
     else if(DivLastTuner==0) Choose=(DivPowerB > DivPowerA+DivSwitchdB) ? 1 : 0;
     else                     Choose=(DivPowerA > DivPowerB+DivSwitchdB) ? 0 : 1;

     if(Choose!=DivLastTuner) DivSwitches++;
     DivLastTuner=Choose;
     if(Choose) DivChoseB++; else DivChoseA++;

     int      Got=Choose ? GotB : GotA;
     uint8_t *Src=Choose ? DivBuf[1] : DivBuf[0];
     memcpy(Out.Data, Src, 2*(size_t)Got);

     double Time=getTime();
     Out.Full=Got*2;
     Out.Rate=getSampleRate();
     Out.Freq=getCenterFreq();
     Out.Time=Time-(double)Got/Out.Rate;
     return Got; }

   int Read(SampleBuffer<uint8_t> &Buffer, int Samples, int Tuner=0)
   { if(Buffer.Allocate(2,Samples)<=0) return 0;
     if(haveDiversity() && Tuner==0) return ReadDiversity(Buffer, Samples);
     int ReadSamples=Read(Buffer.Data, Samples, Tuner);
     double Time=getTime();
     if(ReadSamples>0)
     { Buffer.Full=ReadSamples*2;
       Buffer.Rate=getSampleRate();
       Buffer.Freq=(Tuner==1) ? getCenterFreqB() : getCenterFreq();
       Buffer.Time=Time-(double)ReadSamples/Buffer.Rate; }
     return ReadSamples; }

   // Samples taken off tuner A and B so far, for the status page.
   uint64_t getBytesRead (int Tuner=0) { return Ring[Tuner<2?Tuner:0].BytesRead; }
   uint64_t getOverflows (int Tuner=0) { return Ring[Tuner<2?Tuner:0].Overflows; }
   uint64_t getBuffers   (int Tuner=0) { return Ring[Tuner<2?Tuner:0].Buffers; }

   // ogn-rf reads synchronously, so acquisition is already running once Open() returns.
   int CancelAsync(void)
   { Ring[0].Stop(); Ring[1].Stop();
     return 0; }

   // Accepted for interface parity with RTLSDR; with a synchronous Read() the loop lives in
   // ogn-rf.cc, so there is nothing to start here.
   int ReadAsync(int (*Callback)(uint8_t *Buffer, int Samples, double SampleTime, double SamplePeriod, void *Contex)=0,
                 void *Contex=0, int Buffers=0, int BlockSize=0)
   { (void)Buffers; (void)BlockSize;
     this->Callback=Callback; this->CallbackContext=Contex;
     StartTime=SampleTime=PrevTime=getTime();
     return Streaming ? 0 : -1; }

} ;

// ==================================================================================================

#endif // __RSPDUO_H__
