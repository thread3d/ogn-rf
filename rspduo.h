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
//   * Bias tee - the RSPduo has no bias tee (the RSP2 and RSPdx do), so setBiasTee() records
//     the request and reports success; the config option is accepted and ignored.
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

   // --- ring buffer: the bridge from the API's callback thread to our blocking Read() ---------
   uint8_t      *Ring;              // interleaved 8-bit unsigned I/Q, as Read() wants it
   size_t        RingBytes;         // capacity in bytes
   size_t        RingHead;          // read position
   size_t        RingFull;          // bytes currently stored
   pthread_mutex_t RingLock;
   pthread_cond_t  RingCond;        // signalled by the callback, waited on by Read()
   int           Stopped;           // set by Close()/CancelAsync() to unblock a waiting Read()

   uint64_t      Overflows;         // buffers dropped because the ring was full
   uint64_t      Buffers;           // stream buffers received
   uint64_t      BytesRead;         // counts number of bytes delivered (1 sample = 2 bytes: I/Q)

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

   // --- what the device actually did, for the status page --------------------------------------
   double        SampleRateApplied;  // [Hz]
   double        FreqApplied;        // [Hz] nominal centre frequency
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
     Ring=0; RingBytes=0; RingHead=0; RingFull=0; Stopped=0;
     pthread_mutex_init(&RingLock, 0); pthread_cond_init(&RingCond, 0);
     Overflows=0; Buffers=0; BytesRead=0;
     Callback=0; CallbackContext=0;
     #ifndef __MACH__
     RefClock=CLOCK_REALTIME;
     #endif
     AverPeriod=100.0;
     SamplePeriod=1.0/2000000.0; SampleTime=0; StartTime=0; PrevTime=0; SampleTime_DMS=0.0001*0.0001;
     GainOffset=115.8; GRdB_set=GR_DB_MIN; LNAstate=0; BiasTee=(-1); DcOffsetCorrect=1;
     RspDuoMode=sdrplay_api_RspDuoMode_Single_Tuner;
     SampleRateApplied=0; FreqApplied=0; FreqCorr=0; GaindBApplied=0; }

  ~RSPDuo()
   { Close();
     pthread_mutex_destroy(&RingLock); pthread_cond_destroy(&RingCond); }

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
     if(This) This->ClassStreamACb(xi, xq, params, numSamples, reset); }

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

   void ClassStreamACb(short *xi, short *xq, sdrplay_api_StreamCbParamsT *params,
                       unsigned int numSamples, unsigned int reset)
   { (void)params;                               // firstSampleNum/grChanged are not needed here
     if(numSamples==0) return;
     size_t Bytes=2*(size_t)numSamples;

     pthread_mutex_lock(&RingLock);
     if(Stopped) { pthread_mutex_unlock(&RingLock); return; }

     // reset!=0 means the API restarted the stream (a tuner or rate change). The samples in the
     // ring belong to the old configuration, so throw them away: keeping them would splice two
     // different centre frequencies into one continuous stream.
     if(reset) { RingHead=0; RingFull=0; }

     // Not enough room: drop the whole backlog rather than deliver a torn buffer. This is a real
     // overrun (the reader fell behind), so it is counted and reported.
     if(RingFull+Bytes>RingBytes)
     { RingHead=0; RingFull=0; Overflows++; }

     size_t Tail=(RingHead+RingFull)%RingBytes;
     size_t First=Bytes;
     if(First>RingBytes-Tail) First=RingBytes-Tail;
     size_t Second=Bytes-First;

     for(size_t Idx=0; Idx<First/2; Idx++)
     { int vi=(xi[Idx]>>Shift)+128; if(vi<0) vi=0; else if(vi>255) vi=255;
       int vq=(xq[Idx]>>Shift)+128; if(vq<0) vq=0; else if(vq>255) vq=255;
       Ring[Tail+2*Idx]  =(uint8_t)vi;
       Ring[Tail+2*Idx+1]=(uint8_t)vq; }
     for(size_t Idx=0; Idx<Second/2; Idx++)
     { size_t Src=First/2+Idx;
       int vi=(xi[Src]>>Shift)+128; if(vi<0) vi=0; else if(vi>255) vi=255;
       int vq=(xq[Src]>>Shift)+128; if(vq<0) vq=0; else if(vq>255) vq=255;
       Ring[2*Idx]  =(uint8_t)vi;
       Ring[2*Idx+1]=(uint8_t)vq; }

     RingFull+=Bytes;
     Buffers++;
     BytesRead+=Bytes;
     pthread_cond_signal(&RingCond);
     pthread_mutex_unlock(&RingLock);

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
   { (void)tuner;                                // single-tuner mode: only tuner A is in use
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
   // the nearest, and lower rates are obtained with the API's on-chip decimation.
   // -----------------------------------------------------------------------------------------------

   static double SnapSampleRate(double Wanted, int &Decimation)
   { static const double Rate[3]={2000000.0, 6000000.0, 8000000.0};
     Decimation=1;
     // Below 2 MHz: run the ADC at 2 MHz and decimate by a power of two.
     if(Wanted<Rate[0]*1.5)
     { Decimation=1;
       while( (Decimation<32) && (Wanted<=Rate[0]/(2.0*Decimation)) ) Decimation*=2;
       if(Wanted>Rate[0]) return Rate[0];
       return Rate[0]/(double)Decimation; }
     double Best=Rate[0]; double BestErr=fabs(Wanted-Rate[0]);
     for(int Idx=1; Idx<3; Idx++)
       if(fabs(Wanted-Rate[Idx])<BestErr) { Best=Rate[Idx]; BestErr=fabs(Wanted-Rate[Idx]); }
     return Best; }

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

     // Tuner A, single-tuner mode. This is the mode that needs no second device and no licence.
     Device.tuner       = sdrplay_api_Tuner_A;
     Device.rspDuoMode  = (sdrplay_api_RspDuoModeT)RspDuoMode;
     Device.rspDuoSampleFreq = 0.0;            // only meaningful in Dual_Tuner / Master mode

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
     double Rate=SnapSampleRate((double)SampleRate, Decimation);
     if((uint32_t)fabs(Rate-SampleRate)>1)
       printf("RSPduo::Open() => %d Hz requested, the RSPduo gives %8.1f Hz%s\n",
              SampleRate, Rate, Decimation>1 ? " (with on-chip decimation)" : "");

     Params->devParams->fsFreq.fsHz    = Rate;
     Params->devParams->ppm            = (double)FreqCorr;
     Params->devParams->mode           = sdrplay_api_BULK;   // bulk is the robust choice on USB

     if(Params->rxChannelA)
     { sdrplay_api_TunerParamsT   &Tuner=Params->rxChannelA->tunerParams;
       sdrplay_api_ControlParamsT &Ctrl =Params->rxChannelA->ctrlParams;

       Tuner.rfFreq.rfHz      = (double)Frequency;
       Tuner.rfFreq.syncUpdate= 1;
       // 1.536 MHz is the widest filter below a 2 MSps stream: it covers both European OGN
       // channels (868.200 and 868.400) around a 868.300 centre with room to spare.
       Tuner.bwType           = sdrplay_api_BW_1_536;
       Tuner.ifType           = sdrplay_api_IF_Zero;
       Tuner.loMode           = sdrplay_api_LO_Auto;
       Tuner.gain.gRdB        = GRdB_set;
       Tuner.gain.LNAstate    = (unsigned char)LNAstate;
       Tuner.gain.minGr       = sdrplay_api_NORMAL_MIN_GR;

       // DC offset / IQ imbalance correction. Left at the API default, but made explicit so the
       // behaviour is visible: the correction notches a few kHz at the exact centre frequency
       // (868.300), which sits BETWEEN the two OGN channels and therefore does no harm.
       Ctrl.dcOffset.DCenable = DcOffsetCorrect ? 1 : 0;
       Ctrl.dcOffset.IQenable = DcOffsetCorrect ? 1 : 0;
       Ctrl.decimation.enable = (Decimation>1) ? 1 : 0;
       Ctrl.decimation.decimationFactor = (unsigned char)(Decimation>1 ? Decimation : 1);
       Ctrl.agc.enable        = sdrplay_api_AGC_DISABLE;   // manual gain: ogn-rf owns the gain

       SampleRateApplied = Rate/(double)(Decimation>1 ? Decimation : 1);
     }
     else
     { printf("RSPduo::Open() => the API returned no channel A parameters\n"); Close(); return -1; }

     SamplePeriod=1.0/SampleRateApplied;
     FreqApplied=(double)Frequency;
     FreqRaster=0;

     // --- ring buffer --------------------------------------------------------------------------
     RingBytes=(size_t)1<<21;                  // 2 MiB = 1 M samples = 0.5 s at 2 MSps
     Ring=(uint8_t *)malloc(RingBytes);
     if(Ring==0) { printf("RSPduo::Open() => cannot allocate the sample ring\n"); Close(); return -1; }
     RingHead=0; RingFull=0; Stopped=0; Overflows=0; Buffers=0; BytesRead=0;

     // --- start the stream ---------------------------------------------------------------------
     sdrplay_api_CallbackFnsT CbFns;
     CbFns.StreamACbFn=StaticStreamACb;
     CbFns.StreamBCbFn=0;                      // tuner B is not used in single-tuner mode
     CbFns.EventCbFn  =StaticEventCb;

     StartTime=SampleTime=PrevTime=getTime();

     Err=sdrplay_api_Init(Device.dev, &CbFns, this);
     if(Err!=sdrplay_api_Success)
     { printf("RSPduo::Open() => Init() failed: %s\n", sdrplay_api_GetErrorString(Err));
       Close(); return -1; }
     Streaming=1;

     BuildGainTable();

     printf("RSPduo::Open() => %8.3f MHz, %8.3f Msps, gain reduction %4.1f dB, LNA state %d\n",
            1e-6*FreqApplied, 1e-6*SampleRateApplied, (double)GRdB_set, LNAstate);
     return 1; }

   void Close(void)
   { if(Streaming && Device.dev)
     { pthread_mutex_lock(&RingLock); Stopped=1; pthread_cond_broadcast(&RingCond); pthread_mutex_unlock(&RingLock);
       sdrplay_api_Uninit(Device.dev); }
     Streaming=0; Params=0;

     if(Device.dev)
     { sdrplay_api_ReleaseDevice(&Device); memset(&Device, 0, sizeof(Device)); }

     if(ApiOpen) { sdrplay_api_Close(); ApiOpen=0; }

     if(Ring) { free(Ring); Ring=0; RingBytes=0; }
     RingHead=0; RingFull=0; Stopped=0; Gains=0; }

   // -----------------------------------------------------------------------------------------------
   // Gain
   //
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
     Params->rxChannelA->tunerParams.rfFreq.rfHz=(double)Frequency;
     Params->rxChannelA->tunerParams.rfFreq.syncUpdate=1;
     if(Streaming)
     { sdrplay_api_ErrT Err=sdrplay_api_Update(Device.dev, sdrplay_api_Tuner_A,
                                               sdrplay_api_Update_Tuner_Frf, sdrplay_api_Update_Ext1_None);
       if(Err!=sdrplay_api_Success)
       { printf("RSPduo::setCenterFreq(%d) => Update() failed: %s\n", Frequency, sdrplay_api_GetErrorString(Err));
         return -1; } }
     return 0; }

   uint32_t getCenterFreq(void) { return (uint32_t)FreqApplied; }

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

   int setBiasTee(int On=1) { BiasTee=On; return 0; }   // the RSPduo has no bias tee
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

   int ResetBuffer(void)                            // flush anything already queued
   { pthread_mutex_lock(&RingLock);
     RingHead=0; RingFull=0;
     pthread_mutex_unlock(&RingLock);
     return 0; }

   // Blocks until Samples are available, copying interleaved 8-bit unsigned I/Q out. Returns the
   // number of samples delivered, or -1 when the stream has stopped.
   int Read(uint8_t *Buffer, int Samples)
   { if(!Streaming) return -1;
     Samples&=0xFFFFFF00;                          // multiple of 256, as in rtlsdr.h
     if(Samples<=0) return 0;
     size_t Want=2*(size_t)Samples;
     size_t Done=0;

     pthread_mutex_lock(&RingLock);
     while(Done<Want)
     { if(Stopped) { pthread_mutex_unlock(&RingLock); return -1; }
       if(RingFull==0)
       { pthread_cond_wait(&RingCond, &RingLock);
         continue; }
       size_t Take=Want-Done; if(Take>RingFull) Take=RingFull;
       size_t First=Take;
       if(First>RingBytes-RingHead) First=RingBytes-RingHead;
       memcpy(Buffer+Done, Ring+RingHead, First);
       if(Take>First) memcpy(Buffer+Done+First, Ring, Take-First);
       RingHead=(RingHead+Take)%RingBytes;
       RingFull-=Take;
       Done+=Take; }
     pthread_mutex_unlock(&RingLock);

     return (int)(Done/2); }

   int Read(SampleBuffer<uint8_t> &Buffer, int Samples)
   { if(Buffer.Allocate(2,Samples)<=0) return 0;
     int ReadSamples=Read(Buffer.Data, Samples);
     double Time=getTime();
     if(ReadSamples>0)
     { Buffer.Full=ReadSamples*2;
       Buffer.Rate=getSampleRate();
       Buffer.Freq=getCenterFreq();
       Buffer.Time=Time-(double)ReadSamples/Buffer.Rate; }
     return ReadSamples; }

   // ogn-rf reads synchronously, so acquisition is already running once Open() returns.
   int CancelAsync(void)
   { pthread_mutex_lock(&RingLock); Stopped=1; pthread_cond_broadcast(&RingCond); pthread_mutex_unlock(&RingLock);
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
