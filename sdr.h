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
// Radio selection layer.
//
// ogn-rf grew up around one radio, the RTL-SDR dongle, and the RF acquisition class talked to it
// through the RTLSDR class in rtlsdr.h. This file keeps that interface but puts a switch in front
// of it, so the same acquisition code can drive a DVB-T dongle, a LimeSDR or an SDRplay RSPduo.
//
// The implementations are kept as plain members rather than through virtual inheritance: the
// classes are cheap, only one is ever open, and this way every call site in ogn-rf.cc keeps saying
// "SDR.something(...)" with no pointer indirection or allocation to reason about.
//
// Each optional backend is compiled out when its library is missing, and RTLSDR is used as an
// inert placeholder for it, so the branch still builds on a host that only has librtlsdr.
//
// ==================================================================================================

#ifndef __SDR_H__
#define __SDR_H__

#include <strings.h>               // strcasecmp(), for RF.DeviceType parsing

#include "rtlsdr.h"

#ifdef USE_LIMESDR
#include "limesdr.h"
#else
typedef RTLSDR LimeSDR;            // inert placeholder, never selected: see HaveLimeSDR()
#endif

#ifdef USE_RSPDUO
#include "rspduo.h"
#else
typedef RTLSDR RSPDuo;             // inert placeholder, never selected: see HaveRSPDuo()
#endif

class SDR_Device
{ public:
   enum { RTL_SDR=0, LIME_SDR=1, RSP_DUO=2 };

   int           Type;             // which backend is in use
   uint32_t      DeviceIndex;      // index of the device, as reported on the status page
   uint32_t      FreqRaster;       // [Hz] pushed down to the active backend before tuning

   RTLSDR        RTL;
   LimeSDR       Lms;
   RSPDuo        Rsp;

  public:
   SDR_Device() { Type=RTL_SDR; DeviceIndex=0; FreqRaster=0; }
  ~SDR_Device() { Close(); }

   static bool HaveLimeSDR(void)
   {
#ifdef USE_LIMESDR
     return true;
#else
     return false;
#endif
   }

   static bool HaveRSPDuo(void)
   {
#ifdef USE_RSPDUO
     return true;
#else
     return false;
#endif
   }

   static const char *getTypeName(int Type=RTL_SDR)
   { if(Type==LIME_SDR) return "LimeSDR";
     if(Type==RSP_DUO)  return "RSPduo";
     return "RTL-SDR"; }

   static int getTypeByString(const char *Name)     // parse RF.DeviceType
   { if(Name==0) return RTL_SDR;
     if(strcasecmp(Name,"limesdr")==0) return LIME_SDR;
     if(strcasecmp(Name,"lime")==0)    return LIME_SDR;
     if(strcasecmp(Name,"soapy")==0)   return LIME_SDR;
     if(strcasecmp(Name,"rspduo")==0)  return RSP_DUO;
     if(strcasecmp(Name,"rsp-duo")==0) return RSP_DUO;
     if(strcasecmp(Name,"sdrplay")==0) return RSP_DUO;
     if(strcasecmp(Name,"duo")==0)     return RSP_DUO;
     return RTL_SDR; }

   int setType(int NewType)
   { if(isOpen()) return -1;                         // must not change a running radio
     if( (NewType==LIME_SDR) && !HaveLimeSDR() )     // asked for LimeSDR but it was not compiled in
     { printf("SDR: LimeSDR support was not compiled in (build with USE_LIMESDR=1)\n"); NewType=RTL_SDR; }
     if( (NewType==RSP_DUO) && !HaveRSPDuo() )       // asked for RSPduo but it was not compiled in
     { printf("SDR: RSPduo support was not compiled in (build with USE_RSPDUO=1)\n"); NewType=RTL_SDR; }
     Type=NewType; return Type; }

   int getType(void) { return Type; }

   // -----------------------------------------------------------------------------------------------
   // Life cycle
   // -----------------------------------------------------------------------------------------------

   bool isOpen(void)
   { if(Type==LIME_SDR) return Lms.isOpen();
     if(Type==RSP_DUO)  return Rsp.isOpen();
     return RTL.isOpen(); }

   void Close(void)
   { if(Type==LIME_SDR) Lms.Close();
     else if(Type==RSP_DUO) Rsp.Close();
     else RTL.Close(); }

   double getTime(void)
   { if(Type==LIME_SDR) return Lms.getTime();
     if(Type==RSP_DUO)  return Rsp.getTime();
     return RTL.getTime(); }

   int Open(uint32_t DeviceIndex=0, uint32_t Frequency=868000000, uint32_t SampleRate=2048000)
   { Close();
     this->DeviceIndex=DeviceIndex;
     if(Type==LIME_SDR) { Lms.FreqRaster=FreqRaster; return Lms.Open(DeviceIndex, Frequency, SampleRate); }
     if(Type==RSP_DUO)  { Rsp.FreqRaster=FreqRaster; return Rsp.Open(DeviceIndex, Frequency, SampleRate); }
     RTL.FreqRaster=FreqRaster;
     return RTL.Open(DeviceIndex, Frequency, SampleRate); }

   int getDeviceIndexBySerial(const char *Serial)
   { if(Type==LIME_SDR) return Lms.getDeviceIndexBySerial(Serial);
     if(Type==RSP_DUO)  return Rsp.getDeviceIndexBySerial(Serial);
     return RTL.getDeviceIndexBySerial(Serial); }

   // -----------------------------------------------------------------------------------------------
   // Streaming
   // -----------------------------------------------------------------------------------------------

   int ResetBuffer(void)
   { if(Type==LIME_SDR) return Lms.ResetBuffer();
     if(Type==RSP_DUO)  return Rsp.ResetBuffer();
     return RTL.ResetBuffer(); }

   int Read(uint8_t *Buffer, int Samples)
   { if(Type==LIME_SDR) return Lms.Read(Buffer,Samples);
     if(Type==RSP_DUO)  return Rsp.Read(Buffer,Samples);
     return RTL.Read(Buffer,Samples); }

   int Read(SampleBuffer<uint8_t> &Buffer, int Samples)
   { if(Type==LIME_SDR) return Lms.Read(Buffer,Samples);
     if(Type==RSP_DUO)  return Rsp.Read(Buffer,Samples);
     return RTL.Read(Buffer,Samples); }

   // -----------------------------------------------------------------------------------------------
   // Frequency
   // -----------------------------------------------------------------------------------------------

   int setCenterFreq(uint32_t Frequency)
   { if(Type==LIME_SDR) { Lms.FreqRaster=FreqRaster; return Lms.setCenterFreq(Frequency); }
     if(Type==RSP_DUO)  { Rsp.FreqRaster=FreqRaster; return Rsp.setCenterFreq(Frequency); }
     RTL.FreqRaster=FreqRaster; return RTL.setCenterFreq(Frequency); }

   uint32_t getCenterFreq(void)
   { if(Type==LIME_SDR) return Lms.getCenterFreq();
     if(Type==RSP_DUO)  return Rsp.getCenterFreq();
     return RTL.getCenterFreq(); }

   int setFreqCorrection(int PPM)
   { if(Type==LIME_SDR) return Lms.setFreqCorrection(PPM);
     if(Type==RSP_DUO)  return Rsp.setFreqCorrection(PPM);
     return RTL.setFreqCorrection(PPM); }

   int getFreqCorrection(void)
   { if(Type==LIME_SDR) return Lms.getFreqCorrection();
     if(Type==RSP_DUO)  return Rsp.getFreqCorrection();
     return RTL.getFreqCorrection(); }

   // -----------------------------------------------------------------------------------------------
   // Sample rate
   // -----------------------------------------------------------------------------------------------

   int setSampleRate(uint32_t SampleRate)
   { if(Type==LIME_SDR) return Lms.setSampleRate(SampleRate);
     if(Type==RSP_DUO)  return Rsp.setSampleRate(SampleRate);
     return RTL.setSampleRate(SampleRate); }

   uint32_t getSampleRate(void)
   { if(Type==LIME_SDR) return Lms.getSampleRate();
     if(Type==RSP_DUO)  return Rsp.getSampleRate();
     return RTL.getSampleRate(); }

   // LimeSDR only: on-chip capture-and-decimate ratio. The RSPduo has decimation of its own but
   // it is driven from the sample rate, not from this knob, and an RTL-SDR has no equivalent.
   void setOversample(int Ratio)
   {
#ifdef USE_LIMESDR
     if(Type==LIME_SDR) Lms.Oversample=Ratio;
#else
     (void)Ratio;
#endif
   }

   // LimeSDR only: reference clock selection. Ignored by the other two radios.
   void setClockSource(const char *Name)
   {
#ifdef USE_LIMESDR
     if(Type==LIME_SDR)
     { if(Name && Name[0]) { strncpy(Lms.ClockSource, Name, 16); Lms.ClockSource[15]=0; }
       else Lms.ClockSource[0]=0; }
#else
     (void)Name;
#endif
   }

   void setReferenceRate(double Hz)
   {
#ifdef USE_LIMESDR
     if(Type==LIME_SDR) Lms.ReferenceRate = Hz>0 ? Hz : 0;
#else
     (void)Hz;
#endif
   }

   // What the device reported after we asked, for the status page.
   const char *getClockSourceApplied(void)
   {
#ifdef USE_LIMESDR
     if(Type==LIME_SDR) return Lms.ClockSourceApplied;
#endif
     return "";
   }

   // What the device actually accepted, for the status page: 0 means the driver chose.
   int getOversampleApplied(void)
   {
#ifdef USE_LIMESDR
     if(Type==LIME_SDR) return Lms.OversampleApplied;
#endif
     return 0;
   }

   // -----------------------------------------------------------------------------------------------
   // Gain
   // -----------------------------------------------------------------------------------------------

   int setTunerGainMode(int Manual)
   { if(Type==LIME_SDR) return Lms.setTunerGainMode(Manual);
     if(Type==RSP_DUO)  return Rsp.setTunerGainMode(Manual);
     return RTL.setTunerGainMode(Manual); }

   int setTunerGain(int Gain)
   { if(Type==LIME_SDR) return Lms.setTunerGain(Gain);
     if(Type==RSP_DUO)  return Rsp.setTunerGain(Gain);
     return RTL.setTunerGain(Gain); }

   int getTunerGain(void)
   { if(Type==LIME_SDR) return Lms.getTunerGain();
     if(Type==RSP_DUO)  return Rsp.getTunerGain();
     return RTL.getTunerGain(); }

   int getTunerGains(int *Gain=0)
   { if(Type==LIME_SDR) return Lms.getTunerGains(Gain);
     if(Type==RSP_DUO)  return Rsp.getTunerGains(Gain);
     return RTL.getTunerGains(Gain); }

   int setTunerGainManual(int Manual=1) { return setTunerGainMode(Manual); }
   int setTunerGainAuto(void)           { return setTunerGainMode(0); }

   // -----------------------------------------------------------------------------------------------
   // Options that only some radios have; accepted (and ignored) by the others
   // -----------------------------------------------------------------------------------------------

   int setOffsetTuning(int ON=1)
   { if(Type==LIME_SDR) return Lms.setOffsetTuning(ON);
     if(Type==RSP_DUO)  return Rsp.setOffsetTuning(ON);
     return RTL.setOffsetTuning(ON); }

   int setBiasTee(int On=1)
   { if(Type==LIME_SDR) return Lms.setBiasTee(On);
     if(Type==RSP_DUO)  return Rsp.setBiasTee(On);
     return RTL.setBiasTee(On); }

   // RSPduo only: Dual!=0 runs both tuners at once, 0 is single-tuner (the default). Takes effect
   // at the next Open(), because the mode is fixed when the device is selected.
   //
   // A plain flag rather than the vendor's sdrplay_api_RspDuoModeT: that type only exists when
   // USE_RSPDUO is defined, and ogn-rf.cc must still compile without it.
   void setRspDuoDualTuner(int Dual)
   {
#ifdef USE_RSPDUO
     if(Type==RSP_DUO) Rsp.setRspDuoMode(Dual ? sdrplay_api_RspDuoMode_Dual_Tuner
                                              : sdrplay_api_RspDuoMode_Single_Tuner);
#else
     (void)Dual;
#endif
   }

   // RSPduo only: selection diversity between the two tuners (both on one frequency, the
   // stronger one used per read). Ignored by every other radio.
   void setRspDuoSelectionDiversity(int On)
   {
#ifdef USE_RSPDUO
     if(Type==RSP_DUO) Rsp.setSelectionDiversity(On);
#else
     (void)On;
#endif
   }

   bool haveRspDuoDiversity(void)
   {
#ifdef USE_RSPDUO
     if(Type==RSP_DUO) return Rsp.haveDiversity();
#endif
     return false;
   }

   uint64_t getRspDuoDivChoseA(void)
   {
#ifdef USE_RSPDUO
     if(Type==RSP_DUO) return Rsp.getDivChoseA();
#endif
     return 0;
   }

   uint64_t getRspDuoDivChoseB(void)
   {
#ifdef USE_RSPDUO
     if(Type==RSP_DUO) return Rsp.getDivChoseB();
#endif
     return 0;
   }

   // Ring overruns: the measure of whether the backend kept up with the device. Any non-zero
   // value means samples were discarded because a reader fell behind, which is data loss the
   // Live Time figure cannot distinguish from ogn-rf simply not asking.
   uint64_t getRspDuoOverflows(int Tuner)
   {
#ifdef USE_RSPDUO
     if(Type==RSP_DUO) return Rsp.getOverflows(Tuner);
#else
     (void)Tuner;
#endif
     return 0;
   }

   uint64_t getRspDuoBytesRead(int Tuner)
   {
#ifdef USE_RSPDUO
     if(Type==RSP_DUO) return Rsp.getBytesRead(Tuner);
#else
     (void)Tuner;
#endif
     return 0;
   }

   uint64_t getRspDuoDivSwitches(void)
   {
#ifdef USE_RSPDUO
     if(Type==RSP_DUO) return Rsp.getDivSwitches();
#endif
     return 0;
   }

   // RSPduo only: is the second tuner live, and where is it tuned?
   bool haveTunerB(void)
   {
#ifdef USE_RSPDUO
     if(Type==RSP_DUO) return Rsp.haveTunerB();
#endif
     return false;
   }

   uint32_t getCenterFreqB(void)
   {
#ifdef USE_RSPDUO
     if(Type==RSP_DUO) return Rsp.getCenterFreqB();
#endif
     return 0;
   }

   // -----------------------------------------------------------------------------------------------
   // Identification, for the status page and the log
   // -----------------------------------------------------------------------------------------------

   const char *getDeviceName(void)
   { if(Type==LIME_SDR) return Lms.getDeviceName();
     if(Type==RSP_DUO)  return Rsp.getDeviceName();
     return RTL.getDeviceName(); }

   const char *getTunerTypeName(void)
   { if(Type==LIME_SDR) return Lms.getTunerTypeName();
     if(Type==RSP_DUO)  return Rsp.getTunerTypeName();
     return RTL.getTunerTypeName(); }

   int getTunerType(void)
   { if(Type==LIME_SDR) return Lms.getTunerType();
     if(Type==RSP_DUO)  return Rsp.getTunerType();
     return RTL.getTunerType(); }

   int getUsbStrings(char *Manufacturer, char *Product, char *Serial)
   { if(Type==LIME_SDR) return Lms.getUsbStrings(Manufacturer,Product,Serial);
     if(Type==RSP_DUO)  return Rsp.getUsbStrings(Manufacturer,Product,Serial);
     return RTL.getUsbStrings(Manufacturer,Product,Serial); }

   int getXtalFreq(uint32_t &RtlFreq, uint32_t &TunerFreq)
   { if(Type==LIME_SDR) return Lms.getXtalFreq(RtlFreq,TunerFreq);
     if(Type==RSP_DUO)  return Rsp.getXtalFreq(RtlFreq,TunerFreq);
     return RTL.getXtalFreq(RtlFreq,TunerFreq); }

#ifdef NEW_RTLSDR_LIB
   int getTunerStageGains(int Stage, int32_t *Gain, char *Description=0)
   { if(Type==LIME_SDR) return Lms.getTunerStageGains(Stage,Gain,Description);
     return RTL.getTunerStageGains(Stage,Gain,Description); }

   int getTunerBandwidths(int *Bandwidth=0)
   { if(Type==LIME_SDR) return Lms.getTunerBandwidths(Bandwidth);
     if(Type==RSP_DUO)  return 0;
     return RTL.getTunerBandwidths(Bandwidth); }
#endif

} ;

#endif // __SDR_H__
