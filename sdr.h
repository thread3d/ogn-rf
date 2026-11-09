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
// of it, so the same acquisition code can drive either a DVB-T dongle or a LimeSDR.
//
// The two implementations are kept as plain members rather than through virtual inheritance: the
// classes are cheap, only one is ever open, and this way every call site in ogn-rf.cc keeps saying
// "SDR.something(...)" with no pointer indirection or allocation to reason about.
//
// When SoapySDR is not present at build time, the LimeSDR backend is compiled out and RTLSDR is
// used as an inert placeholder, so the branch still builds on a host that only has librtlsdr.
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

class SDR_Device
{ public:
   enum { RTL_SDR=0, LIME_SDR=1 };

   int           Type;             // which backend is in use
   uint32_t      DeviceIndex;      // index of the device, as reported on the status page
   uint32_t      FreqRaster;       // [Hz] pushed down to the active backend before tuning

   RTLSDR        RTL;
   LimeSDR       Lms;

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

   static const char *getTypeName(int Type=RTL_SDR) { return (Type==LIME_SDR) ? "LimeSDR" : "RTL-SDR"; }

   static int getTypeByString(const char *Name)     // parse RF.DeviceType
   { if(Name==0) return RTL_SDR;
     if(strcasecmp(Name,"limesdr")==0) return LIME_SDR;
     if(strcasecmp(Name,"lime")==0)    return LIME_SDR;
     if(strcasecmp(Name,"soapy")==0)   return LIME_SDR;
     return RTL_SDR; }

   int setType(int NewType)
   { if(isOpen()) return -1;                         // must not change a running radio
     if( (NewType==LIME_SDR) && !HaveLimeSDR() )     // asked for LimeSDR but it was not compiled in
     { printf("SDR: LimeSDR support was not compiled in (build with USE_LIMESDR=1)\n"); NewType=RTL_SDR; }
     Type=NewType; return Type; }

   int getType(void) { return Type; }

   // -----------------------------------------------------------------------------------------------
   // Life cycle
   // -----------------------------------------------------------------------------------------------

   bool isOpen(void)         { return (Type==LIME_SDR) ? Lms.isOpen()       : RTL.isOpen(); }
   void Close(void)          { if(Type==LIME_SDR) Lms.Close();              else RTL.Close(); }
   double getTime(void)      { return (Type==LIME_SDR) ? Lms.getTime()      : RTL.getTime(); }

   int Open(uint32_t DeviceIndex=0, uint32_t Frequency=868000000, uint32_t SampleRate=2048000)
   { Close();
     this->DeviceIndex=DeviceIndex;
     if(Type==LIME_SDR) { Lms.FreqRaster=FreqRaster; return Lms.Open(DeviceIndex, Frequency, SampleRate); }
     RTL.FreqRaster=FreqRaster;
     return RTL.Open(DeviceIndex, Frequency, SampleRate); }

   int getDeviceIndexBySerial(const char *Serial)
   { return (Type==LIME_SDR) ? Lms.getDeviceIndexBySerial(Serial) : RTL.getDeviceIndexBySerial(Serial); }

   // -----------------------------------------------------------------------------------------------
   // Streaming
   // -----------------------------------------------------------------------------------------------

   int ResetBuffer(void)                       { return (Type==LIME_SDR) ? Lms.ResetBuffer() : RTL.ResetBuffer(); }
   int Read(uint8_t *Buffer, int Samples)      { return (Type==LIME_SDR) ? Lms.Read(Buffer,Samples) : RTL.Read(Buffer,Samples); }
   int Read(SampleBuffer<uint8_t> &Buffer, int Samples)
   { return (Type==LIME_SDR) ? Lms.Read(Buffer,Samples) : RTL.Read(Buffer,Samples); }

   // -----------------------------------------------------------------------------------------------
   // Frequency
   // -----------------------------------------------------------------------------------------------

   int setCenterFreq(uint32_t Frequency)
   { if(Type==LIME_SDR) { Lms.FreqRaster=FreqRaster; return Lms.setCenterFreq(Frequency); }
     RTL.FreqRaster=FreqRaster; return RTL.setCenterFreq(Frequency); }

   uint32_t getCenterFreq(void)        { return (Type==LIME_SDR) ? Lms.getCenterFreq()      : RTL.getCenterFreq(); }
   int setFreqCorrection(int PPM)      { return (Type==LIME_SDR) ? Lms.setFreqCorrection(PPM) : RTL.setFreqCorrection(PPM); }
   int getFreqCorrection(void)         { return (Type==LIME_SDR) ? Lms.getFreqCorrection()  : RTL.getFreqCorrection(); }

   // -----------------------------------------------------------------------------------------------
   // Sample rate
   // -----------------------------------------------------------------------------------------------

   int setSampleRate(uint32_t SampleRate) { return (Type==LIME_SDR) ? Lms.setSampleRate(SampleRate) : RTL.setSampleRate(SampleRate); }
   uint32_t getSampleRate(void)           { return (Type==LIME_SDR) ? Lms.getSampleRate()          : RTL.getSampleRate(); }

   // -----------------------------------------------------------------------------------------------
   // Gain
   // -----------------------------------------------------------------------------------------------

   int setTunerGainMode(int Manual)  { return (Type==LIME_SDR) ? Lms.setTunerGainMode(Manual) : RTL.setTunerGainMode(Manual); }
   int setTunerGain(int Gain)        { return (Type==LIME_SDR) ? Lms.setTunerGain(Gain)       : RTL.setTunerGain(Gain); }
   int getTunerGain(void)            { return (Type==LIME_SDR) ? Lms.getTunerGain()           : RTL.getTunerGain(); }
   int getTunerGains(int *Gain=0)    { return (Type==LIME_SDR) ? Lms.getTunerGains(Gain)      : RTL.getTunerGains(Gain); }

   // -----------------------------------------------------------------------------------------------
   // RTL-SDR specific options, accepted (and ignored) for LimeSDR
   // -----------------------------------------------------------------------------------------------

   int setOffsetTuning(int ON=1)     { return (Type==LIME_SDR) ? Lms.setOffsetTuning(ON) : RTL.setOffsetTuning(ON); }
   int setBiasTee(int On=1)          { return (Type==LIME_SDR) ? Lms.setBiasTee(On)      : RTL.setBiasTee(On); }

   // -----------------------------------------------------------------------------------------------
   // Identification, for the status page and the log
   // -----------------------------------------------------------------------------------------------

   const char *getDeviceName(void)   { return (Type==LIME_SDR) ? Lms.getDeviceName()      : RTL.getDeviceName(); }
   const char *getTunerTypeName(void){ return (Type==LIME_SDR) ? Lms.getTunerTypeName()   : RTL.getTunerTypeName(); }
   int getTunerType(void)            { return (Type==LIME_SDR) ? Lms.getTunerType()       : RTL.getTunerType(); }

   int getUsbStrings(char *Manufacturer, char *Product, char *Serial)
   { return (Type==LIME_SDR) ? Lms.getUsbStrings(Manufacturer,Product,Serial)
                             : RTL.getUsbStrings(Manufacturer,Product,Serial); }

   int getXtalFreq(uint32_t &RtlFreq, uint32_t &TunerFreq)
   { return (Type==LIME_SDR) ? Lms.getXtalFreq(RtlFreq,TunerFreq) : RTL.getXtalFreq(RtlFreq,TunerFreq); }

#ifdef NEW_RTLSDR_LIB
   int getTunerStageGains(int Stage, int32_t *Gain, char *Description=0)
   { return (Type==LIME_SDR) ? Lms.getTunerStageGains(Stage,Gain,Description)
                             : RTL.getTunerStageGains(Stage,Gain,Description); }

   int getTunerBandwidths(int *Bandwidth=0)
   { return (Type==LIME_SDR) ? Lms.getTunerBandwidths(Bandwidth) : RTL.getTunerBandwidths(Bandwidth); }
#endif

} ;

#endif // __SDR_H__
