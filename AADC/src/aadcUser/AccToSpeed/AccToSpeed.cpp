/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#include <mutex>
#include "stdafx.h"
#include "AccToSpeed.h"
#include "ScmCommunication.h"

// This will define the filter and expose it via plugin class factory.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_ACCTOSPEED_FILTER,                         // references to header file
        "AccToSpeed",                                   // label
        AccToSpeed,                                      // class
        //adtf::filter::pin_trigger({"road_struct_in"}));	// set trigger pin
 adtf::filter::timer_trigger(2));

// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
AccToSpeed::AccToSpeed()
{

    // ------------------------------------------
    SetName("AccToSpeed Constructor");

    // -----------------------------------------
    // set pins
    o_TRoadSignExt.registerPin(this, m_ReaderRoadSign    , "road_struct_in"     );
    o_TSignalValue.registerPin(this, m_ReaderSpeed    , "speed"     );

    o_TSignalValue.registerPin(this, m_WriterCarSpeed   , "car_speed");

    // -----------------------------------------
    // set property variables

}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult AccToSpeed::Configure()
{
    m_ui32LastTimeStampSpeed = 0;
    m_ui32LastTimeStampRoadSign= 0;
    m_dataRoadSign.af32TVec[0] = 0;
    m_dataRoadSign.af32TVec[1] = 0;
    m_dataRoadSign.af32TVec[2] = 0;
    signDetectedFlag= tFalse;
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    LOG_INFO("CONFIG DONE!");
    // done
    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
tResult AccToSpeed::Process(tTimeStamp)
{

    TSignalValue::Data tmp_speed;
    // if new data is speed signal
    if(IS_OK(o_TSignalValue.readPin(m_ReaderSpeed, (void *) & m_dataSpeed, m_ui32LastTimeStampSpeed)))
    {

        m_ui32LastTimeStampSpeed = m_dataSpeed.ui32ArduinoTimestamp;


        // process data
    }

    // if new data is steering signal
    TRoadSignExt::Data tmp_roadsign;
    if(IS_OK(o_TRoadSignExt.readPin(m_ReaderRoadSign, (void *) & tmp_roadsign, m_ui32LastTimeStampRoadSign)))
    {
        m_dataRoadSign = tmp_roadsign;
        m_ui32LastTimeStampRoadSign = m_dataRoadSign.ui32ArduinoTimestamp;
        signDetectedFlag =tTrue;

        // process data


    }
    RETURN_IF_FAILED(ProcessRoadSignInput(m_dataRoadSign));
    // done
    RETURN_NOERROR;
}
// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

tResult AccToSpeed::ProcessRoadSignInput(TRoadSignExt::Data inputSignal)
{
    boost::lock_guard<boost::mutex> lock(criticalSectionTransmit);

    TSignalValue::Data tmp_speed = m_dataSpeed;

    if (inputSignal.af32TVec[2] < 0.50000 && signDetectedFlag == tTrue)
    {
      LOG_ERROR("fucked up point!");
      tmp_speed.f32Value = 0.0f;
      LOG_WARNING("z %f", inputSignal.af32TVec[2]);
    }
    tmp_speed.ui32ArduinoTimestamp = m_pClock->GetStreamTime();
    //LOG_INFO(cString::Format("AccToSpeed: speed is %f", tmpSpeed.f32Value));
    RETURN_IF_FAILED(o_TSignalValue.writePin(m_WriterCarSpeed, (void *) &tmp_speed, m_pClock->GetStreamTime()));
    RETURN_NOERROR;

}
