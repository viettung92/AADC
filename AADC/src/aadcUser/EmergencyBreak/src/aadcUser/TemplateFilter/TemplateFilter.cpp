/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. Team FAUtonomous. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

TODO: write a short description of your filter!!
This filter is our template filter for aadc2018.
Use this filter for creating your own filters.
Keep in mind to change "cTemplateFilter" to your filtername.

**********************************************************************/

#include "stdafx.h"
#include "TemplateFilter.h"

// This will define the filter and expose it via plugin class factory.
// Class cTemplateFilter will be embedded to the filter and called by
// data trigger of "speed" by default.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
    CID_TEMPLATE_FILTER_FILTER,		// references to header file
    "cTemplateFilter",              // label
    cTemplateFilter,                // class
    adtf::filter::pin_trigger({"input"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
cTemplateFilter::cTemplateFilter()
{

    // -----------------------------------------
    // load the streamtypes: happens in the constructor of TSignalValue;
    // register pins
    m_SpeedSignalId.registerPin(this, m_ReaderSpeed, "inputSpeed" );
    m_SpeedSignalId.registerPin(this, m_WriterSpeed, "outputSpeed");


    // -----------------------------------------
    // set property variables
    RegisterPropertyVariable("How cool is Binh?", m_propBinhsCoolnessFactor);
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult cTemplateFilter::Configure()
{
    // done
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
// This is how to read a sample
tResult cTemplateFilter::Process(tTimeStamp tmTimeOfTrigger)
{
    // read from inputPin
    TSignalValue::Data rec_carspeed; // previous: tSignalValue
    static tTimeStamp lasttm = 0;
    RETURN_IF_FAILED(m_SpeedSignalId.readPin(m_ReaderSpeed, (void *) &rec_carspeed, lasttm));
    lasttm = rec_carspeed.ui32ArduinoTimestamp; //do this for every pin
    RETURN_IF_FAILED(TransmitSpeed(rec_carspeed));

    // done
    RETURN_NOERROR;
}


// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

// TransmitSpeed
// This is how you write a signal output!!
tResult cTemplateFilter::TransmitSpeed(TSignalValue::Data inputSignal)
{
    // set new timestamp ( stream time 2 x submitted, muss nochmal angepasst werden )
    inputSignal.ui32ArduinoTimestamp = m_pClock->GetStreamTime();
    // write to outputPin
    RETURN_IF_FAILED(m_SpeedSignalId.writePin(m_WriterSpeed, (void *) &inputSignal, m_pClock->GetStreamTime()));

    // done
    RETURN_NOERROR;
}
