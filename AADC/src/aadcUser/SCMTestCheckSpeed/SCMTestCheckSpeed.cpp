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
#include "SCMTestCheckSpeed.h"
#include "ScmCommunication.h"

// This will define the filter and expose it via plugin class factory.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_SCM_TEST_CHECK_SPEED_FILTER,                         // references to header file
        "SCMTestCheckSpeed",                                   // label
        SCMTestCheckSpeed,                                      // class
        adtf::filter::pin_trigger({"action_in"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
SCMTestCheckSpeed::SCMTestCheckSpeed()
{

    // ------------------------------------------
    SetName("SCMTestCheckSpeed Constructor");

    // -----------------------------------------
    // set pins
    o_TSignalValue.registerPin(this, m_ReaderSpeed          , "speed_in"        );
    o_TActionStruct.registerPin(this, m_ReaderAction        , "action_in"       );

    o_TSignalValue.registerPin(this, m_WriterCarSpeed       , "car_speed_out"   );
    o_TFeedbackStruct.registerPin(this, m_WriterFeedback    , "feedback_out"    );

    // -----------------------------------------
    // set property variables
    RegisterPropertyVariable("Speed limited to ", m_propF32LimitSpeedControllerOutput       );

}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult SCMTestCheckSpeed::Configure()
{
    m_ui32LastTimeStampSpeed = 0;
    m_ui32LastTimeStampAction = 0;
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    LOG_INFO(cString::Format("Configuration finished!"));
    // done
    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
tResult SCMTestCheckSpeed::Process(tTimeStamp tmTimeOfTrigger)
{

    // if new data is action signal
    if(IS_OK(o_TActionStruct.readPin(m_ReaderAction, (void *) & m_dataActionIn, m_ui32LastTimeStampAction)))
    {
        if (m_dataActionIn.ui8FilterId == 1)
        {

            m_ui32LastTimeStampAction = m_dataActionIn.ui32ArduinoTimestamp;

        // process data
        RETURN_IF_FAILED(ProcessActionInput());
        }
    }

    // done
    RETURN_NOERROR;
}
// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

tResult SCMTestCheckSpeed::ProcessActionInput()
{
    boost::lock_guard<boost::mutex> lock(criticalSectionTransmit);

    {
        if (m_dataActionIn.ui32Command == 1010){
                m_dataFeedbackOut.ui8FilterId = 1;
                m_dataFeedbackOut.ui32FeedbackStatus = 1011;

                RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &m_dataFeedbackOut, m_pClock->GetStreamTime()));


        }else{
            LOG_INFO("right filter but command not found!");
        }
    }



    RETURN_NOERROR;

}

