/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

This filter is used for braking.
It takes speed and an tActionCommand as pin input and will brake with brakingSpeed = (-1) * (currentSpeed * m_propPercentageActiveBraking).
Braking will only be triggerm_f32CarStoppedThresholded if actionCommand == AC_SA_STOP_CAR.

Ported by Pham, improved (use classes to shorten and simplify) and tested (not done yet) by Illmer Status:doing (still untested and not apdapted to the new ActionCommand

**********************************************************************/

#include <mutex>
#include "stdafx.h"
#include "ActionStop.h"
#include "ScmCommunication.h"
#include <ADTF3_helper.h>

// This will define the filter and expose it via plugin class factory.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_ACTION_STOP_FILTER,                         // references to header file
        "cActionStop",                                   // label
        ActionStop,                                     // class
        adtf::filter::pin_trigger({"speedInput"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
ActionStop::ActionStop()
{
    // ------------------------------------------
    // initialize variables
    // set threshold
    m_f32CarStoppedThreshold = tFloat(0.5f);


    // -----------------------------------------
    // set pins
    // input
    m_SpeedSignal.registerPin(this,      m_ReaderSpeed,     "speedInput");
    m_ActionCommand.registerPin(this,    m_ReaderAction,    "actionInput");

    // output
    //Register(m_WriterSpeed   , "speedOutput"   , pSpeedDataType   );
    //Register(m_WriterFeedback, "feedbackOutput", pFeedbackDataType);
    m_SpeedSignal.registerPin(this,     m_WriterSpeed,      "speedOutput");
    m_FeedbackStruct.registerPin(this,  m_WriterFeedback,   "feedbackOutput");

    // -----------------------------------------
    // set property variables
    RegisterPropertyVariable("ActionStopCommand activated", m_propCommandActivated       );
    RegisterPropertyVariable("Percentage for braking [%]" , m_propPercentageActiveBraking);

}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult ActionStop::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    // done
    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
tResult ActionStop::Process(tTimeStamp tmTimeOfTrigger)
{
    // -----------------------------------------
    // create pointer for samples
    // object_ptr<const ISample> pReadSpeedSample;
    // object_ptr<const ISample> pReadActionSample;
    //object_ptr<const ISample> pReadTempSample;


    // -----------------------------------------
    // get the last speed sample
    //if (IS_OK(m_ReaderSpeed.GetLastSample(pReadTempSample)))
    //{//hier
        TSignalValue::Data speedSignal;
        static tTimeStamp lasttmspeed = 0;
        //TSignalValue::Data outputSpeedSignal;
        if(IS_OK(m_SpeedSignal.readPin(m_ReaderSpeed, (void *) &speedSignal, lasttmspeed)))
        {
            // process data
            //RETURN_IF_FAILED(TransmitSpeed(inputSpeedSignal));
            //}
            //}
            /*
    // get the last speed sample
    if(IS_OK(m_ReaderSpeed.GetLastSample(pReadSpeedSample)))
    {
        tFloat32 inputSpeed     = 0.f;
        tUInt32  inputTimestamp = 0;

        // decode last sample
        auto oDecoder = m_SpeedSampleFactory.MakeDecoderFor(*pReadSpeedSample);
        RETURN_IF_FAILED(oDecoder.IsValid());

        // get values
        RETURN_IF_FAILED(oDecoder.GetElementValue(o_SpeedSignalId.f32Value     , &inputSpeed));
        RETURN_IF_FAILED(oDecoder.GetElementValue(o_SpeedSignalId.ui32Timestamp, &inputTimestamp));

        tSignalValue speedSignal;
        speedSignal.f32Value             = inputSpeed;
        speedSignal.ui32ArduinoTimestamp = inputTimestamp;
*/
            // if ActionStop command was activated
            lasttmspeed = inputSpeedSignal.ui32ArduinoTimestamp;
            if(m_propCommandActivated)
            {
                // brake!
                speedSignal.f32Value = - (speedSignal.f32Value * m_propPercentageActiveBraking);

                // if the car has finally stopped
                if(speedSignal.f32Value <= m_f32CarStoppedThreshold)
                {
                    // set flag
                    //m_mutexCommandActivatedFlag.lock();
                    m_propCommandActivated = tFalse;
                    //m_mutexCommandActivatedFlag.unlock();

                    // send feedback, that car has stopped (FB_SA_STOPPED)
                    //TransmitFeedback(TFeedbackStruct::Data inputFeedback)
                    TFeedbackStruct::Data feedbackStop;
                    feedbackStop.ui8FilterId = F_STOP_ACTION;
                    feedbackStop.ui32FeedbackStatus = FB_SA_STOPPED;
                    RETURN_IF_FAILED(TransmitFeedback(feedbackStop));
                }

            }
#ifndef NDEBUG
            LOG_INFO(cString::Format("ActionStop: %f / %d", speedSignal.f32Value, inputTimestamp));
#endif
            // transmit speed data
            RETURN_IF_FAILED(TransmitSpeed(speedSignal));
        }
    //}


    // get the last action commmand sample
    //-----------------------------------------
    //--------------!!!!!!!--------------------
    //-----------------------------------------
    //we dont do something with the action command sample
    //if(IS_OK(m_ReaderAction.GetLastSample(pReadTempSample)))
    //{//here
        // decode last sample
        //auto oDecoder = m_ActionCommandSampleFactory.MakeDecoderFor(*pReadActionSample);
        //RETURN_IF_FAILED(oDecoder.IsValid());
        TActionStruct::Data tmp_actionStruct;
        static tTimeStamp lasttmaction = 0;
        RETURN_IF_FAILED(m_ActionCommand.readPin(m_ReaderAction, (void *) &tmp_actionStruct, lasttmaction));//m_ReaderAction
        lasttmaction = tmp_actionStruct.ui32ArduinoTimestamp;
        // get values
        //      tActionCommand actionCommand;
        //     RETURN_IF_FAILED(oDecoder.GetElementValue(o_ActionCommandStructId.command, &actionCommand.ui32Command));
#ifndef NDEBUG
        LOG_INFO(cString::Format("ActionStruct: Id: %d, Enabled: %d, Started: %d, Command:  %d /n", tmp_actionStruct.ui8FilterId, tmp_actionStruct.bEnabled, tmp_actionStruct.bStarted, tmp_actionStruct.ui32Command));
#endif
        //      LOG_INFO(cString::Format("ActionStop: %d", actionCommand.ui32Command));
        // process data
        //      RETURN_IF_FAILED(ProcessActionCommand(actionCommand));
        // }

        // done
        RETURN_IF_FAILED(ProcessActionCommand(tmp_actionStruct));
        RETURN_NOERROR;
    //}//here
}

// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------
/*
    // TransmitSpeed
    tResult ActionStop::TransmitSpeed(tSignalValue inputSpeed)
    {
        object_ptr<ISample> pWriteSample;
        // allocate for our sample
        RETURN_IF_FAILED(alloc_sample(pWriteSample));

        // create coding for signal
        auto codec = m_SpeedSampleFactory.MakeCodecFor(pWriteSample);

        // set values
        RETURN_IF_FAILED(codec.SetElementValue(o_SpeedSignalId.ui32Timestamp, inputSpeed.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(codec.SetElementValue(o_SpeedSignalId.f32Value     , inputSpeed.f32Value));

        // write to sample
        m_WriterSpeed << pWriteSample << flush << trigger;

        // done
        RETURN_NOERROR;
    }
    */

// TransmitSpeed
tResult ActionStop::TransmitSpeed(TSignalValue::Data inputSpeed)
{
    RETURN_IF_FAILED(m_SpeedSignal.writePin(m_WriterSpeed, (void *) &inputSpeed, m_pClock->GetStreamTime()));

    // done
    RETURN_NOERROR;
}

// TransmitFeedback
tResult ActionStop::TransmitFeedback(TFeedbackStruct::Data inputFeedback)//tSize inputFilterId, tSize inputFeedbackStatus)
{
    /*
        object_ptr<ISample> pWriteSample;
        // allocate for our sample
        RETURN_IF_FAILED(alloc_sample(pWriteSample));

        // create coding for signal
        auto codec = m_FeedbackSampleFactory.MakeCodecFor(pWriteSample);

        // set values
        RETURN_IF_FAILED(codec.SetElementValue(o_FeedbackStructId.filterId      , inputFilterId      ));
        RETURN_IF_FAILED(codec.SetElementValue(o_FeedbackStructId.feedbackStatus, inputFeedbackStatus));
        */
    RETURN_IF_FAILED(m_FeedbackStruct.writePin(m_WriterFeedback, (void *) &inputFeedback, m_pClock->GetStreamTime()));

    // write to sample
    //m_WriterFeedback << pWriteSample << flush << trigger;

    // done
    RETURN_NOERROR;
}

// ProcessActionCommandtActionCommand
tResult ActionStop::ProcessActionCommand(TActionStruct::Data inputActionCommand)
{
    // brake if command was made
    if(inputActionCommand.ui32Command == AC_SA_STOP_CAR)
    {
        //m_mutexCommandActivatedFlag.lock();
        m_propCommandActivated = tTrue;
        //m_mutexCommandActivatedFlag.unlock();
    }


    // done
    RETURN_NOERROR;
}
