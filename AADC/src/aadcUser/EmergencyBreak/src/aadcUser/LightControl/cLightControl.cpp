/*********************************************************************
typedef struct
{
    tUInt32 ui32ArduinoTimestamp;
    tBool bValue;
} tBoolSignalValue;

**********************************************************************/

#include <mutex>
#include "stdafx.h"
#include "cLightControl.h"
#include <ADTF3_helper.h>
#include "ScmCommunication.h"

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_LIGHT_CONTORL_DATA_TRIGGERED_FILTER,
                                    "cLightControl",
                                    LightControl,
                                    // adtf::filter::pin_trigger({"carspeedInput","actionInput","setspeedInput"}));
                                    adtf::filter::pin_trigger({"actionInput"}));

LightControl::LightControl()
{
    SetName("LightControl Constructor");

    m_SignalValue.registerPin(this, carspeedInput, "carspeedInput");
    m_SignalValue.registerPin(this, setspeedInput, "setspeedInput");

    m_BoolValue.registerPin(this, headLightOutput,       "headLightOutput"      );
    m_BoolValue.registerPin(this, reverseLightOutput,    "reverseLightOutput"   );
    m_BoolValue.registerPin(this, brakeLightOutput,      "brakeLightOutput"     );
    m_BoolValue.registerPin(this, turnRightLightOutput,  "turnRightLightOutput" );
    m_BoolValue.registerPin(this, turnLeftLightOutput,   "turnLeftLightOutput"  );
    m_BoolValue.registerPin(this, hazardLightOutput,     "hazardLightOutput"    );

    m_FeedbackStruct.registerPin(this, feedbackOutput,   "feedbackOutput"       );

    m_ActionStruct.registerPin(this, actionInput,        "actionInputs"         );

    RegisterPropertyVariable("Factor for enabling brake lights", factorEnableBrake);
    RegisterPropertyVariable("Factor for disabling brake lights", factorDisableBrake);
    RegisterPropertyVariable("Speed Average Count", averageSampleCount);

    LOG_SUCCESS(cString::Format("Registration finished!"));

}


//implement the Configure function to read ALL Properties
tResult LightControl::Configure()
{



    factorEnableBrake = 0.95; // should be less than 1
    factorDisableBrake = 1.04; // should be greater than 1
    averageSampleCount = 10; // number of samples used to calculate speed average

    //default values
    HeadLight       = tFalse;
    ReverseLight    = tFalse;
    BrakeLight      = tFalse;
    TurnRightLight  = tFalse;
    TurnLeftLight   = tFalse;
    HazardLight     = tFalse;

    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    LOG_INFO(cString::Format("Configuration finished!"));


    RETURN_NOERROR;
}



tResult LightControl::ProcessSpeedInput(tTimeStamp &tmTimeOfTrigger)
{

    tSignalValue rec_carspeed;
    //ProcessSpeedInput
    //RETURN_IF_FAILED(getLastSampleTSignalValue(carspeedInput,o_SpeedIndex,m_SignalValueSampleFactory,rec_carspeed));
    RETURN_IF_FAILED(m_SignalValue.readPin(carspeedInput, (void *) & rec_carspeed));


    // save received car speed value
    last_carspeeds.push_back(fabsf(rec_carspeed.f32Value));
    if (last_carspeeds.size() > averageSampleCount) {
        last_carspeeds.pop_front();
    }


    // average calculation
    tFloat32 average_speed = 0;
    for (std::list<tFloat32>::iterator p = last_carspeeds.begin(); p != last_carspeeds.end(); ++p) {
        average_speed += (tFloat32)*p;
    }

    tFloat32 speed_average = average_speed / last_carspeeds.size();

    // enable brake lights
    if ( fabsf(rec_carspeed.f32Value) < ( speed_average * factorEnableBrake) && !brakelightEnabled ) {
        tFloat32 diff = (rec_carspeed.f32Value) - ( speed_average * factorEnableBrake);
        if(diff < -0.5 )
        {
            tBoolSignalValue brake_light_data;
            brake_light_data.ui32ArduinoTimestamp = 0;
            brake_light_data.bValue = tTrue;


            RETURN_IF_FAILED(m_BoolValue.writePin(brakeLightOutput, (void *) &brake_light_data, m_pClock->GetStreamTime()));
            brakelightEnabled = tTrue;
        }
    }
    // disable brake lights
    if ( fabsf(rec_carspeed.f32Value) > ( speed_average * factorDisableBrake ) && brakelightEnabled ) {
        tBoolSignalValue brake_light_data;
        brake_light_data.ui32ArduinoTimestamp = 0;
        brake_light_data.bValue = tFalse;

        RETURN_IF_FAILED(m_BoolValue.writePin(brakeLightOutput, (void *) &brake_light_data, m_pClock->GetStreamTime()));
        brakelightEnabled = tFalse;
    }

    RETURN_NOERROR;
}

tResult LightControl::ProcessSetSpeedInput(tTimeStamp &tmTimeOfTrigger)
{
    // get set car speed
    //ProcessSpeedInput
    tSignalValue rec_setcarspeed;
    RETURN_IF_FAILED(m_SignalValue.readPin(setspeedInput, (void *) & rec_setcarspeed));
    //RETURN_IF_FAILED(getLastSampleTSignalValue(setspeedInput,o_SpeedIndex,
    //                         m_SignalValueSampleFactory,rec_setcarspeed));


    // enable reverse lights
    if (rec_setcarspeed.f32Value < 0 && !reverselightEnabled ) {
        tBoolSignalValue reverse_light_data;
        reverse_light_data.ui32ArduinoTimestamp = 0;
        reverse_light_data.bValue = tTrue;

        RETURN_IF_FAILED(m_BoolValue.writePin(reverseLightOutput, (void *) &reverse_light_data, m_pClock->GetStreamTime()));
        reverselightEnabled = tTrue;
    }

    // disable reverse lights
    if ( rec_setcarspeed.f32Value >= 0 && reverselightEnabled ) {
        tBoolSignalValue reverse_light_data;
        reverse_light_data.ui32ArduinoTimestamp = 0;
        reverse_light_data.bValue = tFalse;


        RETURN_IF_FAILED(m_BoolValue.writePin(reverseLightOutput, (void *) &reverse_light_data, m_pClock->GetStreamTime()));

        reverselightEnabled = tFalse;
    }

    RETURN_NOERROR;
}


tResult LightControl::ProcessAction(tTimeStamp &tmTimeOfTrigger)
{

    TActionStruct::Data action;
    RETURN_IF_FAILED(m_BoolValue.readPin(actionInput, (void *) &action));

    // action = tActionStruct.Read_Action(mediaSample, F_LIGHT_FILTER);

    iFeedbackStruct feedback;
    feedback.ui8FilterId = F_LIGHT_FILTER;

    if(action.bEnabled == tTrue && action.bStarted == tTrue) {
        // Light Command: 7000 - 7999
        // 7 a b c
        // a	Head Light 0: no change, 1: disable, 2: enable
        // b	Reverse Light: 0: no change, 1: disable, 2: enable
        // c	Turn Lights: 0: no change, 1: disable, 2: left turn, 3: right turn, 4: hazard

        // check command input (range 7000 to 7999)
        if(action.ui32Command > F_LIGHT_FILTER * 1000 && action.ui32Command < (F_LIGHT_FILTER + 1) * 1000 ) {

            tUInt32 a_headLight = ( action.ui32Command % 1000 ) / 100;
            tUInt32 b_reverseLight = ( action.ui32Command % 100 ) / 10;
            tUInt32 c_turnLight = action.ui32Command % 10;

            // output clock for mediasamples
            auto cur_time = m_pClock->GetStreamTime();

            //////////////////////
            // process head light
            /////////////////////
            tBoolSignalValue headLightData;
            headLightData.ui32ArduinoTimestamp = 0;

            if ( a_headLight == 0 ) {
                // do nothing
            } else if ( a_headLight < 3 ) {
                if ( a_headLight == 1 )
                    headLightData.bValue = tFalse;
                else
                    headLightData.bValue = tTrue;
                RETURN_IF_FAILED(m_BoolValue.writePin(headLightOutput, (void *) &headLightData, m_pClock->GetStreamTime()));

            } else {
                LOG_ERROR(cString::Format("LightControl: Head lights, invalid command %d", a_headLight));
                RETURN_ERROR(ERR_INVALID_STATE);
            }
            //////////////////////
            // process reverse light
            /////////////////////
            tBoolSignalValue reverseLightData;
            reverseLightData.ui32ArduinoTimestamp = 0;

            if ( b_reverseLight == 0 ) {
                // do nothing
            } else if ( b_reverseLight == 1 ) {
                reverseLightData.bValue = tFalse;
                RETURN_IF_FAILED(m_BoolValue.writePin(reverseLightOutput, (void *) &reverseLightData, m_pClock->GetStreamTime()));

            } else if (b_reverseLight == 2) {
                reverseLightData.bValue = tTrue;
                RETURN_IF_FAILED(m_BoolValue.writePin(reverseLightOutput, (void *) &reverseLightData, m_pClock->GetStreamTime()));
            } else {
                LOG_ERROR(cString::Format("LightControl: Reverse lights, invalid command %d", b_reverseLight));
                RETURN_ERROR(ERR_INVALID_STATE);
            }

            //////////////////////
            // process turn lights
            /////////////////////
            tBoolSignalValue TurnRightLightData;
            tBoolSignalValue TurnLeftLightData;
            tBoolSignalValue HazardLightData;

            TurnLeftLightData.ui32ArduinoTimestamp = 0;
            TurnRightLightData.ui32ArduinoTimestamp = 0;
            HazardLightData.ui32ArduinoTimestamp = 0;

            if ( c_turnLight == 0 ) {
                // do nothing
            } else if (c_turnLight < 5) {

                // disable all (case 1)
                TurnLeftLightData.bValue = tFalse;
                TurnRightLightData.bValue = tFalse;
                HazardLightData.bValue = tFalse;

                if (c_turnLight == 2) {
                    // left turn
                    TurnLeftLightData.bValue = tTrue;

                    RETURN_IF_FAILED(m_BoolValue.writePin(turnLeftLightOutput, (void *) &TurnLeftLightData, m_pClock->GetStreamTime()));
                } else if (c_turnLight == 3) {
                    // right turn
                    TurnRightLightData.bValue = tTrue;

                    RETURN_IF_FAILED(m_BoolValue.writePin(turnRightLightOutput, (void *) &TurnRightLightData, m_pClock->GetStreamTime()));
                } else { // case 1 OR 4 (disable of hazard light should also disable turn lights
                    if (c_turnLight == 4) {
                        // hazard light
                        HazardLightData.bValue = tTrue;
                    }

                    RETURN_IF_FAILED(m_BoolValue.writePin(hazardLightOutput, (void *) &HazardLightData, m_pClock->GetStreamTime()));
                }

            } else {
                LOG_ERROR(cString::Format("LightControl: turn lights, invalid command %d", c_turnLight));
                RETURN_ERROR(ERR_INVALID_STATE);
            }

            feedback.ui32FeedbackStatus = action.ui32Command;
            m_FeedbackStruct.writePin(feedbackOutput,(void *) &feedback,cur_time);

        } else {
            LOG_ERROR(cString::Format("LightControl: Received command %d, expected command in range %d to %d", action.ui32Command,  F_LIGHT_FILTER * 1000, (F_LIGHT_FILTER + 1) * 1000 ));
        }
    }


    RETURN_NOERROR;

}


tResult LightControl::Process(tTimeStamp tmTimeOfTrigger)
{

    ProcessSpeedInput(tmTimeOfTrigger);
    ProcessAction(tmTimeOfTrigger);

    RETURN_NOERROR;
}

