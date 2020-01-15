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

#include "CarControllerFilter.h"
#include "ADTF3_helper.h"
//*************************************************************************************************


ADTF_PLUGIN("Car Controller Plugin", cCarControllerFilter)


cCarControllerFilter::cCarControllerFilter() : m_pUiFileWidget(nullptr)
{
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }
    create_pin(*this, m_oOutputSteeringController, "steering", pTypeSignalValue);
    create_pin(*this, m_oOutputSpeedController, "speed", pTypeSignalValue);

    object_ptr<IStreamType> pTypeBoolSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBoolSignalValue, m_BoolSignalValueSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlBoolSignalValueId.ui32ArduinoTimestamp));
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("bValue")              , m_ddlBoolSignalValueId.bValue));
    }
    else
    {
        LOG_INFO("No mediadescription for tBoolSignalValue found!");
    }

    create_pin(*this, m_oOutputHeadLight   , "head_light"        , pTypeBoolSignalValue);
    create_pin(*this, m_oOutputTurnLeft    , "turn_signal_left"  , pTypeBoolSignalValue);
    create_pin(*this, m_oOutputTurnRight   , "turn_signal_right" , pTypeBoolSignalValue);
    create_pin(*this, m_oOutputBrakeLight  , "brake_light"       , pTypeBoolSignalValue);
    create_pin(*this, m_oOutputHazard      , "hazard_light"      , pTypeBoolSignalValue);
    create_pin(*this, m_oOutputReverseLight, "reverse_light"     , pTypeBoolSignalValue);
    create_pin(*this, m_oAction            , "action_out"        , m_ActionStruct.getObject());

}


cCarControllerFilter::~cCarControllerFilter()
{

}

QWidget* cCarControllerFilter::CreateView()
{
    // use single UI File in background
    m_pUiFileWidget = new cCarControllerWidget(nullptr);
    connect(m_pUiFileWidget                       , SIGNAL(steeringReceived(int)), this, SLOT(SendSteering(int)));
    connect(m_pUiFileWidget                       , SIGNAL(throttleReceived(int)), this, SLOT(SendThrottle(int)));

    connect(m_pUiFileWidget->getLightButtonGroup(), SIGNAL(buttonClicked(int))     , this, SLOT(ToggleLights(int)));
    connect(m_pUiFileWidget                       , SIGNAL(buttonClicked(int))     , this, SLOT(ToggleLights(int)));

    // -- LIGHT ACTION BUTTONS
    connect(m_pUiFileWidget->getLightActionButtonGroup(), SIGNAL(buttonClicked(int))     , this, SLOT(SubmitLightAction(int)));
    connect(m_pUiFileWidget                       , SIGNAL(buttonClicked(int))     , this, SLOT(SubmitLightAction(int)));
    // --

    // -- MOVE TO POINT ACTION BUTTONS
    connect(m_pUiFileWidget->getMTPActionButtonGroup(), SIGNAL(buttonClicked(int))     , this, SLOT(SubmitMTPAction(int)));
    connect(m_pUiFileWidget                       , SIGNAL(buttonClicked(int))     , this, SLOT(SubmitMTPAction(int)));
    // --
    return m_pUiFileWidget;
}

tVoid cCarControllerFilter::ReleaseView()
{
    delete m_pUiFileWidget;
    m_pUiFileWidget = nullptr;
}

tResult cCarControllerFilter::OnIdle()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    RETURN_NOERROR;
}

tResult cCarControllerFilter::OnTimer()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);
    
    RETURN_NOERROR;
}

tResult cCarControllerFilter::Init(tInitStage eStage)
{
    RETURN_IF_FAILED(adtf::ui::cQtUIFilter::Init(eStage));
    
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult cCarControllerFilter::SendSteering(int value)
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    //transmitSignalValue(m_oOutputSteeringController, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, value);
    transmitSignalValue(m_oOutputSteeringController, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, m_pClock->GetStreamTime(), m_ddlSignalValueId.value, value);
    RETURN_NOERROR;
}

tResult cCarControllerFilter::SendThrottle(int value)
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

//    transmitSignalValue(m_oOutputSpeedController, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, value);
    transmitSignalValue(m_oOutputSpeedController, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, m_pClock->GetStreamTime(), m_ddlSignalValueId.value, value);
    RETURN_NOERROR;
}


tResult cCarControllerFilter::SubmitMTPAction(int buttonId)
{
    TActionStruct::Data action;
    action.ui8FilterId = F_FOLLOW_PATH;
    action.bEnabled = tTrue;
    action.bStarted = tTrue;
    switch (buttonId)
    {
        case 15:
            action.ui32Command = AC_FP_GOTO_XY;
            LOG_INFO(cString::Format("Transmit Action: AC_FP_GOTO_XY (%d)", AC_FP_GOTO_XY));
              break;
        case 16:
            action.ui32Command = AC_FP_PARKING_TRANS;
            LOG_INFO(cString::Format("Transmit Action: AC_FP_PARKING_TRANS (%d)", AC_FP_PARKING_TRANS));
              break;
        case 17:
            action.ui32Command = AC_FP_GO_STRAIGHT;
            LOG_INFO(cString::Format("Transmit Action: AC_FP_GO_STRAIGHT (%d)", AC_FP_GO_STRAIGHT));
              break;
        case 18:
            action.ui32Command = AC_FP_GOTO_XY_NOSTOP;
            LOG_INFO(cString::Format("Transmit Action: AC_FP_GOTO_XY_NOSTOP (%d)", AC_FP_GOTO_XY_NOSTOP));
              break;
        case 19:
            action.ui32Command = AC_FP_LEFT_TURN;
            LOG_INFO(cString::Format("Transmit Action: AC_FP_LEFT_TURN (%d)", AC_FP_LEFT_TURN));
              break;
        case 20:
            action.ui32Command = AC_FP_STOP;
            LOG_INFO(cString::Format("Transmit Action: AC_FP_STOP (%d)", AC_FP_STOP));
              break;
        case 21:
            action.ui32Command = AC_FP_CONTINUE;
            LOG_INFO(cString::Format("Transmit Action: AC_FP_CONTINUE (%d)", AC_FP_CONTINUE));
              break;
        case 22:
            action.ui32Command = AC_FP_RIGHT_TURN;
            LOG_INFO(cString::Format("Transmit Action: AC_FP_RIGHT_TURN (%d)", AC_FP_RIGHT_TURN));
              break;

        default:
            LOG_INFO(cString::Format("Dont transmit Action: (%d)", buttonId));
            RETURN_NOERROR;
    }
    action.ui32ArduinoTimestamp = m_pClock->GetStreamTime();
    m_ActionStruct.writePin(m_oAction, (void*) &action, m_pClock->GetStreamTime());

    RETURN_NOERROR;
}

tResult cCarControllerFilter::SubmitLightAction(int buttonId)
{
    TActionStruct::Data action;
    action.ui8FilterId = F_LIGHT_FILTER;
    action.bEnabled = tTrue;
    action.bStarted = tTrue;
    switch (buttonId)
    {
        case 6:
            action.ui32Command = AC_LC_HEAD_LIGHT_ON;
            LOG_INFO(cString::Format("Transmit Action: AC_LC_HEAD_LIGHT_ON (%d)", AC_LC_HEAD_LIGHT_ON));
              break;
        case 7:
            action.ui32Command = AC_LC_HEAD_LIGHT_OFF;
            LOG_INFO(cString::Format("Transmit Action: AC_LC_HEAD_LIGHT_OFF (%d)", AC_LC_HEAD_LIGHT_OFF));
              break;
        case 8:
            action.ui32Command = AC_LC_RERVERSE_LIGHT_ON;
            LOG_INFO(cString::Format("Transmit Action: AC_LC_RERVERSE_LIGHT_ON (%d)", AC_LC_RERVERSE_LIGHT_ON));
              break;
        case 9:
            action.ui32Command = AC_LC_RERVERSE_LIGHT_OFF;
            LOG_INFO(cString::Format("Transmit Action: AC_LC_RERVERSE_LIGHT_OFF (%d)", AC_LC_RERVERSE_LIGHT_OFF));
              break;
        case 10:
            action.ui32Command = AC_LC_TURN_LEFT;
            LOG_INFO(cString::Format("Transmit Action: AC_LC_TURN_LEFT (%d)", AC_LC_TURN_LEFT));
              break;
        case 11:
            action.ui32Command = AC_LC_TURN_RIGHT;
            LOG_INFO(cString::Format("Transmit Action: AC_LC_TURN_RIGHT (%d)", AC_LC_TURN_RIGHT));
              break;
        case 12:
            action.ui32Command = AC_LC_TURN_AND_HAZARD_DISABLE;
            LOG_INFO(cString::Format("Transmit Action: AC_LC_TURN_AND_HAZARD_DISABLE (%d)", AC_LC_TURN_AND_HAZARD_DISABLE));
              break;
        case 13:
            action.ui32Command = AC_LC_HAZARD_LIGHT_ON;
            LOG_INFO(cString::Format("Transmit Action: AC_LC_HAZARD_LIGHT_ON (%d)", AC_LC_HAZARD_LIGHT_ON));
              break;
        case 14:
            action.ui32Command = AC_LC_SIGNALIZE_READY_ON_STARTUP;
            LOG_INFO(cString::Format("Transmit Action: AC_LC_SIGNALIZE_READY_ON_STARTUP (%d)", AC_LC_SIGNALIZE_READY_ON_STARTUP));
              break;

        default:
            LOG_INFO(cString::Format("Dont transmit Action: (%d)", buttonId));
            RETURN_NOERROR;
    }
    action.ui32ArduinoTimestamp = m_pClock->GetStreamTime();
    m_ActionStruct.writePin(m_oAction, (void*) &action, m_pClock->GetStreamTime());

    RETURN_NOERROR;
}

tResult cCarControllerFilter::ToggleLights(int buttonId)
{
    static bool headToggle;
    static bool reverseToggle;
    static bool brakeToggle;
    static bool turnRightToggle;
    static bool turnLeftToggle;
    static bool hazzardLightToggle;

    switch (buttonId)
    {
        case 0: // Head
            transmitBoolSignalValue(m_oOutputHeadLight, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, !headToggle);
            headToggle = !headToggle;
            LOG_INFO(cString::Format("Heads toggled: %d", headToggle));
            break;
        case 1: // Brake
            transmitBoolSignalValue(m_oOutputBrakeLight, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, !brakeToggle);
            brakeToggle = !brakeToggle;
            LOG_INFO(cString::Format("Brake toggled: %d", brakeToggle));
            break;
        case 2: // Reverse
            transmitBoolSignalValue(m_oOutputReverseLight, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, !reverseToggle);
            reverseToggle = !reverseToggle;
            LOG_INFO(cString::Format("Reverse toggled: %d", reverseToggle));
            break;
        case 3: // Hazard
            transmitBoolSignalValue(m_oOutputHazard, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, !hazzardLightToggle);
            hazzardLightToggle = !hazzardLightToggle;
            LOG_INFO(cString::Format("Hazard toggled: %d", hazzardLightToggle));
            break;
        case 4: // Left
            transmitBoolSignalValue(m_oOutputTurnLeft, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, !turnLeftToggle);
            turnLeftToggle = !turnLeftToggle;
            LOG_INFO(cString::Format("Turn Left toggled: %d", turnLeftToggle));
            break;
        case 5: // Right
            transmitBoolSignalValue(m_oOutputTurnRight, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, !turnRightToggle);
            turnRightToggle = !turnRightToggle;
            LOG_INFO(cString::Format("Turn right toggled: %d", turnRightToggle));
            break;

        default:
            break;
    }

    RETURN_NOERROR;
}
