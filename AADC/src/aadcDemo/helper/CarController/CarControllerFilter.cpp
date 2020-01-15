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
#include "ScmCommunication.h"

//*************************************************************************************************


ADTF_PLUGIN("Car Controller Plugin", cCarControllerFilter)


cCarControllerFilter::cCarControllerFilter() : m_pUiFileWidget(nullptr)
{
    object_ptr<IStreamType> pTypeSignalValue;
    if (IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory)))
    {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tUltrasonicStruct found!");
    }
    create_pin(*this, m_oOutputSteeringController, "steering", pTypeSignalValue);
    create_pin(*this, m_oOutputSpeedController, "speed", pTypeSignalValue);

    object_ptr<IStreamType> pTypeBoolSignalValue;
    if( IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBoolSignalValue, m_BoolSignalValueSampleFactory)))
    {
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlBoolSignalValueId.ui32ArduinoTimestamp));
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("bValue")              , m_ddlBoolSignalValueId.bValue));
    }
    else
    {
        LOG_INFO("No mediadescription for tUltrasonicStruct found!");
    }
    create_pin(*this, m_oOutputHeadLight   , "head_light"        , pTypeBoolSignalValue);
    create_pin(*this, m_oOutputTurnLeft    , "turn_signal_left"  , pTypeBoolSignalValue);
    create_pin(*this, m_oOutputTurnRight   , "turn_signal_right" , pTypeBoolSignalValue);
    create_pin(*this, m_oOutputBrakeLight  , "brake_light"       , pTypeBoolSignalValue);
    create_pin(*this, m_oOutputHazard      , "hazard_light"      , pTypeBoolSignalValue);
    create_pin(*this, m_oOutputReverseLight, "reverse_light"     , pTypeBoolSignalValue);


    create_pin(*this, m_oAction, "outputAction",  m_ActionStruct.object);

    create_pin(*this, m_oCarPose, "outputCarPose",  m_CarPose.object);
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


    // -- ACTION STOP ACTION BUTTONS
    connect(m_pUiFileWidget->getASActionButtonGroup(), SIGNAL(buttonClicked(int))     , this, SLOT(SubmitASAction(int)));
    connect(m_pUiFileWidget                       , SIGNAL(buttonClicked(int))     , this, SLOT(SubmitASAction(int)));
    // --

    // -- US ACC ACTION BUTTONS
    connect(m_pUiFileWidget->getUSACCActionButtonGroup(), SIGNAL(buttonClicked(int))     , this, SLOT(SubmitUSACCAction(int)));
    connect(m_pUiFileWidget                       , SIGNAL(buttonClicked(int))     , this, SLOT(SubmitUSACCAction(int)));
    // --

    // -- LANE DETECTION ACTION BUTTONS
    connect(m_pUiFileWidget->getLDActionButtonGroup(), SIGNAL(buttonClicked(int))     , this, SLOT(SubmitLDAction(int)));
    connect(m_pUiFileWidget                       , SIGNAL(buttonClicked(int))     , this, SLOT(SubmitLDAction(int)));
    // --

    // -- ARBITRARY ACTION BUTTONS
    connect(m_pUiFileWidget->getArbitraryButtonGroup(), SIGNAL(buttonClicked(int))     , this, SLOT(SubmitArbitraryAction(int)));
    connect(m_pUiFileWidget                       , SIGNAL(buttonClicked(int))     , this, SLOT(SubmitArbitraryAction(int)));
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
    transmitSignalValue(m_oOutputSteeringController, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp,  m_pClock->GetStreamTime(), m_ddlSignalValueId.value, value);

    RETURN_NOERROR;
}

tResult cCarControllerFilter::SendThrottle(int value)
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

//    transmitSignalValue(m_oOutputSpeedController, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, value);
    transmitSignalValue(m_oOutputSpeedController, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, m_pClock->GetStreamTime(), m_ddlSignalValueId.value, value);
    RETURN_NOERROR;
}


tResult cCarControllerFilter::SubmitASAction(int buttonId)
{
    TActionStruct::Data action;
    action.ui8FilterId = F_STOP_ACTION;
    action.bEnabled = tTrue;
    action.bStarted = tTrue;
    switch (buttonId)
    {
        case 24:
            action.ui32Command = AC_SA_STOP_CAR;
            LOG_INFO(cString::Format("Transmit Action: AC_SA_STOP_CAR (%d)", AC_SA_STOP_CAR));
              break;
       // case 25:
       //     action.ui32Command = AC_FP_PARKING_TRANS;
       //     LOG_INFO(cString::Format("Transmit Action: AC_FP_PARKING_TRANS (%d)", AC_FP_PARKING_TRANS));
       //       break;

        default:
            LOG_INFO(cString::Format("Dont transmit Action: (%d)", buttonId));
            RETURN_NOERROR;
    }
    action.ui32ArduinoTimestamp = m_pClock->GetStreamTime();
    m_ActionStruct.writePin(m_oAction, (void*) &action, m_pClock->GetStreamTime());

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
        case 1:
            action.ui32Command = AC_FP_SET_FOLLOW_FILE;
            LOG_INFO(cString::Format("Transmit Action: AC_FP_SET_FOLLOW_FILE (%d)", AC_FP_SET_FOLLOW_FILE));
              break;
        case 2:
            action.ui32Command = AC_FP_PARKING_TRANS;
            LOG_INFO(cString::Format("Transmit Action: AC_FP_PARKING_TRANS (%d)", AC_FP_PARKING_TRANS));
              break;
        case 3:
            action.ui32Command = AC_FP_GO_STRAIGHT;
            LOG_INFO(cString::Format("Transmit Action: AC_FP_GO_STRAIGHT (%d)", AC_FP_GO_STRAIGHT));
              break;
        case 4:
            action.ui32Command = AC_FP_CHANGE_TO_ORIGINAL_LANE;
            LOG_INFO(cString::Format("Transmit Action: AC_FP_CHANGE_TO_ORIGINAL_LANE (%d)", AC_FP_CHANGE_TO_ORIGINAL_LANE));
              break;
        case 5:
            action.ui32Command = AC_FP_LEFT_TURN;
            LOG_INFO(cString::Format("Transmit Action: AC_FP_LEFT_TURN (%d)", AC_FP_LEFT_TURN));
              break;
        case 6:
            action.ui32Command = AC_FP_STOP;
            LOG_INFO(cString::Format("Transmit Action: AC_FP_STOP (%d)", AC_FP_STOP));
              break;
        case 7:
            action.ui32Command = AC_FP_GO_BACKWARDS_BEFORE_OVERTAKING;
            LOG_INFO(cString::Format("Transmit Action: GO_BACKWARDS_BEFORE_OVERTAKING (%d)", AC_FP_GO_BACKWARDS_BEFORE_OVERTAKING));
              break;
        case 8:
            action.ui32Command = AC_FP_RIGHT_TURN;
            LOG_INFO(cString::Format("Transmit Action: AC_FP_RIGHT_TURN (%d)", AC_FP_RIGHT_TURN));
              break;
        case 9:
             action.ui32Command = RELOAD_XML_FILES;
             LOG_INFO(cString::Format("Transmit Action: RELOAD_XML_FILES (%d)", RELOAD_XML_FILES));
           break;

        default:
            LOG_INFO(cString::Format("Dont transmit Action: (%d)", buttonId));
            RETURN_NOERROR;
    }
    action.ui32ArduinoTimestamp = m_pClock->GetStreamTime();
    m_ActionStruct.writePin(m_oAction, (void*) &action, m_pClock->GetStreamTime());

    RETURN_NOERROR;
}

tResult cCarControllerFilter::SubmitUSACCAction(int buttonId)
{
    TActionStruct::Data action;
    action.ui8FilterId = F_LIGHT_FILTER;
    action.bEnabled = tTrue;
    action.bStarted = tTrue;
    switch (buttonId)
    {
        case 1:
            action.ui32Command = AC_UA_DRIVING_MODE;
            LOG_INFO(cString::Format("Transmit Action: AC_UA_DRIVING_MODE (%d)", AC_UA_DRIVING_MODE));
              break;
        case 2:
            action.ui32Command = AC_UA_INTERSECTION_CHECK_ALL;
            LOG_INFO(cString::Format("Transmit Action: AC_UA_INTERSECTION_CHECK_ALL (%d)", AC_UA_INTERSECTION_CHECK_ALL));
              break;
        case 9:
            action.ui32Command = AC_UA_OVERTAKE_CHECK_ONCOMING_TRAFFIC;
            LOG_INFO(cString::Format("Transmit Action: AC_UA_OVERTAKE_CHECK_ONCOMING_TRAFFIC (%d)", AC_UA_OVERTAKE_CHECK_ONCOMING_TRAFFIC));
              break;
        case 10:
            action.ui32Command = AC_UA_OVERTAKE_CHECK_OWN_LANE_STRAIGHT_LEFTHALF;
            LOG_INFO(cString::Format("Transmit Action: AC_UA_OVERTAKE_CHECK_OWN_LANE_STRAIGHT_LEFTHALF (%d)", AC_UA_OVERTAKE_CHECK_OWN_LANE_STRAIGHT_LEFTHALF));
              break;
       // case 4:
       //     action.ui32Command = AC_UA_PARKING_MODE;
      //      LOG_INFO(cString::Format("Transmit Action: AC_UA_PARKING_MODE (%d)", AC_UA_PARKING_MODE));
      //        break;
        case 6:
            action.ui32Command = AC_UA_INTERSECTION_CHECK_ONCOMING_TRAFFIC;
            LOG_INFO(cString::Format("Transmit Action: AC_UA_INTERSECTION_CHECK_ONCOMING_TRAFFIC (%d)", AC_UA_INTERSECTION_CHECK_ONCOMING_TRAFFIC));
              break;
        case 8:
            action.ui32Command = AC_UA_OVERTAKE_CHECK_OVERTAKE_ORIGINAL_LANE;
            LOG_INFO(cString::Format("Transmit Action: AC_UA_OVERTAKE_CHECK_OVERTAKE_ORIGINAL_LANE (%d)", AC_UA_OVERTAKE_CHECK_OVERTAKE_ORIGINAL_LANE));
              break;
        case 7:
            action.ui32Command = AC_UA_PARKING_CROSS_CHECK_ONCOMING_TRAFFIC;
            LOG_INFO(cString::Format("Transmit Action: AC_UA_PARKING_CROSS_CHECK_ONCOMING_TRAFFIC (%d)", AC_UA_PARKING_CROSS_CHECK_ONCOMING_TRAFFIC));
              break;
        case 11:
            action.ui32Command = AC_UA_CHECK_NO_MOVEMENT;
            LOG_INFO(cString::Format("Transmit Action: AC_UA_CHECK_NO_MOVEMENT (%d)", AC_UA_CHECK_NO_MOVEMENT));
              break;
        case 3:
            action.ui32Command = AC_UA_INTERSECTION_CHECK_CROSS_TRAFFIC_RIGHT;
            LOG_INFO(cString::Format("Transmit Action: AC_UA_INTERSECTION_CHECK_CROSS_TRAFFIC_RIGHT (%d)", AC_UA_INTERSECTION_CHECK_CROSS_TRAFFIC_RIGHT));
              break;
        case 13:
            action.ui32Command = AC_UA_OVERTAKE_CHECK_OWN_LANE_STRAIGHT;
            LOG_INFO(cString::Format("Transmit Action: AC_UA_OVERTAKE_CHECK_OWN_LANE_STRAIGHT (%d)", AC_UA_OVERTAKE_CHECK_OWN_LANE_STRAIGHT));
              break;
        case 5:
            action.ui32Command = AC_UA_PARKING_CROSS_PULLOUT_CHECK_CROSS_TRAFFIC;
            LOG_INFO(cString::Format("Transmit Action: AC_UA_PARKING_CROSS_PULLOUT_CHECK_CROSS_TRAFFIC (%d)", AC_UA_PARKING_CROSS_PULLOUT_CHECK_CROSS_TRAFFIC));
              break;
    //    case 13:
    //        action.ui32Command = AC_UA_CHECK_MOVING_AGAIN;
    //        LOG_INFO(cString::Format("Transmit Action: AC_UA_CHECK_MOVING_AGAIN (%d)", AC_UA_CHECK_MOVING_AGAIN));
    //          break;


        default:
            LOG_INFO(cString::Format("Dont transmit Action: (%d)", buttonId));
            RETURN_NOERROR;
    }
    action.ui32ArduinoTimestamp = m_pClock->GetStreamTime();
    m_ActionStruct.writePin(m_oAction, (void*) &action, m_pClock->GetStreamTime());

    RETURN_NOERROR;
}

tResult cCarControllerFilter::SubmitArbitraryAction(int buttonId)
{
    TActionStruct::Data action;
    action.bEnabled = tTrue;
    action.bStarted = tTrue;
    TPoseStruct::Data carpose;
    int com_num = 0;
    switch (buttonId)
    {
    case 1:
        action.ui8FilterId = F_FOLLOW_PATH;
        com_num = m_pUiFileWidget->getCarPose(carpose);
        LOG_INFO(cString::Format("Transmit CarPose:  x: %f y: %f yaw: %f speed: %f", carpose.f32PosX,carpose.f32PosY,carpose.f32Yaw,carpose.f32CarSpeed));
        if(com_num == 0){
            action.ui32Command = AC_FP_GOTO_XY;
            LOG_INFO(cString::Format("Transmit Action: AC_FP_GOTO_XY (%d)", AC_FP_GOTO_XY));
        } else {
            action.ui32Command = AC_FP_GOTO_XY_NOSTOP;
            LOG_INFO(cString::Format("Transmit Action: AC_FP_GOTO_XY_NOSTOP (%d)", AC_FP_GOTO_XY_NOSTOP));
        }
        carpose.ui32ArduinoTimestamp = m_pClock->GetStreamTime();
        m_CarPose.writePin(m_oCarPose, (void*) &carpose, m_pClock->GetStreamTime());
        break;
    case 2:
        action.ui32Command = m_pUiFileWidget->getArbitraryCommand();
        action.ui8FilterId = (int)(action.ui32Command/1000);
        LOG_INFO(cString::Format("Transmit Action: %d to Filter %d", (int)action.ui32Command, (int)action.ui8FilterId));
        break;


    default:
        LOG_INFO(cString::Format("Dont transmit Action: (%d)", buttonId));
        RETURN_NOERROR;
    }

    action.ui32ArduinoTimestamp = m_pClock->GetStreamTime();
    m_ActionStruct.writePin(m_oAction, (void*) &action, m_pClock->GetStreamTime());


    RETURN_NOERROR;
}

tResult cCarControllerFilter::SubmitLDAction(int buttonId)
{
    TActionStruct::Data action;
    action.ui8FilterId = F_LANE_DETECTION;
    action.bEnabled = tTrue;
    action.bStarted = tTrue;
    switch (buttonId)
    {
        case 1:
            action.ui32Command = AC_LD_DRIVE;
            LOG_INFO(cString::Format("Transmit Action: AC_LD_DRIVE (%d)", AC_LD_DRIVE));
              break;
        case 2:
            action.ui32Command = AC_LD_DRIVE_SLOW;
            LOG_INFO(cString::Format("Transmit Action: AC_LD_DRIVE_SLOW (%d)", AC_LD_DRIVE_SLOW));
              break;
        case 3:
            action.ui32Command = AC_LD_STOP;
            LOG_INFO(cString::Format("Transmit Action: AC_LD_STOP (%d)", AC_LD_STOP));
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
    action.ui8FilterId = F_ULTRASONIC_ACC;
    action.bEnabled = tTrue;
    action.bStarted = tTrue;
    switch (buttonId)
    {
        case 1:
            action.ui32Command = AC_LC_HEAD_LIGHT_ON;
            LOG_INFO(cString::Format("Transmit Action: AC_LC_HEAD_LIGHT_ON (%d)", AC_LC_HEAD_LIGHT_ON));
              break;
        case 2:
            action.ui32Command = AC_LC_HEAD_LIGHT_OFF;
            LOG_INFO(cString::Format("Transmit Action: AC_LC_HEAD_LIGHT_OFF (%d)", AC_LC_HEAD_LIGHT_OFF));
              break;
        case 3:
            action.ui32Command = AC_LC_RERVERSE_LIGHT_ON;
            LOG_INFO(cString::Format("Transmit Action: AC_LC_RERVERSE_LIGHT_ON (%d)", AC_LC_RERVERSE_LIGHT_ON));
              break;
        case 4:
            action.ui32Command = AC_LC_RERVERSE_LIGHT_OFF;
            LOG_INFO(cString::Format("Transmit Action: AC_LC_RERVERSE_LIGHT_OFF (%d)", AC_LC_RERVERSE_LIGHT_OFF));
              break;
        case 5:
            action.ui32Command = AC_LC_TURN_LEFT;
            LOG_INFO(cString::Format("Transmit Action: AC_LC_TURN_LEFT (%d)", AC_LC_TURN_LEFT));
              break;
        case 6:
            action.ui32Command = AC_LC_TURN_RIGHT;
            LOG_INFO(cString::Format("Transmit Action: AC_LC_TURN_RIGHT (%d)", AC_LC_TURN_RIGHT));
              break;
        case 7:
            action.ui32Command = AC_LC_TURN_AND_HAZARD_DISABLE;
            LOG_INFO(cString::Format("Transmit Action: AC_LC_TURN_AND_HAZARD_DISABLE (%d)", AC_LC_TURN_AND_HAZARD_DISABLE));
              break;
        case 8:
            action.ui32Command = AC_LC_HAZARD_LIGHT_ON;
            LOG_INFO(cString::Format("Transmit Action: AC_LC_HAZARD_LIGHT_ON (%d)", AC_LC_HAZARD_LIGHT_ON));
              break;
        case 9:
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
