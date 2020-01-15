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

#pragma once

#include "stdafx.h"

#define CID_CAR_CONTROLLER  "car_controller.filter.demo.aadc.cid"

/*! forward declaration */
class cCarControllerWidget;

/*! the main class for the car controller filter. */
class cCarControllerFilter : public QObject, virtual public cQtUIFilter
{
    Q_OBJECT

public slots:

    /*!
     * Sends a steering.
     *
     * \param   value   The value.
     *
     * \return  Standard Result Code.
     */
    tResult SendSteering(int value);

    /*!
     * Sends a throttle.
     *
     * \param   value   The value.
     *
     * \return  Standard Result Code.
     */
    tResult SendThrottle(int value);

    /*!
     * Toggle lights.
     *
     * \param   buttonId    Identifier for the button.
     *
     * \return  Standard Result Code.
     */
    tResult ToggleLights(int buttonId);


    /*!
     * Submit actions.
     *
     * \param   buttonId    Identifier for the button.
     *
     * \return  Standard Result Code.
     */
    tResult SubmitLightAction(int buttonId);
    tResult SubmitArbitraryAction(int buttonId);
    tResult SubmitMTPAction(int buttonId);
    tResult SubmitASAction(int buttonId);
    tResult SubmitUSACCAction(int buttonId);
    tResult SubmitLDAction(int buttonId);


public:
    ADTF_CLASS_ID_NAME(cCarControllerFilter, CID_CAR_CONTROLLER, "Car Controller");
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::ui::ant::IQtXSystem),
                            REQUIRE_INTERFACE(adtf::services::IReferenceClock));

private:

    /*! The output speed controller */
    cPinWriter     m_oOutputSpeedController;
    /*! The output steering controller */
    cPinWriter     m_oOutputSteeringController;

    /*! The output sample writer turn right */
    cPinWriter     m_oOutputTurnRight;
    /*! The output sample writer turn left */
    cPinWriter     m_oOutputTurnLeft;
    /*! The output sample writer hazard */
    cPinWriter     m_oOutputHazard;
    /*! The output sample writer head light */
    cPinWriter     m_oOutputHeadLight;
    /*! The output sample writer reverse light */
    cPinWriter     m_oOutputReverseLight;
    /*! The output sample writer brake light */
    cPinWriter     m_oOutputBrakeLight;

    /*! A signal value identifier. */
    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;


    /*! The signal value sample factory */
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;

    /*! A bool signal value identifier. */
    struct tBoolSignalValueId
    {
        tSize ui32ArduinoTimestamp;
        tSize bValue;
    } m_ddlBoolSignalValueId;

    /*! A car pose struct identifier. */
    struct tCarPoseId
    {
        tSize ui32ArduinoTimestamp;
        tSize f32PosX;
        tSize f32PosY;
        tSize f32Roll;  //Rotation um Mittelachse (Hinten->Vorne-Achse)
        tSize f32Pitch; //Rotation um Querachse (Seite->Seiten-Achse)
        tSize f32Yaw;   //Rotation um Aufwärtachse (Boden->Decken-Achse)
        tSize f32CarSpeed;
        tSize f32Radius;
    } m_ddlCarPoseId;

    /*! A action identifier. */
    struct tActionId
    {
        tSize ui32ArduinoTimestamp;
        tSize ui8FilterId;
        tSize bEnabled;
        tSize bStarted;
        tSize ui32Command;
    } m_ddlActionId;

    /*! The signal value sample factory */
    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_ActionSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_CarPoseSampleFactory;


    /*! The user interface file widget */
    cCarControllerWidget*     m_pUiFileWidget;

     /*! The mutex */
    std::mutex m_oMutex;

    TActionStruct m_ActionStruct;
    TPoseStruct m_CarPose;
    cPinWriter m_oAction;
    cPinWriter m_oCarPose;
   
    /*! The reference clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;
public:

    /*! Default constructor. */
    cCarControllerFilter();

    /*! Destructor. */
    virtual ~cCarControllerFilter();

protected: // Implement cBaseQtFilter
    QWidget * CreateView() override;
    tVoid    ReleaseView() override;

    tResult OnIdle() override;

    tResult  OnTimer() override;

    tResult  Init(tInitStage eStage) override;
};
