/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The spelling mistake of brake was done by the employees of Audi ;)

This filter is an example filter to learn ADTF.
Filter was presented in first adtf online tutorial by Audi.
It takes 'speed' and the laser scanner as input and will signal an emergency break.
$from the frist video session enhanced by Illmer and Yu 21.08.2018
Annotation: this filer is based on the filter from the video session completed by the solution of the last year team and own ideas

Annotation about the LIDAR open angle: from car left 270° to car front 360°/0° to car right 90°
**********************************************************************/
#include <mutex>
#include "stdafx.h"
#include <math.h>
#include "EmergencyBreak.h"
#include "aadc_roadSign_enums.h"
#include <ADTF3_helper.h>
#include "ScmCommunication.h"
#include <property_structs.h>

//#include "../KuerNico_UltrasonicACC/KuerNico_UltrasonicACC.h"

#define USACC_CAR_WIDTH 10
#define PI M_PI
// This will define the filter and expose it via plugin class factory.
// Class EmergencyBreak will be embedded to the filter and called by
// data trigger of "speed" by default.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_EMERGENCY_BREAK_FILTER,		// references to header file
        "cEmergencyBreak",			// label
        EmergencyBreak,				// class

        adtf::filter::pin_trigger({"ultrasonic_struct", "laserscannerIn", "inputSpeed"}));




// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
EmergencyBreak::EmergencyBreak()
{
   

    m_Ultrasonic.registerPin(this,      m_ReaderUltrasonicStructInput, "ultrasonic_struct" );
    m_LaserScanner.registerPin(this,    m_ReaderLaserScanner,          "laserscannerIn");
    m_SpeedSignal.registerPin(this,     m_ReaderSpeed,                 "inputSpeed" );
    o_TReaderSignExt.registerPin(this,    m_RoadSignId   , "roadsignIn"   );
    m_SpeedSignal.registerPin(this,     m_WriterSpeed,                 "outputSpeed");
    m_ActionCommand.registerPin(this,   m_WriterAction,               "actionInput");
    //o_TFeedbackStruct.registerPin(this, m_ReaderFeedbackObstacleDetector         ,"feedbackObstacleDetectorIn"     );
   //o_TActionStruct.registerPin(this, m_WriterActionObstacleDetector        , "ActionObstacleDetectorOut"     );

    m_Ultrasonic.registerPin(this,   m_WriterUS,               "USOutput");

    switchLightOn.registerPin(this, switchLightON_OUT,  "switchLightON_OUT" );



    // -----------------------------------------
    // set property variables
    RegisterPropertyVariable("field of view min angle [deg]", m_propMinimumLaserScannerAngle);
    RegisterPropertyVariable("field of view max angle [deg]", m_propMaximumLaserScannerAngle);
    RegisterPropertyVariable("min distance to obstacle [mm]", m_propEmergencyThresholdFront );
    RegisterPropertyVariable("Enable front"                 , m_propEnableEmergencyFront);

    RegisterPropertyVariable("Emergency threshold Side Left in mm", m_propEmergencyThresholdSideLeft);
    RegisterPropertyVariable("Enable Side Left"             , m_propEnableEmergencySideLeft);
    RegisterPropertyVariable("Emergency threshold Side Right in mm", m_propEmergencyThresholdSideRight);
    RegisterPropertyVariable("Enable Side Right"            , m_propEnableEmergencySideRight);
    RegisterPropertyVariable("Emergency threshold Rear in mm", m_propEmergencyThresholdRear);
    RegisterPropertyVariable("Enable Rear"                  , m_propEnableEmergencyRear);

    RegisterPropertyVariable("Enable Emergency Output"      , m_propEnableEmergencyOutput);


    LOG_INFO("Config finished");

}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult EmergencyBreak::Configure()
{
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// -----------------------------s-
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
tResult EmergencyBreak::Process(tTimeStamp tmTimeOfTrigger)
{
    if(m_EmergencyBreakSTOPBool)
    {
            LOG_INFO("m_EmergencyBreakSTOPBool TRUE");
    }
    else
    {
        LOG_INFO("m_EmergencyBreakSTOPBool FALSE");
    }
     LOG_INFO("TRUE OR FALSE");
    if((m_EmergencyBreakUSBool || m_EmergencyBreakLIDARBool || m_EmergencyBreakSTOPBool))
    {
            LOG_INFO("TRUE");
    }
    else
    {
        LOG_INFO("FALSE");
    }

    LOG_INFO("------- Process --------");

    if (IS_OK(ProcessSpeedInput()))
    {

    }
    else if (IS_OK(ProcessUS()))
    {
    }

    else {
        ProcessRoadSign();
        TLaserScannerData::Data inputLaserData;
        if(IS_OK(m_LaserScanner.readPin(m_ReaderLaserScanner, (void *) &inputLaserData)))
        {
            TLaserScannerData::Data scan;
            scan = inputLaserData;

            CheckEmergencyBreakLIDAR(scan);

        }
    }

    // done
    RETURN_NOERROR;
}


// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

tResult EmergencyBreak::ProcessSpeedInput()
{
    TSignalValue::Data inputSpeedSignal;
    static tTimeStamp lasttmspeed = 0;
    tResult res;
    if(IS_FAILED(res = m_SpeedSignal.readPin(m_ReaderSpeed, (void *) &inputSpeedSignal, lasttmspeed)))
    {
        RETURN_ERROR(res);
    }
    lasttmspeed = inputSpeedSignal.ui32ArduinoTimestamp;
   
    RETURN_IF_FAILED(TransmitSpeed(inputSpeedSignal));
    RETURN_NOERROR;
}

tResult EmergencyBreak::ProcessUS()
{
    TUltrasonicStruct::Data ultrasonicSignal;
    static tTimeStamp lasttmus = 0;
    tResult res;
    if(IS_FAILED(res = m_Ultrasonic.readPin(m_ReaderUltrasonicStructInput, (void *) &ultrasonicSignal, lasttmus)))
    {
        RETURN_ERROR(res);
    }
    lasttmus = ultrasonicSignal.tSideLeft.ui32ArduinoTimestamp;
    if(IS_FAILED( res = m_Ultrasonic.writePin(m_WriterUS, (void *) &ultrasonicSignal, m_pClock->GetStreamTime())))
    {
        RETURN_ERROR(res);
    }

    CheckEmergencyBreakUS(ultrasonicSignal);
    RETURN_NOERROR;
}


tResult EmergencyBreak::ProcessRoadSign()
{
    TRoadSignExt::Data roadSign;
    static tTimeStamp lasttmus = 0;
    tResult res;
    if(IS_FAILED(res = o_TReaderSignExt.readPin(m_RoadSignId, (void *) &roadSign, lasttmus)))
    {
        RETURN_ERROR(res);
    }

    if(roadSign.i16Identifier == 1 && counter<200)
    {
        LOG_INFO("STOP FOR ROAD SIGN ID: (%d)", roadSign.i16Identifier);
        //TActionStruct::Data actionCommand;
        //actionCommand.ui32Command =  AC_SA_STOP_CAR;

        //TransmitAction(actionCommand);
        LOG_INFO("STOP");
        m_EmergencyBreakSTOPBool = tTrue;
        //lastStopSign = roadSign.ui32ArduinoTimestamp;
        //lasttmus = roadSign.ui32ArduinoTimestamp;

        //LOG_INFO("**********");
        //LOG_INFO(lastStopSign);

        LOG_INFO("COUNTER: (%d)", counter);

        TBoolSignalValue::Data lightOn;
        lightOn.ui32ArduinoTimestamp = 0;
        lightOn.bValue = tTrue;
                RETURN_IF_FAILED(switchLightOn.writePin(switchLightON_OUT, (void *) &lightOn, m_pClock->GetStreamTime()));

        LOG_INFO("-----> Emergency ON: .:::::LIGHT ON");
        counter++;
    }
    else if(counter >= 200 && counter <= 400)
    {
        counter++;
        roadSign.i16Identifier = 0;
        LOG_INFO("STOP FOR ROAD SIGN ID: (%d)", roadSign.i16Identifier);
        m_EmergencyBreakSTOPBool = tFalse;

        TBoolSignalValue::Data lightOn;
        lightOn.bValue = tFalse;
         lightOn.ui32ArduinoTimestamp = 0;
                RETURN_IF_FAILED(switchLightOn.writePin(switchLightON_OUT, (void *) &lightOn, m_pClock->GetStreamTime()));
        LOG_INFO("-----> Emergency OFF: .:::::LIGHT OFF");

    }
    else if(counter > 400)
    {
        counter = 0;
        m_EmergencyBreakSTOPBool = tFalse;
        TBoolSignalValue::Data lightOn;
        lightOn.bValue = tFalse;
         lightOn.ui32ArduinoTimestamp = 0;
                RETURN_IF_FAILED(switchLightOn.writePin(switchLightON_OUT, (void *) &lightOn, m_pClock->GetStreamTime()));
    }
    else
    {
        m_EmergencyBreakSTOPBool = tFalse;
        TBoolSignalValue::Data lightOn;
        lightOn.bValue = tFalse;
         lightOn.ui32ArduinoTimestamp = 0;
                RETURN_IF_FAILED(switchLightOn.writePin(switchLightON_OUT, (void *) &lightOn, m_pClock->GetStreamTime()));
                  LOG_INFO("-----> Emergency OFF: .:::::LIGHT OFF");
    }

   RETURN_NOERROR;
}

// TransmitSpeed
tResult EmergencyBreak::TransmitSpeed(TSignalValue::Data outputSignal)
{
    if((m_EmergencyBreakUSBool || m_EmergencyBreakLIDARBool || m_EmergencyBreakSTOPBool)){
        outputSignal.f32Value = 0; //m_propPercentageForActiveBreaking*outputSignal.f32Value;
    }
    LOG_INFO("EmergencyBreak:  speed (%f)", outputSignal.f32Value);
    RETURN_IF_FAILED(m_SpeedSignal.writePin(m_WriterSpeed, (void *) &outputSignal, (m_pClock->GetStreamTime())));
    // done
    RETURN_NOERROR;
}



tResult EmergencyBreak::TransmitAction(TActionStruct::Data outputActionCommand)
{
    // set values
    RETURN_IF_FAILED(m_ActionCommand.writePin(m_WriterAction, (void *) &outputActionCommand, m_pClock->GetStreamTime()));

    // done
    RETURN_NOERROR;
}

// checkEmergencyBreak
// this function checks the LIDAR values if an emergency break is needed
void EmergencyBreak::CheckEmergencyBreakLIDAR(TLaserScannerData::Data inputScanPoints)
{

    if (m_propEnableEmergencyFront)
    {
        TPolarCoordiante::Data scanPoint;
        TPolarCoordiante::Data closestObstacle;
        closestObstacle.f32Angle  =     0.0f;
        closestObstacle.f32Radius = 99999.9f;

        // check for closest obstacle

        for(uint32_t i = 0; i < (inputScanPoints.ui32Size);++i)
        {
            scanPoint = inputScanPoints.tScanArray[i];
            scanPoint.f32Angle = AngleCompensation(scanPoint.f32Angle);
            //
            if((/*x &&*/ scanPoint.f32Radius != 0.0))
            {
                fstream file_pose;
                file_pose.open( debugFileDirectory, ios::out | ios::app);
                file_pose <<scanPoint.f32Radius << " " << scanPoint.f32Angle << "\n";
                file_pose.close();
            }
            // laserscanner limits
            if(scanPoint.f32Angle >= tFloat32(m_propMinimumLaserScannerAngle) && scanPoint.f32Angle <= tFloat32(m_propMaximumLaserScannerAngle))
            {
                // radius 0 is invalid, get closest obstacle
                if(scanPoint.f32Radius > 0 && closestObstacle.f32Radius > scanPoint.f32Radius)
                {
                    closestObstacle = scanPoint;

                }
            }

        }



        // is it too close? :/
        TActionStruct::Data actionCommand;
        if(closestObstacle.f32Radius < tFloat32(m_propEmergencyThresholdFront))
        {
            //LOG_INFO("STOP");
            m_EmergencyBreakLIDARBool = tTrue;
        }
        else
        {
            m_EmergencyBreakLIDARBool = tFalse;
        }
        //TransmitAction(actionCommand);
    }
}

// this function checks with the US if an emergency break is needed
void EmergencyBreak::CheckEmergencyBreakUS(TUltrasonicStruct::Data ultrasonicSignal)
{
    tBool emergency_brake = tFalse;
    if ((m_propEnableEmergencyRear == tTrue)
            && (((ultrasonicSignal.tRearLeft.f32Value*10.0) < m_propEmergencyThresholdRear)
                || (ultrasonicSignal.tRearCenter.f32Value*10.0) < m_propEmergencyThresholdRear
                || (ultrasonicSignal.tRearRight.f32Value*10.0) < m_propEmergencyThresholdRear))
    {
        emergency_brake = tTrue;
    }
    else if ((m_propEnableEmergencySideLeft == tTrue)
             && ((ultrasonicSignal.tSideLeft.f32Value*10.0) < m_propEmergencyThresholdSideLeft))
    {
        emergency_brake = tTrue;
    }

    else if ((m_propEnableEmergencySideRight == tTrue)
             && ((ultrasonicSignal.tSideRight.f32Value*10.0) < m_propEmergencyThresholdSideRight))
    {
        emergency_brake = tTrue;
    }
    m_EmergencyBreakUSBool = emergency_brake;
    // is it too close? :/
    TActionStruct::Data actionCommand;
    actionCommand.ui32Command = (emergency_brake == tTrue) ? AC_SA_STOP_CAR : 0;

    TransmitAction(actionCommand);
}


EmergencyBreak::LidarObstacles GetLidarWeights(EmergencyBreak::LidarObstacles tmp_dataLidarObstacle)
{
    EmergencyBreak::LidarObstacles tmp_Obstacle = tmp_dataLidarObstacle;

    tFloat32 f32ClosestObstacel = 9999.0;

    for (uint32_t i=0; i<tmp_Obstacle.ui32Size; i++)
    {
        if (tmp_Obstacle.tScanArrayEval[i].ui32ObstacleCounter != 0)
        {
            if(tmp_Obstacle.tScanArrayEval[i].f32Radius > 10.0)
            {
                tFloat32 f32AngleThresholdForDetection = EmergencyBreak::CalcAngleFromDistance(tmp_Obstacle.tScanArrayEval[i].f32Radius);
                tFloat32 f32AngleMin = (-f32AngleThresholdForDetection/2);
                tFloat32 f32AngleMax = (f32AngleThresholdForDetection/2);
                if((tmp_dataLidarObstacle.tScanArrayEval[i].f32Angle <= (f32AngleMin)) ||
                        (tmp_Obstacle.tScanArrayEval[i].f32Angle >= f32AngleMax) )
                {
                    tmp_Obstacle.tScanArrayEval[i].ui32ObstacleCounter=0;
                }
                else if((tmp_Obstacle.tScanArrayEval[i].ui32ObstacleCounter != 0)&&(f32ClosestObstacel>tmp_Obstacle.tScanArrayEval[i].f32Radius))
                {
                    f32ClosestObstacel = tmp_Obstacle.tScanArrayEval[i].f32Radius;
                }
            }
            else
            {
                LOG_WARNING("USACC: GetLidarWeights: Object is too close! Radius: %f \n", tmp_Obstacle.tScanArrayEval[i].f32Radius);
            }
        }




    }

    return tmp_Obstacle;
}



tFloat32 EmergencyBreak::CalcAngleFromDistance(tFloat32 distance)
{
    return (asinf(tFloat32((USACC_CAR_WIDTH))/tFloat32(distance))*(tFloat32(180.0)/tFloat32(PI)));
}


//to compensate the inconsistante angle range-> output car right 90° to car left -90°
tFloat32 EmergencyBreak::AngleCompensation(tFloat32 angle)
{
    if (angle >= 0.0 && angle <= 90)
    {
        return angle;
    }
    else if(angle >= 270 && angle <= 360)
    {
        return (angle -360);
    }
    else
    {
        LOG_ERROR("Angle out of range!");
        return 100000;//Error value but without error handeling!
    }
}
