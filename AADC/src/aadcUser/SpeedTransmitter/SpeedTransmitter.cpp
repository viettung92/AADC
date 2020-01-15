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

#include "stdafx.h"
#include "SpeedTransmitter.h"

#include <common_helper.h>
#include <property_structs.h>
#include "ScmCommunication.h"
#include <cmath>
#include "tinyxml2.h"

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_SPEEDTRANSMITTER_DATA_TRIGGERED_FILTER,
    "SpeedTransmitter",
    SpeedTransmitter,
    adtf::filter::pin_trigger({"input"}));


SpeedTransmitter::SpeedTransmitter()
{
    m_SpeedId.registerPin(this, m_ReaderSpeed   , "inputSpeed");
    o_LaserScanner.registerPin(this, m_ReaderLaserScanner, "laserscannerIn");
}


//implement the Configure function to read ALL Properties
tResult SpeedTransmitter::Configure()
{
    RETURN_NOERROR;
}

tResult SpeedTransmitter::Process(tTimeStamp tmTimeOfTrigger)
{
    TSignalValue::Data ldSpeed;
    static tTimeStamp ldSpeedtts = 0;
    if(IS_OK(m_SpeedId.readPin(m_ReaderSpeed, (void *) &ldSpeed, ldSpeedtts)))
    {
        LOG_INFO("SpeedTransmitter: rcv  speed (%f)", ldSpeed.f32Value);
    }
/*
    // get the last laserscanner sample
    TLaserScannerData::Data tmp_dataInputLaserData;
    if(IS_OK(o_LaserScanner.readPin(m_ReaderLaserScanner, (void *) &tmp_dataInputLaserData)))
    {
        TPolarCoordiante::Data scanPoint;
        TPolarCoordiante::Data closestObstacle;
        closestObstacle.f32Angle  =     0.0f;
        closestObstacle.f32Radius = 99999.9f;

        // check for closest obstacle

        for(uint32_t i = 0; i < (tmp_dataInputLaserData.ui32Size);++i)
        {
            scanPoint = inputScanPoints.tScanArray[i];
            scanPoint.f32Angle = AngleCompensation(scanPoint.f32Angle);
            //
            if((x && scanPoint.f32Radius != 0.0))
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
        actionCommand.ui32Command = (closestObstacle.f32Radius < tFloat32(m_propEmergencyThresholdFront)) ? AC_SA_STOP_CAR : 0;

        m_EmergencyBreakLIDARBool = (closestObstacle.f32Radius < tFloat32(m_propEmergencyThresholdFront)) ? tTrue : tFalse;

    }
*/

    RETURN_NOERROR;
}
