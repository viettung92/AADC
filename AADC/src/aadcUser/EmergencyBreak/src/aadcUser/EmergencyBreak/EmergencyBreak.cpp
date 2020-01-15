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
$from the frist video session enhanced by Illmer and Xiangfei 21.08.2018
Annotation: this filer is based on the filter from the video session completed by the solution of the last year team and own ideas

Annotation about the LIDAR open angle: from car left 270° to car front 360°/0° to car right 90°
**********************************************************************/
#include <mutex>
#include "stdafx.h"
#include "EmergencyBreak.h"
#include <ADTF3_helper.h>
#include "ScmCommunication.h"

#define NDEBUG

#ifdef OBJECTDETECTION
static int compareAngle(const void * l1, const void *l2)
{
    TPolarCoordiante::Data *pc1 = (TPolarCoordiante::Data *) l1;
    TPolarCoordiante::Data *pc2 = (TPolarCoordiante::Data *) l2;
    return pc1->f32Angle < pc2->f32Angle;

}
#endif
// This will define the filter and expose it via plugin class factory.
// Class EmergencyBreak will be embedded to the filter and called by
// data trigger of "speed" by default.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_EMERGENCY_BREAK_FILTER,		// references to header file
        "cEmergencyBreak",			// label
        EmergencyBreak,				// class
        adtf::filter::pin_trigger({"inputSpeed"}));/*,
    adtf::filter::pin_trigger({"ultrasonic_struct"}),	// set trigger pin
    adtf::filter::pin_trigger({"laserscannerIn"}));	// set trigger pin //I am not sure how that works

*/


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
EmergencyBreak::EmergencyBreak()
{
    // ------------------------------------------
    // create pointers for adtf streamtypes
    // coding convention:       p******DataType
    //object_ptr<IStreamType> pSpeedDataType;

    // ------------------------------------------
    // initialize variables

    // initialization of memory sizes of media samples to create
    //nSize_emergency = 0;
    //nSize_tSignalValue = 0;

    // set 'obstacle detected' flag to false
    //m_bObstacleFlag = tFalse;

    m_Ultrasonic.registerPin(this,      m_ReaderUltrasonicStructInput, "ultrasonic_struct" );
    m_LaserScanner.registerPin(this,    m_ReaderLaserScanner,          "laserscannerIn");
    m_SpeedSignal.registerPin(this,     m_ReaderSpeed,                 "inputSpeed" );
    m_SpeedSignal.registerPin(this,     m_WriterSpeed,                 "outputSpeed");
    m_ActionCommand.registerPin(this,   m_WriterAction,               "actionInput");

    m_Ultrasonic.registerPin(this,   m_WriterUS,               "USOutput");

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

#ifdef  OBJECTDETECTION
    RegisterPropertyVariable("Defines the threshold for which max angle difference two values are considered as succeed"     , m_propF32ObjectThresholdAngle     );
    RegisterPropertyVariable("Defines the threshold for which max radius difference two values are considered as part of one object"     , m_propF32ObjectThresholdRadius     );
    RegisterPropertyVariable("Defines the thtreshold for the used lidar data"    , m_propF32ObjectThresholdDetection     );
#endif
    //RegisterPropertyVariable("No Restart After Obstacle"    , m_propNoRestartAfterObstacle);
    //RegisterPropertyVariable("Percentage for active breaking", m_propPercentageForActiveBreaking);
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult EmergencyBreak::Configure()
{
    //m_ui32TimestampSpeed = 0;
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
tResult EmergencyBreak::Process(tTimeStamp tmTimeOfTrigger)
{
    if (IS_OK(ProcessSpeedInput()))
    {

    }
    else if (IS_OK(ProcessUS()))
    {

    }
    else {
        TLaserScannerData::Data inputLaserData;
        if(IS_OK(m_LaserScanner.readPin(m_ReaderLaserScanner, (void *) &inputLaserData)))
        {
            TLaserScannerData::Data scan;
            scan = inputLaserData;
#ifndef NDEBUG
            for (uint32_t i=0; i<inputLaserData.ui32Size; i++)
            {
                LOG_INFO("Angle scan: %f", inputLaserData.tScanArray[i].f32Angle);
            }
#endif
            CheckEmergencyBreakLIDAR(scan);
<<<<<<< HEAD
=======
#ifdef OBJECTDETECTION
            TLaserScannerData::Data Objects;
            Objects = ObjectDetectionWithLidar(scan);
            fstream file_pose;
            for (uint32_t i = 0; i < (scan.ui32Size); i++)
            {
                file_pose.open( debugFileDirectoryObjects, ios::out | ios::app);
                file_pose <<scan.tScanArray[i].f32Radius << " " << scan.tScanArray[i].f32Angle << "\n";
                file_pose <<Objects.tScanArray[i].f32Radius << " " << Objects.tScanArray[i].f32Angle << "\n";
                file_pose <<"\n";
                file_pose.close();
            }
#endif
>>>>>>> 20ada4039981c534b426d3c801c5d29259f223f8
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
    if(IS_FAILED(res = TransmitSpeed(inputSpeedSignal)))
    {
        RETURN_ERROR(res);
    }
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

// TransmitSpeed
tResult EmergencyBreak::TransmitSpeed(TSignalValue::Data outputSignal)
{
    if((m_EmergencyBreakUSBool || m_EmergencyBreakLIDARBool) && m_propEnableEmergencyOutput){
        outputSignal.f32Value =0; //m_propPercentageForActiveBreaking*outputSignal.f32Value;
    }
    RETURN_IF_FAILED(m_SpeedSignal.writePin(m_WriterSpeed, (void *) &outputSignal, m_pClock->GetStreamTime()));

    // done
    RETURN_NOERROR;
}


//--------------------------------------------------------
//------!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!--------
//--------------------------------------------------------

//This funktion must be adjust to need of the state machine
// TransmitAction
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
#ifndef NDEBUG
        LOG_INFO("inputScanPoints.ui32Size: %d", inputScanPoints.ui32Size);
#endif
        for(uint32_t i = 0; i < (inputScanPoints.ui32Size);++i)
        {
            scanPoint = inputScanPoints.tScanArray[i];
            scanPoint.f32Angle = AngleCompensation(scanPoint.f32Angle);
            //
            if((m_propDebugToFile && scanPoint.f32Radius != 0.0))
            {
                fstream file_pose;
                file_pose.open( debugFileDirectory, ios::out | ios::app);
                //file_pose <<"Radius: " << scanPoint.f32Radius << " Angle: " << scanPoint.f32Angle << "\n";
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

#ifndef NDEBUG
                    LOG_INFO("Neuer Wert fuer ClosestObstacle");
                    LOG_INFO(cString::Format("at the moment closest obstacle radius: %f", closestObstacle.f32Radius));
#endif
                }
            }

        }
#ifndef NDEBUG
        LOG_INFO(cString::Format("closest obstacle radius: %f", closestObstacle.f32Radius));
#endif
        // is it too close? :/
        TActionStruct::Data actionCommand;
        actionCommand.ui32Command = (closestObstacle.f32Radius < tFloat32(m_propEmergencyThresholdFront)) ? AC_SA_STOP_CAR : 0;

        m_EmergencyBreakLIDARBool = (closestObstacle.f32Radius < tFloat32(m_propEmergencyThresholdFront)) ? tTrue : tFalse;

        TransmitAction(actionCommand);
    }
}

// this function checks with the US if an emergency break is needed
void EmergencyBreak::CheckEmergencyBreakUS(TUltrasonicStruct::Data ultrasonicSignal)
{
    LOG_INFO("%f", ultrasonicSignal.tRearCenter.f32Value);
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
#ifdef OBJECTDETECTION
TLaserScannerData::Data EmergencyBreak::ObjectDetectionWithLidar(TLaserScannerData::Data lasersample)
{
    /* zu ueberpruefen:
     * funktioniert es wie gedacht?
     * ist der Ausgabewert richtig?
     */
    //tFloat32 f32arrayRadius [lasersample.ui32Size];
    //tFloat32 f32arrayAngle [lasersample.ui32Size];
    tFloat32 f32counter = 0.0f;
    for (uint32_t i=0; (i<lasersample.ui32Size); i++)
    {
        //f32arrayRadius[i] = lasersample.tScanArray[i].f32Radius;
        //f32arrayAngle[i] = AngleCompensation(lasersample.tScanArray[i].f32Angle);
        lasersample.tScanArray[i].f32Angle = AngleCompensation(lasersample.tScanArray[i].f32Angle);
    }
    //sort the input data
    qsort((void *) lasersample.tScanArray, lasersample.ui32Size, sizeof(TPolarCoordiante::Data), compareAngle);//check if it works
    //check if it works
    TLaserScannerData::Data objectCounterArray = lasersample; //the radius contains the information about the counter
    //set the all elements of the radius to 0.0
    for (uint32_t i=0; (i<lasersample.ui32Size); i++)
    {
        objectCounterArray.tScanArray[i].f32Radius = 0.0f;

    }
    for (uint32_t i=0; (i<lasersample.ui32Size); i++)
    {
        if(i == 0)//intercept first value
        {
            if (lasersample.tScanArray[i].f32Radius < m_propF32ObjectThresholdDetection)
            {
                if(objectCounterArray.tScanArray[i].f32Radius != 0.0f)
                {
                    LOG_WARNING("There was already a value in the counter array. Nevertheless, continue the calculation! Angle: %f", objectCounterArray.tScanArray[i].f32Angle);
                }
                f32counter++; //it has to be a new object because it the first value
                objectCounterArray.tScanArray[i].f32Radius = f32counter;
            }
        }

        else if ((lasersample.tScanArray[i].f32Radius != 0.0f) && (lasersample.tScanArray[i].f32Radius < m_propF32ObjectThresholdDetection)) //zero is an invalid value
        {
            if(lasersample.tScanArray[i-1].f32Radius == 0.0f)//last value was 0
            {
                if(objectCounterArray.tScanArray[i].f32Radius != 0.0f)
                {
                    LOG_WARNING("There was already a value in the counter array. Nevertheless, continue the calculation! Angle: %f", objectCounterArray.tScanArray[i].f32Angle);
                }
                f32counter++;
                objectCounterArray.tScanArray[i].f32Radius = f32counter;
            }
            else if(lasersample.tScanArray[i-1].f32Radius >= m_propF32ObjectThresholdDetection)//last sample was higher than the treshold
            {
                if(objectCounterArray.tScanArray[i].f32Radius != 0.0f)
                {
                    LOG_WARNING("There was already a value in the counter array. Nevertheless, continue the calculation! Angle: %f", objectCounterArray.tScanArray[i].f32Angle);
                }
                f32counter++;
                objectCounterArray.tScanArray[i].f32Radius = f32counter;
            }
            else if(lasersample.tScanArray[i-1].f32Radius < m_propF32ObjectThresholdDetection)//the last sample was within the threhold
            {
                if(m_propF32ObjectThresholdRadius <= fabsf(lasersample.tScanArray[i-1].f32Radius-lasersample.tScanArray[i].f32Radius))//considered as a different object
                {
                    if(objectCounterArray.tScanArray[i].f32Radius != 0.0f)
                    {
                        LOG_WARNING("There was already a value in the counter array. Nevertheless, continue the calculation! Angle: %f", objectCounterArray.tScanArray[i].f32Angle);
                    }
                    f32counter++;
                    objectCounterArray.tScanArray[i].f32Radius = f32counter;
                }
                else if (m_propF32ObjectThresholdRadius > fabsf(lasersample.tScanArray[i-1].f32Radius-lasersample.tScanArray[i].f32Radius))//considered as the same object
                {
                    if(objectCounterArray.tScanArray[i].f32Radius != 0.0f)
                    {
                        LOG_WARNING("There was already a value in the counter array. Nevertheless, continue the calculation! Angle: %f", objectCounterArray.tScanArray[i].f32Angle);
                    }
                    objectCounterArray.tScanArray[i].f32Radius = f32counter;
                }
            }
        }
    }

    return objectCounterArray;
}
#endif
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
