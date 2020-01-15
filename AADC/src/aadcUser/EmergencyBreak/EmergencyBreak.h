/*********************************************************************
    Copyright (c) 2018
    Audi Autonomous Driving Cup. All rights reserved.

    Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
    3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
    4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    $from the frist video session enhanced by Illmer and Xiangfei 21.08.2018
    Annotation: this filer is based on the filter from the video session completed by the solution of the last year team and own ideas
    **********************************************************************/

#pragma once

#define CID_EMERGENCY_BREAK_FILTER "emergency_break.filter.user.aadc.cid"

#define OBJECTDETECTION

// namespace
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;

using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;


// EmergencyBreak
class EmergencyBreak : public cTriggerFunction
{
public:
    /*------------ STRUCTS -------------*/
    // create structs to hold information we get from SignalValues
    // coding convention: 	o_******Id

	
    TUltrasonicStruct m_Ultrasonic;

    TSignalValue m_SpeedSignal;

    TLaserScannerData m_LaserScanner;

    TActionStruct m_ActionCommand;
    TFeedbackStruct o_TFeedbackStruct;

    TRoadSignExt o_TReaderSignExt;

    TBoolSignalValue switchLightOn;

    TBoolSignalValue m_BoolValue;

	struct LidarOneAngleObstacle
    {
        tFloat32 f32Radius;
        tFloat32 f32Angle;
        tUInt32 ui32ObstacleCounter;
        LidarOneAngleObstacle()
        {
            f32Radius = 0.0f;
            f32Angle = 0.0f;
            ui32ObstacleCounter = 0.0f;
        }
    };
    
    struct LidarPoint
    {
		tUInt32 ui32Counter;
		tFloat32 f32Radius;
		tFloat32 f32Angle;
		tFloat32 f32x;
		tFloat32 f32y;
		LidarPoint()
		{
			ui32Counter = 0;
			f32Radius = 0.0f;
			f32Angle = 0.0f;
			f32x = 0.0f;
			f32y = 0.0;
		}
		
	};

    struct LidarObstacles
    {
        tUInt32 ui32Size;
        LidarOneAngleObstacle tScanArrayEval[360];

        LidarObstacles()
        {
            ui32Size = 0;
        }

    };
    /*------- SAMPLE FACTORIES ---------*/
    // a factory contains all data samples, which we can load into a tSignalValue
    // plus, it is responsible for (de)code the samples
    // coding convention:	m_******SampleFactory



    /*-------------- PINS --------------*/
    // create pins for input and output
    // coding convention:	m_Reader******	for input  signals
    //			m_Writer******	for output signals

    // input pins
    cPinReader m_ReaderSpeed;
    cPinReader m_ReaderLaserScanner;
    cPinReader m_ReaderUltrasonicStructInput;
    cPinReader m_RoadSignId;

    //cPinReader m_ReaderFeedbackObstacleDetector;        // 06: Obstacle Detection

    // output pins
    cPinWriter m_WriterSpeed;
    cPinWriter m_WriterAction;
    cPinWriter m_WriterActionObstacleDetector;        // 06: Obstacle Detection

    cPinWriter m_WriterLS;
    cPinWriter m_WriterUS;

    cPinWriter switchLightON_OUT;

    /*------------ PROPERTIES ----------*/
    // set property variables
    // coding convention:	m_prop******
    //LIDAR
    adtf::base::property_variable<tFloat32> m_propMinimumLaserScannerAngle = tFloat(-60.0f);	// reduce angle of laser scanner, cause we dont need all
    adtf::base::property_variable<tFloat32> m_propMaximumLaserScannerAngle = tFloat(60.0f);
    adtf::base::property_variable<tFloat32> m_propEmergencyThresholdFront  = tFloat(400.0f);	// min distance in mm, before emergency break
    adtf::base::property_variable<tBool> m_propEnableEmergencyFront = tBool(tTrue);
    //Ultrasonic
    adtf::base::property_variable<tFloat32> m_propEmergencyThresholdSideLeft = tFloat(1000.0f); //in mm
    adtf::base::property_variable<tBool> m_propEnableEmergencySideLeft = tBool(tFalse);
    adtf::base::property_variable<tFloat32> m_propEmergencyThresholdSideRight = tFloat(1000.0f); //in mm
    adtf::base::property_variable<tBool> m_propEnableEmergencySideRight = tBool(tFalse);
    adtf::base::property_variable<tFloat32> m_propEmergencyThresholdRear = tFloat(500.0f);// in mm
    adtf::base::property_variable<tBool> m_propEnableEmergencyRear = tBool(tTrue);
    adtf::base::property_variable<tBool> m_propEnableEmergencyOutput = tBool(tTrue);

    adtf::base::property_variable<tBool> m_propDebugToFile = tBool(tFalse);//

    //TFeedbackStruct::Data m_dataFeedbackObstacleDetector;

    //not used yet
    /*
    adtf::base::property_variable<tBool> m_propNoRestartAfterObstacle = tBool(tFalse);
    adtf::base::property_variable<tBool> m_propPercentageForActiveBreaking = tFloat(0.0f);//stops
    */

    /*------------ VARIABLES -----------*/
    // coding convention:	m_******
    //    tBool m_EmergencyBreakBool;
    tBool m_EmergencyBreakUSBool = tFalse;
    tBool m_EmergencyBreakLIDARBool = tFalse;
    tBool m_EmergencyBreakSTOPBool = tFalse;

    int counter = 0;

    cFilename debugFileDirectory= "/home/aadc/AADC/src/aadcUser/EmergencyBreak/debug/lidar.dat";
    
    /*! The reference clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    //init timestamp
    //tUInt32 m_ui32TimestampSpeed;
    //tUInt32 timestamp_obstacleDetector = -1;

public:
    /*------------ FUNCTIONS -----------*/
    // constructor
    EmergencyBreak();

    // destructor
    ~EmergencyBreak() = default;

    // Configure
    virtual tResult Configure() override;

    // Process
    virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;

    tResult ProcessUS();

    tResult ProcessRoadSign();

    tResult ProcessSpeedInput();
    // TransmitSpeed
    tResult TransmitSpeed(TSignalValue::Data inputSignal);

    // TransmitAction
    tResult TransmitAction(TActionStruct::Data outputActionCommand);

    // CheckEmergencyBreak with the LIDAR
    void CheckEmergencyBreakLIDAR(TLaserScannerData::Data inputScanPoints);

    void CheckEmergencyBreakUS(TUltrasonicStruct::Data ultrasonicSignal);


    static tFloat32 AngleCompensation(tFloat32 angle);
    
    static tFloat32 CalcAngleFromDistance(tFloat32 distance);
    
    EmergencyBreak::LidarObstacles GetLidarWeights(EmergencyBreak::LidarObstacles tmp_dataLidarObstacle);
}; // EmergencyBreak
