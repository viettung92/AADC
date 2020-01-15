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
#define DEBUG

#pragma once

//------ Defines -------

#define CID_EMERGENCY_CAR_DETECTION_FILTER "emergency_car_detection.filter.user.aadc.cid"
//#define OBJECTDETECTION

#define N_SAMPLES 2048
#define SAMPLEFREQUENCY 8000

#ifndef M_PI
#define M_PI 3.1415926
#endif

#ifdef DEBUG
    tBool debugToFile, debugToConsole, debugOnCar;
    cFilename debugFileDirectory;
#endif

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
class EmergencyCarDetection : public cTriggerFunction
{
private:
    /*------------ STRUCTS -------------*/
    // create structs to hold information we get from SignalValues
    // coding convention: 	o_******Id

    TSignalValue m_SpeedSignal;
    TFeedbackStruct m_Feedback;

    /*------- SAMPLE FACTORIES ---------*/
    // a factory contains all data samples, which we can load into a tSignalValue
    // plus, it is responsible for (de)code the samples
    // coding convention:	m_******SampleFactory



    /*-------------- PINS --------------*/
    // create pins for input and output
    // coding convention:	m_Reader******	for input  signals
    //			m_Writer******	for output signals

    // input pins
    cPinReader m_ReaderAudio;
    cPinReader m_ReaderSpeed;

    // output pins
    cPinWriter m_WriterFeedback;

    /*------------ PROPERTIES ----------*/
    // set property variables
    // coding convention:	m_prop******
    adtf::base::property_variable<tBool> m_propBDisableFilter = tFalse;	// reduce angle of laser scanner, cause we dont need all


    /*------------ VARIABLES -----------*/
    // coding convention:	m_******
    //    tBool m_EmergencyBreakBool;

    /*struct LidarObstacles
    {
        tUInt32 ui32Size;
        LidarOneAngleObstacle tScanArrayEval[360];

        LidarObstacles()
        {
            ui32Size = 0;
        }

    };*/



    //debugFileDirectory = "../src/aadcUser/EmergencyBreak/debug/lidar.dat";

    //nicht verwendet
    /*
    // flag is set when obstacle detected
    tBool m_bObstacleFlag;
    // memory sizes of media samples to create
    tInt nSize_emergency;	// size of JuryEmergencyStop
    tInt nSize_tSignalValue;    // size of tSignalValue
    tInt nSize_tBoolSignalValue; // size of tBoolSignalValue

    // struct to save temporary sub-structs of mediatype "UltrasonicStruct"
    tUltrasonicStruct tmp_US_struct;

    // temporary tSignalValue to save current measured speed
    tSignalValue tmp_actualSpeed;

    // temporary tLaserScannerData to save current results
    tLaserScannerData tmp_Laser_Data;

    // temporary tSignalValue to save value from speed controller
    tSignalValue tmp_speedController;

    tBoolSignalValue tmp_EmergencyStop;
    */
    /*! The reference clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    //init timestamp
    //tUInt32 m_ui32TimestampSpeed;

public:
    /*------------ FUNCTIONS -----------*/
    // constructor
    EmergencyCarDetection();

    // destructor
    ~EmergencyCarDetection() = default;

    // Configure
    virtual tResult Configure() override;

    // Process
    virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;

    tResult ProcessSpeedInput();

    void AudioIn();

    void Windowing_Hanning(const float signalBufferIn[N_SAMPLES], float windowOut[N_SAMPLES]);

    void FormatToCpx(const float in[N_SAMPLES], kiss_fft_cpx cpxOut[N_SAMPLES], tInt32 length);

    void ComputeFft(const char* title, const kiss_fft_cpx cpxIn[N_SAMPLES], kiss_fft_cpx cpxFftOut[N_SAMPLES]);

    void Magnitude(const char* title, const kiss_fft_cpx fftIn[N_SAMPLES], float magnitudeOut[N_SAMPLES]);

}; // EmergencyBreak
