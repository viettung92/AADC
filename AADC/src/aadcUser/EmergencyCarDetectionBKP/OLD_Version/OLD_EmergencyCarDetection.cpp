/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Code by Nico
**********************************************************************/
#include <mutex>
#include "stdafx.h"
#include "EmergencyCarDetection.h"
#include <ADTF3_helper.h>
#include "ScmCommunication.h"
#include "common_helper.h"
#include <fstream>

//#include <QAudioInput>

//main.cpp
//#include <QtWidgets/QApplication>
// #include "widget.h"

//xyseriesiodevice.h
#include <QtCore/QIODevice>
//#include <QtCharts/QChartGlobal>

//widget.h
// #include <QtWidgets/QWidget>
// #include <QtCharts/QChartGlobal>

//xyseriesiodevice.cpp
// #include "xyseriesiodevice.h"
// #include <QtCharts/QXYSeries>

// widget.cpp
// #include "widget.h"
// #include <QtMultimedia/QAudioDeviceInfo>
// #include <QtMultimedia/QAudioInput>
// #include <QtCharts/QChartView>
// #include <QtCharts/QLineSeries>
// #include <QtCharts/QChart>
// #include <QtWidgets/QVBoxLayout>
// #include <QtCharts/QValueAxis>
// #include "xyseriesiodevice.h"



//#include <QtCore/QIODevice>
//#include "hFiles/qconfig.h"
//#include "/opt/ADTF/3.3.3/3rdparty/qt5/include/QtCore/qconfig.h" //Wieso wird die datei nicht gefunden??????????????ß
//#include "/opt/ADTF/3.3.3/3rdparty/qt5/include/QtCore/qglobal.h"
//#include "/opt/ADTF/3.3.3/3rdparty/qt5/include/QtCore/qiodevice.h"
//#include "/opt/ADTF/3.3.3/3rdparty/qt5/include/QtMultimedia/qaudioinput.h"


//#define FIXED_POINT 32
#include "./kiss_fft130/kiss_fft.h"

#define DEBUG


// This will define the filter and expose it via plugin class factory.
// Class EmergencyBreak will be embedded to the filter and called by
// data trigger of "speed" by default.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_EMERGENCY_CAR_DETECTION_FILTER,		// references to header file
        "EmergencyCarDetection",			// label
        EmergencyCarDetection,				// class

        adtf::filter::pin_trigger({"audio"}));

//ADTF_PLUGIN("Audio Visualization Plugin", cAudioVisualization);


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
EmergencyCarDetection::EmergencyCarDetection()
{
    // ------------------------------------------
    // create pointers for adtf streamtypes
    // coding convention:       p******DataType
    //object_ptr<IStreamType> pSpeedDataType;
    // ------------------------------------------
    object_ptr<IStreamType> pType = make_object_ptr<cStreamType>(stream_meta_type_audio());
    create_pin(*this, m_ReaderAudio, "audio", pType);

#ifdef DEBUG
    debugToFile = true;
    debugToConsole = true;
    debugOnCar = false;

    if(debugOnCar) {debugFileDirectory = add_date_to_filename("/home/aadc/AADC/src/aadcUser/EmergencyCarDetection/debug/fft","txt").c_str();}
    else  {debugFileDirectory = add_date_to_filename("/home/nico/1_Audi_Cup/aadc2018/src/aadcUser/EmergencyCarDetection/debug/fft","txt").c_str();}
#endif

    //m_Ultrasonic.registerPin(this,      m_ReaderMic, "mic_input" );
    m_SpeedSignal.registerPin(this, m_ReaderSpeed, "speed");
    m_Feedback.registerPin(this,   m_WriterFeedback,               "feedback_output");

    // -----------------------------------------
    // set property variables
    RegisterPropertyVariable("Disable Filter", m_propBDisableFilter);//m_propBDisableFilter



    LOG_INFO("Config finished");

    //TODO: Cut&paste following block to Process (after testing)
    //***********************************************************************************

    float f32_testSignalSine[My_N], f32_windowedSignal[My_N], f32MagnitudeOut[My_N];
    kiss_fft_cpx cpxWindowSignal[My_N], cpxFftOut[My_N];
    size_t i;
    tInt16 i16SinFreq = 333;

    //Generate SineWave for testing
    for (i = 0; i < My_N; i++)
    {
        f32_testSignalSine[i] = sin(2 * M_PI * i16SinFreq * i/My_N );
    }

    Windowing_Hanning(f32_testSignalSine, f32_windowedSignal);
    FormatToCpx(f32_windowedSignal, cpxWindowSignal, My_N);
    ComputeFft("SineWave (complex)", cpxWindowSignal, cpxFftOut);
    Magnitude("SineWave - Magnitude", cpxFftOut, f32MagnitudeOut);


    //***********************************************************************************

}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult EmergencyCarDetection::Configure()
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
tResult EmergencyCarDetection::Process(tTimeStamp tmTimeOfTrigger)
{
    if (IS_OK(ProcessSpeedInput()))
    {

    }


    // done
    RETURN_NOERROR;
}


// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------



tResult EmergencyCarDetection::ProcessSpeedInput()
{
    TSignalValue::Data inputSpeedSignal;
    static tTimeStamp lasttmspeed = 0;
    tResult res;
    if(IS_FAILED(res = m_SpeedSignal.readPin(m_ReaderSpeed, (void *) &inputSpeedSignal, lasttmspeed)))
    {
        RETURN_ERROR(res);
    }
    lasttmspeed = inputSpeedSignal.ui32ArduinoTimestamp;


    /*if(IS_FAILED(res = TransmitSpeed(inputSpeedSignal)))
    {
        RETURN_ERROR(res);
    }
    */
   // RETURN_IF_FAILED(TransmitSpeed(inputSpeedSignal));
    RETURN_NOERROR;
}

void EmergencyCarDetection::FormatToCpx(const float in[My_N], kiss_fft_cpx cpxOut[My_N], tInt32 length)
{
    for (int i = 0; i < length; i++)
    {
    cpxOut[i].r = in[i];
    cpxOut[i].i = 0;
    }
}

void EmergencyCarDetection::Windowing_Hanning(const float signalBufferIn[My_N], float dataOut[My_N])
{
    for (int i = 0; i < My_N; i++) {
        double multiplier = 0.5 * (1 - cos(2*M_PI*i/(My_N-1)));
        dataOut[i] = multiplier * signalBufferIn[i];
    }
}


void EmergencyCarDetection::ComputeFft(const char* title, const kiss_fft_cpx cpxIn[My_N], kiss_fft_cpx cpxFftOut[My_N])
{
  kiss_fft_cfg cfg;

  printf("%s\n", title);
  //printf("Signal: Sinus with f=%s\My_N", SINEFREQ);
  if ((cfg = kiss_fft_alloc(My_N, 0/*is_inverse_fft*/, NULL, NULL)) != NULL)
  {
    size_t i;

    kiss_fft(cfg, cpxIn, cpxFftOut);
    free(cfg);
#ifdef DEBUG
    if(debugToConsole)
    {
        for (i = 0; i < My_N; i++)
          printf(" in[%2zu] = %+f , %+f    "
                 "out[%2zu] = %+f , %+f\n",
                 i, cpxIn[i].r, cpxIn[i].i,
                 i, cpxFftOut[i].r, cpxFftOut[i].i);
    }
#endif

  }
  else
  {
    printf("not enough memory?\n");
    exit(-1);
  }

#ifdef DEBUG

    fstream file_fft;
    file_fft.open(debugFileDirectory, ios::out | ios::app);
    file_fft << title << "\n \n";

    int i;
    for (i = 0; i < My_N; i++)
    {
        if(debugToFile)
        {


            file_fft << i << "\t\t\t\t\t\t" << cpxIn[i].r << "\t\t\t\t\t\t" << cpxIn[i].i << "\t\t\t\t\t\t" << cpxFftOut[i].r << "\t\t\t\t\t\t" << cpxFftOut[i].i << "\n";

        }
    }
    file_fft << "\n \n" << "--------------------------------------" << "\n\n\n";
    file_fft.close();
#endif
}

void EmergencyCarDetection::Magnitude(const char* title, const kiss_fft_cpx fftIn[My_N], float magnitudeOut[])

{
    for (int i = 0; i < My_N/2; i++)
    {
        magnitudeOut[i] = sqrt( (fftIn[i].r * fftIn[i].r)  +  (fftIn[i].i * fftIn[i].i) );
    }
#ifdef DEBUG
    fstream file_mag;
    cFilename debugFileDirectoryMagnitude = add_date_to_filename("/home/nico/1_Audi_Cup/aadc2018/src/aadcUser/EmergencyCarDetection/debug/magnitude","txt").c_str();
    file_mag.open(debugFileDirectoryMagnitude, ios::out | ios::app);
    file_mag << title << "\n \n";

    int i;
    for (i = 0; i < My_N/2; i++)
    {
        if(debugToFile)
        {


            file_mag << i   << "\t\t" << magnitudeOut[i] << "\n";
//* ((float)SAMPLEFREQUENCY/(float)My_N)
        }
    }
    //file_mag << "\n \n" << "--------------------------------------" << "\n\n\n";
    file_mag.close();
#endif
}


void EmergencyCarDetection::AudioIn()
{
    QFile destinationFile;   // Class member
    QAudioInput* audio; // Class member
    {
        destinationFile.setFileName("/tmp/test.raw");
        destinationFile.open( QIODevice::WriteOnly | QIODevice::Truncate );

        QAudioFormat format;
        // Set up the desired format, for example:
        format.setSampleRate(8000);
        format.setChannelCount(1);
        format.setSampleSize(8);
        format.setCodec("audio/pcm");
        format.setByteOrder(QAudioFormat::LittleEndian);
        format.setSampleType(QAudioFormat::UnSignedInt);

        QAudioDeviceInfo info = QAudioDeviceInfo::defaultInputDevice();
        if (!info.isFormatSupported(format)) {
            qWarning() << "Default format not supported, trying to use the nearest.";
            format = info.nearestFormat(format);
        }

        audio = new QAudioInput(format, this);
        connect(audio, SIGNAL(stateChanged(QAudio::State)), this, SLOT(handleStateChanged(QAudio::State)));

        QTimer::singleShot(3000, this, SLOT(stopRecording()));
        audio->start(&destinationFile);
        // Records audio for 3000ms
    }
}


//This funktion must be adjust to need of the state machine

// TransmitAction
//tResult EmergencyCarDetection::TransmitAction(TActionStruct::Data outputActionCommand)
//{
//    // set values
//    RETURN_IF_FAILED(m_ActionCommand.writePin(m_WriterAction, (void *) &outputActionCommand, m_pClock->GetStreamTime()));

//    // done
//    RETURN_NOERROR;
//}
