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
//#include <mutex>
#include "stdafx.h"

//#define FIXED_POINT 32
#include "./kiss_fft130/kiss_fft.h"

#include "EmergencyCarDetection.h"
//#include <ADTF3_helper.h>
#include "ScmCommunication.h"
#include "common_helper.h"
#include <fstream>



#include <iomanip>
#include <complex>
#include <cmath>
#include <vector>


using namespace std;





//*************************************************************************************************

    tBool debugToFile = true;
    tBool debugToConsole = false;

    cFilename debugFileDirectorySine = add_date_to_filename("/home/aadc/AADC/src/aadcUser/EmergencyCarDetection/debug/sine","txt").c_str();
    cFilename debugFileDirectoryDisplayBuffer = add_date_to_filename("/home/aadc/AADC/src/aadcUser/EmergencyCarDetection/debug/display_buffer","txt").c_str();
    cFilename debugFileDirectoryMagnitude = add_date_to_filename("/home/aadc/AADC/src/aadcUser/EmergencyCarDetection/debug/magnitude","txt").c_str();

    cFilename debugFileDirectoryFft = add_date_to_filename("/home/aadc/AADC/src/aadcUser/EmergencyCarDetection/debug/fft","txt").c_str();

//load compare signal for cross correlation from file
cFilename fileDirectoryFftCompareSignal = "/home/aadc/AADC/src/aadcUser/EmergencyCarDetection/fftCompareSignal_siren.txt";


ADTF_PLUGIN("Emergency Car Detection Plugin", EmergencyCarDetection)

EmergencyCarDetection::EmergencyCarDetection() : m_chartView(nullptr), m_series(nullptr)
{
    // ------------------------------------------
    // create pointers for adtf streamtypes
    // coding convention:       p******DataType
    //object_ptr<IStreamType> pSpeedDataType;
    // ------------------------------------------
    object_ptr<IStreamType> pType = make_object_ptr<cStreamType>(stream_meta_type_audio());


    // -- Get Media Descriptions (for create_pin type) --
    //pTypeEcdActionStruct
    object_ptr<IStreamType> pTypeEcdActionStruct;
    cString structNameAction = "pTypeEcdActionStruct";
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(structNameAction.GetPtr(), pTypeEcdActionStruct, m_ecdActionStructSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_ecdActionStructSampleFactory, cString("ui8FilterId"), m_ddlEcdActionStructId.ui8FilterId));
        (adtf_ddl::access_element::find_index(m_ecdActionStructSampleFactory, cString("ui32Command"), m_ddlEcdActionStructId.ui32Command));
    }
    else
    {
        LOG_WARNING(cString::Format("No mediadescription for %s found!", structNameAction.GetPtr()));
    }

    //pTypeEcdFeedbackStruct
    object_ptr<IStreamType> pTypeEcdFeedbackStruct;
    cString structNameFeedback = "pTypeEcdFeedbackStruct";
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(structNameFeedback.GetPtr(), pTypeEcdFeedbackStruct, m_ecdFeedbackStrucktSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_ecdFeedbackStrucktSampleFactory, cString("ui8FilterId"),        m_ddlEcdFeedbackStructId.ui8FilterId));
        (adtf_ddl::access_element::find_index(m_ecdFeedbackStrucktSampleFactory, cString("ui32FeedbackStatus"), m_ddlEcdFeedbackStructId.ui32FeedbackStatus));
    }
    else
    {
        LOG_WARNING(cString::Format("No mediadescription for %s found!", structNameFeedback.GetPtr()));
    }

    create_pin(*this, m_WriterFeedback, "feedback_struct", pTypeEcdFeedbackStruct);
    create_pin(*this, m_ReaderAction, "action_struct", pTypeEcdActionStruct);
    create_pin(*this, m_inAudio, "audio", pType);
    create_pin(*this, m_WriterFFt, "fft", pType);
    //create_pin(*this, m_WriterCorrelation, "correlation", pType);


    // -----------------------------------------
    // set property variables
    RegisterPropertyVariable("Disable Filter", m_propBDisableFilter);//m_propBDisableFilter
    RegisterPropertyVariable("Debug", m_propBDebug);//m_propBDisableFilter
    RegisterPropertyVariable("Debug to file", m_propBDebugToFile);//m_propBDisableFilter

    //Initialize global Variables
    ui16_fftBufferCounter = 0;
    m_propBDisableFilter = tTrue;

    m_feedbackEmergencyCarDetected.ui8FilterId = F_EMERGENCY_CAR_DETECTION;
    m_feedbackEmergencyCarDetected.ui32FeedbackStatus = FB_ECD_EMERGENCY_CAR_DETECTED;

//    kiss_fft_cpx test[M_SAMPLES];
//    LoadCompareSignalFromFile(M_SAMPLES, test);
    if(m_propBDebug == tTrue)LOG_INFO("Config finished");
}
//--------------- END OF CONSTRUCTOR ---------------------------
//--------------------------------------------------------------


EmergencyCarDetection::~EmergencyCarDetection()
{
}

QWidget* EmergencyCarDetection::CreateView()
{
    m_series = new QLineSeries();

    QChart *chart = new QChart();
    chart->legend()->hide();
    chart->addSeries(m_series);
    chart->createDefaultAxes();
    chart->setTitle("Audio Input");


    QValueAxis* axisX = new QValueAxis;
    axisX->setLabelFormat("%g");
    axisX->setTitleText("Samples");
    axisX->setRange(0, m_nrSamplesToDisplay);

    QValueAxis *axisY = new QValueAxis;
    axisY->setRange(-1, 1);
    axisY->setTitleText("Audio level");

    chart->setAxisX(axisX, m_series);
    chart->setAxisY(axisY, m_series);

    m_chartView = new QChartView(chart);
    m_chartView->setRenderHint(QPainter::Antialiasing);

    return m_chartView;
}

tVoid EmergencyCarDetection::ReleaseView()
{
    delete m_chartView;
    m_chartView = nullptr;
    m_series = nullptr;
}


tResult EmergencyCarDetection::OnTimer()
{

    tFloat32 f32_newSamplesIn[N_SAMPLES]={};

    //this will empty the Reader queue and return the last sample received.
    //If no sample was during the time between the last execution of Process the "old sample" is return.
    object_ptr<const ISample> pSampleAudio;

    tInt32 my_i = 0;
    //Sample Action Reader
    object_ptr<const ISample> pSampleAction;
    while (IS_OK(m_ReaderAction.GetNextSample(pSampleAction)))
    {
        auto oDecoder = m_ecdActionStructSampleFactory.MakeDecoderFor(*pSampleAction);
        RETURN_IF_FAILED(oDecoder.IsValid());
        TActionStruct::Data actionInput;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlEcdActionStructId.ui32ArduinoTimestamp,
                         &actionInput.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlEcdActionStructId.ui8FilterId,
                         &actionInput.ui8FilterId));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlEcdActionStructId.bEnabled,
                         &actionInput.bEnabled));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlEcdActionStructId.bStarted,
                         &actionInput.bStarted));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlEcdActionStructId.ui32Command,
                         &actionInput.ui32Command));

        if(actionInput.ui32Command == AC_ECD_START_DETECTION)
        {
            m_propBDisableFilter = tFalse;
        }
    }


    while (IS_OK(m_inAudio.GetNextSample(pSampleAudio)))
    {
        adtf::ucom::object_ptr_shared_locked<const adtf::streaming::ISampleBuffer> pBuffer;
        pSampleAudio->Lock(pBuffer);
        //get number of audio samples in media samples
        const tInt32 availableSamples = tInt32(pBuffer->GetSize()) / m_codecPackageSizeInBytes;
        //if we have nothing displayed yet
        if (m_buffer.isEmpty())
        {
            m_buffer.reserve(m_nrSamplesToDisplay);
            for (tInt32 i = 0; i < m_nrSamplesToDisplay; ++i)
                m_buffer.append(QPointF(i, 0));
        }

        //find start index in current buffer
        tInt32 start = 0;
        if (availableSamples < m_nrSamplesToDisplay)
        {
            start = m_nrSamplesToDisplay - availableSamples;
            for (tInt32 s = 0; s < start; ++s)
            {
                m_buffer[s].setY(m_buffer.at(s + availableSamples).y()); // shift y values by size availableSamples
            }
        }

        //copy data to local variabe to get non-const char pointer
        std::vector<uchar> audioData;
        audioData.resize(pBuffer->GetSize() / sizeof(uchar));
        memcpy(audioData.data(), pBuffer->GetPtr(), audioData.size());

        uchar* data = static_cast<uchar*>(audioData.data());

        for (tInt32 s = start; s < m_nrSamplesToDisplay; ++s, data += m_codecPackageSizeInBytes)
        {
            m_buffer[s].setY(qreal(*data - 128) / qreal(128));
        }

        //update data for ui
        m_series->replace(m_buffer);

        pBuffer->Unlock();

        if(m_propBDebugToFile == tTrue)
        {
            fstream file_fft;
            file_fft.open(debugFileDirectoryDisplayBuffer, ios::out | ios::app);
            file_fft << "EmergencyCarDetection: m_buffer, samples: " << availableSamples << ", GetNextSample-Durchlauf (Reset by function call):"<< my_i << "\n \n";

            for (tInt32 i = 0; i < m_nrSamplesToDisplay; ++i, data += m_codecPackageSizeInBytes)
            {
                file_fft << i << "\t x:" << m_buffer[i].x() << "\t y:" << m_buffer[i].y() << "\n";
            }

            file_fft << "\n \n" << "--------------------------------------" << "\n\n\n";
            file_fft.close();
        my_i = my_i + 1;
        }
    }

    if(m_propBDisableFilter != tTrue)
    {
        //Call function for FFT processing of the samples
        tUInt16 ui16_BufferFull = ( N_SAMPLES / (200*(SAMPLEFREQUENCY/8000)) ) + 20;//Every count the buffer gets filled with about 200 samples (for microphone sampling with 8kHz)
        if(ui16_fftBufferCounter == ui16_BufferFull) //Call FFT function only when buffer is about to be full
        {
            ui16_fftBufferCounter = 20; //reset counter //!!!!!!!!!!!!!!!! UNCOMMENT for normal Filter use and delete "+ 20" three lines above !!!!!!!!!!!!!!!!!!!

            //Read out last N_SAMPLES from buffer and save it to f32_newSamplesIn
            for (tInt32 u= m_nrSamplesToDisplay-N_SAMPLES; u < m_nrSamplesToDisplay; ++u)
            {
                tInt32 i = u-(m_nrSamplesToDisplay-N_SAMPLES); //counting from 0
                f32_newSamplesIn[i] = m_buffer[u].y();
            }

            RETURN_IF_FAILED(ProcessingFft(f32_newSamplesIn));
        }
        ui16_fftBufferCounter ++;
    }

    RETURN_NOERROR;
}


tResult EmergencyCarDetection::ProcessingFft(const tFloat32 f32_newSamplesIn[])
{
    tFloat32 f32_windowedSignal[N_SAMPLES], f32MagnitudeOut[N_SAMPLES];// f32CorrelationOut[N_SAMPLES];
    tInt32 ZeroPaddedLength = N_SAMPLES * 2; //doubled length for ZeroPadding
//    tInt32 ZeroPaddedLengthCompareSignal = M_SAMPLES * 2;
    kiss_fft_cpx cpxWindowSignal[N_SAMPLES], cpxFftAudioSignal[N_SAMPLES], cpxZeroPaddedSignal[ZeroPaddedLength];
    kiss_fft_cpx cpxFftCompareSignal[M_SAMPLES], cpxCrossCorrelated[CORR_SAMPLES];

//    tFloat32 f32_testSignalSine[N_SAMPLES];
//    tInt16 i16SinFreq = 333;
//    //Generate SineWave for testing
//    for (tInt32 i = 0; i < N_SAMPLES; i++)
//    {
//        f32_testSignalSine[i] = sin(2 * M_PI * i16SinFreq * i/N_SAMPLES );
//    }


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Main Function calls
    Windowing_Hanning(f32_newSamplesIn, f32_windowedSignal);
    FormatToCpx(f32_windowedSignal, cpxWindowSignal, N_SAMPLES);
    cpxZeroPadding(cpxWindowSignal,cpxZeroPaddedSignal,N_SAMPLES, N_SAMPLES * 2);
    ComputeFft(cpxZeroPaddedSignal, cpxFftAudioSignal, ZeroPaddedLength);
    Magnitude(cpxFftAudioSignal, f32MagnitudeOut, N_SAMPLES);

#ifndef BOOL_RECORD_COMPARE_SIGNAL
    LoadCompareSignalFromFile(M_SAMPLES, cpxFftCompareSignal);
#endif

    CrossCorrelation(cpxFftAudioSignal, cpxFftCompareSignal, cpxCrossCorrelated, N_SAMPLES, M_SAMPLES);
//    FormatToReal(f32CorrelationOut)
    m_WriterFFt.Transmit();
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


    RETURN_NOERROR;
}


void EmergencyCarDetection::Windowing_Hanning(const tFloat32 signalBufferIn[], float dataOut[])
{
    for (tInt32 i = 0; i < N_SAMPLES; i++) {
        double multiplier = 0.5 * (1 - cos(2*M_PI*i/(N_SAMPLES-1)));
        dataOut[i] = multiplier * signalBufferIn[i];
    }
}


void EmergencyCarDetection::FormatToCpx(const float in[], kiss_fft_cpx cpxOut[], tInt32 length)
{
    for (tInt32 i = 0; i < length; i++)
    {
        cpxOut[i].r = in[i];
        cpxOut[i].i = 0;
    }
}

void EmergencyCarDetection::FormatToReal(kiss_fft_cpx cpxIn[], float out[],  tInt32 length)
{
    for (tInt32 i = 0; i < length; i++)
    {
        out[i] = cpxIn[i].r;
    }
}


void EmergencyCarDetection::cpxZeroPadding(kiss_fft_cpx cpxIn[], kiss_fft_cpx cpxZeroPaddedOut[], tInt32 lengthIn, tInt32 lengthOut)
{
    for (tInt32 i = 0; i < lengthIn ; i++)
    {
        cpxZeroPaddedOut[i].r = cpxIn[i].r;
        cpxZeroPaddedOut[i].i = cpxIn[i].i;
    }
    for (tInt32 i = lengthIn; i < lengthOut; i++) //Zero Padding
    {
        cpxZeroPaddedOut[i].r = 0;
        cpxZeroPaddedOut[i].i = 0;
    }
}


void EmergencyCarDetection::LoadCompareSignalFromFile(tInt32 readLength,kiss_fft_cpx cpxFftCompareSignal[]) //Readout compare signal values from file
{
    fstream file_compareSignal;
    string line ={};

    file_compareSignal.open(fileDirectoryFftCompareSignal, ios::in | ios::app);

    float tmp[readLength][2]={};
    tInt32 i = 0, k = 0;

    while (getline(file_compareSignal, line) && i<readLength)
    {
        float value;
        k = 0;
        stringstream ss(line);

        while (ss >> value)
        {
            tmp[i][k] = value;
            ++k;
        }
        ++i;
    }
    for(tInt32 u=0; u < readLength; ++u)
    {
//        if(m_propBDebug == tTrue){LOG_INFO("ReadFromFile: %f, %f", tmp[u][0], tmp[u][1]);}
        cpxFftCompareSignal[u].r = tmp[u][0];
        cpxFftCompareSignal[u].i = tmp[u][1];
    }

    file_compareSignal.close();
}


void EmergencyCarDetection::ComputeFft(const kiss_fft_cpx cpxIn[], kiss_fft_cpx cpxFftOut[], tInt32 lengthIn)
{
     kiss_fft_cfg cfg;
     kiss_fft_cpx cpxFftTemp[lengthIn]; //does contain all calculated elements (second half is unrelevant)

  if ((cfg = kiss_fft_alloc(lengthIn, 0/*is_inverse_fft*/, NULL, NULL)) != NULL)
  {

    kiss_fft(cfg, cpxIn, cpxFftTemp);
    free(cfg);

    for(tInt32 u=0; u < lengthIn/2; ++u) //cut away unrelevant second half of elements
    {
        cpxFftOut[u].r = cpxFftTemp[u].r;
        cpxFftOut[u].i = cpxFftTemp[u].i;
    }
//    if(m_propBDebug == tTrue)
//    {
//            size_t i;
//          for (i = 0; i < (size_t)lengthIn/2; i++)
//          printf(" in[%2zu] = %+f , %+f    "
//                     "out[%2zu] = %+f , %+f\n",
//                     i, cpxIn[i].r, cpxIn[i].i,
//                     i, cpxFftOut[i].r, cpxFftOut[i].i);
//    }

  }
  else
  {
    printf("not enough memory?\n");
    exit(-1);
  }
#ifdef BOOL_RECORD_COMPARE_SIGNAL
  {
      fstream file_fft;
      file_fft.open(fileDirectoryFftCompareSignal, ios::out | ios::app);

      tInt32 i;
      for (i = 0; i < lengthIn/2; i++)
      {
          file_fft <<  cpxFftOut[i].r << "\t" << cpxFftOut[i].i << "\n";
      }
      file_fft.close();
  }
#endif
if(m_propBDebug == tTrue)
{
    if(debugToFile)
    {
        fstream file_fft;
        file_fft.open(debugFileDirectoryFft, ios::out | ios::app);
        file_fft << "Compute FFT "<< "\n \n";

        tInt32 i;
        for (i = 0; i < lengthIn/2; i++)
        {
            file_fft << i << "\t\t\t\t\t\t" << cpxIn[i].r << "\t\t\t\t\t\t" << cpxIn[i].i << "\t\t\t\t\t\t" << cpxFftOut[i].r << "\t\t\t\t\t\t" << cpxFftOut[i].i << "\n";
        }
        file_fft << "\n \n" << "--------------------------------------" << "\n\n\n";
        file_fft.close();
    }
}
}


void EmergencyCarDetection::ComputeIfft(const kiss_fft_cpx cpxFftIn[], kiss_fft_cpx cpxOut[], tInt32 lengthIn)
{
    kiss_fft_cfg cfg2;

    if ((cfg2 = kiss_fft_alloc(lengthIn, 1/*is_inverse_fft*/, NULL, NULL)) != NULL)
    {
        kiss_fft(cfg2, cpxFftIn, cpxOut);
        free(cfg2);
    }
    else
    {
        printf("not enough memory?\n");
        exit(-1);
    }
}


void EmergencyCarDetection::Magnitude(const kiss_fft_cpx fftIn[], float magnitudeOut[], tInt32 length)

{
    for (int i = 0; i < length; i++)
    {
        magnitudeOut[i] = sqrt( (fftIn[i].r * fftIn[i].r)  +  (fftIn[i].i * fftIn[i].i) );
    }
    if(m_propBDebugToFile == tTrue)
    {
        {
            fstream file_mag;

            file_mag.open(debugFileDirectoryMagnitude, ios::out | ios::app);
            file_mag << "Magnitude - FFT Frequency Resolution: " << ((float)SAMPLEFREQUENCY/(float)length)<< "\n \n";

            int i;
            for (i = 0; i < length; i++)
            {
                file_mag << i << "\t" << magnitudeOut[i] << "\n";
        //        (float)SAMPLEFREQUENCY/(float)N_SAMPLES)
            }
            //file_mag << "\n \n" << "--------------------------------------" << "\n\n\n";

            file_mag.close();
        //        file_mag_csv.close();
        }

    }
}


void EmergencyCarDetection::CrossCorrelation(kiss_fft_cpx fftSampledAudioIn[],  kiss_fft_cpx fftCompareSignalIn[], kiss_fft_cpx correlationOut[], tInt32 lengthSampleAudioIn, tInt32 lengthCompareSignalIn)
{

// Implement   corr(a, b) = ifft(fft(a_and_zeros) * fft(b_and_zeros[reversed])) => O(n log n)
//        or   corr(a, b) = ifft(fft(a_and_zeros) * conj(fft(b_and_zeros))) =>  O(n) -> is faster
    correlationOut[CORR_SAMPLES]={};
    kiss_fft_cpx conjugatedFftCompareSignal[lengthCompareSignalIn]={};
    kiss_fft_cpx resElementMult[CORR_SAMPLES]={};
    kiss_fft_cpx tmpCompSignal[(lengthSampleAudioIn + lengthCompareSignalIn)], tmpAudioIn[(lengthSampleAudioIn + lengthCompareSignalIn)];
    //get the complex conjugate -> conj(b_and_zeros)
    for(tInt16 i=0; i < lengthCompareSignalIn; ++i)
    {
        conjugatedFftCompareSignal[i].i =  -fftCompareSignalIn[i].i;
    }

    cpxZeroPadding(conjugatedFftCompareSignal, tmpCompSignal, lengthCompareSignalIn, (lengthSampleAudioIn + lengthCompareSignalIn));
    cpxZeroPadding(fftSampledAudioIn, tmpAudioIn, lengthSampleAudioIn, (lengthSampleAudioIn + lengthCompareSignalIn));
    //elementwise multiplication of a_and_zeros and conj(b_and_zeros)
    for(tInt16 u=0; u < (lengthSampleAudioIn + lengthCompareSignalIn); ++u)
    {
        resElementMult[u].r =  (tmpAudioIn[u].r * tmpCompSignal[u].r) - (tmpAudioIn[u].i * tmpCompSignal[u].i);
        resElementMult[u].i =  (tmpAudioIn[u].r * tmpCompSignal[u].i) + (tmpAudioIn[u].i * tmpCompSignal[u].r);
    }
    for(tInt16 u=(lengthSampleAudioIn + lengthCompareSignalIn); u < CORR_SAMPLES; ++u) //special zero padding to fill to CORR_SAMPLES (which should be a power of 2)
    {
        resElementMult[u].r =  0;
        resElementMult[u].i =  0;
    }

    //compute cross correlation
    ComputeIfft(resElementMult,correlationOut,CORR_SAMPLES);

    if(m_propBDebugToFile == tTrue)
    {
        cFilename debugFileDirectoryCorrelation = add_date_to_filename("/home/aadc/AADC/src/aadcUser/EmergencyCarDetection/debug/correlation","txt").c_str();
        fstream file_correlation;
        file_correlation.open(debugFileDirectoryCorrelation, ios::out | ios::app);
        file_correlation << "Compute Correlation "<< "\n \n";

        tInt32 i;
        for (i = 0; i < CORR_SAMPLES; i++)
        {
            file_correlation << i << "\t\t" << correlationOut[i].r << "\t\t" << correlationOut[i].i << "\n";
        }
        file_correlation << "\n \n" << "--------------------------------------" << "\n\n\n";
        file_correlation.close();
    }
}

void EmergencyCarDetection::Detection(kiss_fft_cpx correlationOut[])
{



//    If emergency car is detected -> Send Feedback detected
//    TransmitFeedbackCarDetected();
}


tResult EmergencyCarDetection::Init(tInitStage eStage)
{
    RETURN_IF_FAILED(adtf::ui::cQtUIFilter::Init(eStage));

    RETURN_NOERROR;
}

tResult EmergencyCarDetection::TransmitFeedbackCarDetected()
{
    TFeedbackStruct::Data feedbackEmergencyCarDetection = m_feedbackEmergencyCarDetected;
    tTimeStamp lastTimestamp = 0;
    RETURN_IF_FAILED(m_feedback.writePin(m_WriterFeedback, (void *) &feedbackEmergencyCarDetection, lastTimestamp));
    if(m_propBDebug)LOG_INFO("Transmitted feedback emergency car detected");

    RETURN_NOERROR;
}

//tResult EmergencyCarDetection::FftOut(cPinWriter &output, void *content_ptr, tTimeStamp streamTime)
//{
//    Data &content = *((Data *) content_ptr);
//            object_ptr<ISample> sample;
//            tResult res;

//            if (IS_OK(res = alloc_sample(sample, streamTime)))
//            {
//                auto oCodec = factory.MakeCodecFor(sample);

//                if(IS_FAILED(res = oCodec.IsValid()))
//                {
//                    LOG_WARNING("CODEC CREATION FAILED!");
//                    RETURN_ERROR(res);
//                }

//                //Set timestamp
//                content.ui32ArduinoTimestamp = streamTime;

//                if(IS_FAILED(res = oCodec.SetElementValue(index.f32Value, content.f32Value)))
//                {
//                    LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32Value for index %d",type_name, index.f32Value));
//                    RETURN_ERROR(res);
//                }
//                if(IS_FAILED(res = oCodec.SetElementValue(index.ui32ArduinoTimestamp, content.ui32ArduinoTimestamp)))
//                {

//                    LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.ui32ArduinoTimestamp for index %d", type_name, index.ui32ArduinoTimestamp));
//                    RETURN_ERROR(res);
//                }
//            }
//            else
//            {
//                LOG_WARNING("writePin: alloc_sample did not work!");
//                RETURN_ERROR(res);

//            }

//            output << sample << flush << trigger;

//            RETURN_NOERROR;
//}
