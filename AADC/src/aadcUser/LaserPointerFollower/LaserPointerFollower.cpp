/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. Team FAUtonomous. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

This filter will detect lanes and propose a steering angle to hold lanes.
1. Set the ROI, where the lanes are located.
2. Transform image to bird's eye view.
2. Detect lanes with Hough Transform.
4. Calculate steering angle

**********************************************************************/

#include "stdafx.h"
#include "LaserPointerFollower.h"
#include <common_helper.h>
#include <property_structs.h>
#include "ScmCommunication.h"
#include <cmath>
#include "tinyxml2.h"

// This will define the filter and expose it via plugin class factory.
// Class cTemplateFilter will be embedded to the filter and called by
// data trigger of "speed" by default.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
    CID_LASERPOINTER_FILTER,		// references to header file
    "LaserPointerFollower",              // label
    LaserPointerFollower,                // class
    adtf::filter::pin_trigger({"inputVideo"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
LaserPointerFollower::LaserPointerFollower()
{

    // --------- VIDEO PINS ----------
    //create and set inital input format type
    m_sInputImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pVideoDataType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pVideoDataType, m_sInputImageFormat);

    // set output type
    adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeOutput, m_sOutputImageFormat);

    // Register input pin
    Register(m_ReaderVideo, "inputVideo" , pVideoDataType);
    // Register output pin
    Register(m_WriterDebugBirdVideo, "outputDebugBirdVideo"   , pTypeOutput);

    m_SteeringId.registerPin(this, m_WriterSteering, "outputSteering" );
    m_SpeedId   .registerPin(this, m_WriterSpeed   , "outputSpeed");

    //register callback for type changes
    m_ReaderVideo.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pVideoDataType) -> tResult
    {
        return ChangeType(m_ReaderVideo, *pVideoDataType.Get());
    });

    RegisterPropertyVariable("Frames", m_propFrame);

    m_SteeringData = 0;
    m_ui32FrameCounter = 0;
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult LaserPointerFollower::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));


    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
// This is how to read a sample
tResult LaserPointerFollower::Process(tTimeStamp)
{

    // video input
    object_ptr<const ISample> pReadVideoSample;
    if (IS_OK(m_ReaderVideo.GetNextSample(pReadVideoSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadVideoSample->Lock(pReadBuffer)))
        {
            if(m_ui32FrameCounter == m_propFrame)
            {

                // create a opencv matrix from the media sample buffer
                Mat inputImage (cv::Size(m_sInputImageFormat.m_ui32Width, m_sInputImageFormat.m_ui32Height),
                                CV_8UC3, (uchar*)pReadBuffer->GetPtr());

                // set our global mat to input image
                m_MatInputImage       = inputImage;
                m_MatTransformedImage = m_MatInputImage.clone();

                cvtColor(m_MatTransformedImage, m_MatTransformedImage, CV_RGB2HSV);
                Mat mask (cv::Size(m_sInputImageFormat.m_ui32Width, m_sInputImageFormat.m_ui32Height),
                                CV_8UC3, Scalar(0, 0, 0));
                inRange(m_MatTransformedImage, Scalar(60, 60, 60), Scalar(90, 255, 255), mask);


                // Dilation to get better edges
                Mat element = getStructuringElement(MORPH_RECT,
                                                    Size(5, 5),
                                                    Point(3, 3));
                Mat eelement = getStructuringElement(MORPH_RECT,
                                                    Size(3, 3),
                                                    Point(1, 1));
                // Apply the dilation operation
                erode(mask, mask, eelement);
                dilate(mask, mask, element);


                Mat leftMat = mask(Rect(50, 600, m_sInputImageFormat.m_ui32Width/2 - 50, m_sInputImageFormat.m_ui32Height - 600));
                Mat rightMat = mask(Rect(m_sInputImageFormat.m_ui32Width/2 , 600,
                                         m_sInputImageFormat.m_ui32Width/2 - 50, m_sInputImageFormat.m_ui32Height - 600));

                int zeroThreshold = 100;
                int leftZeros  = countNonZero(leftMat);
                int rightZeros = countNonZero(rightMat);
                bool left = false;
                bool right = false;
                if(leftZeros >= zeroThreshold && leftZeros > rightZeros)
                {
                    left = true;
                    right = false;
                }
                if(rightZeros >= zeroThreshold && rightZeros > leftZeros)
                {
                    left = false;
                    right = true;
                }
                cvtColor(mask, mask, CV_GRAY2BGR);

                if(!left && !right)
                {
                    if(m_SteeringData <= 17&& m_SteeringData >= -17)
                    {
                        m_SteeringData = 7;
                    }
                    if(m_SteeringData < 7)
                    {
                        m_SteeringData += 20;
                    }
                    if(m_SteeringData > 7)
                    {
                        m_SteeringData -= 20;
                    }
                }
                if(!left)
                {
                    rectangle(mask, Rect(50, 600, m_sInputImageFormat.m_ui32Width/2 - 50, m_sInputImageFormat.m_ui32Height - 600),
                              Scalar(0, 0, 255), CV_AA);
                }
                else
                {
                    rectangle(mask, Rect(50, 600, m_sInputImageFormat.m_ui32Width/2 - 50, m_sInputImageFormat.m_ui32Height - 600),
                              Scalar(0, 255, 0), CV_AA);
                    if(m_SteeringData > 0)
                    {
                        m_SteeringData = 0;
                    }
                    m_SteeringData -= 10;
                    if(m_SteeringData < -65)
                    {
                        m_SteeringData = -65;
                    }
                }
                if(!right)
                {
                    rectangle(mask, Rect(m_sInputImageFormat.m_ui32Width/2 , 600, m_sInputImageFormat.m_ui32Width/2 - 50, m_sInputImageFormat.m_ui32Height - 600),
                              Scalar(0, 0, 255), CV_AA);
                }
                else
                {
                    rectangle(mask, Rect(m_sInputImageFormat.m_ui32Width/2 , 600, m_sInputImageFormat.m_ui32Width/2 - 50, m_sInputImageFormat.m_ui32Height - 600),
                              Scalar(0, 255, 0), CV_AA);
                    if(m_SteeringData < 0)
                    {
                        m_SteeringData = 0;
                    }
                    m_SteeringData += 10;
                    if(m_SteeringData > 65)
                    {
                        m_SteeringData = 65;
                    }
                }

                // output steering SignalValue
                TSignalValue::Data steeringSignalValue;
                steeringSignalValue.f32Value = m_SteeringData;
                RETURN_IF_FAILED(m_SteeringId.writePin(m_WriterSteering, (void *) &steeringSignalValue,
                                                       m_pClock->GetStreamTime()));
                m_MatTransformedImage = mask.clone();

                m_ui32FrameCounter = 0;
            }

            m_ui32FrameCounter++;
        }
        // output//Write processed Image to Output Pin
        if (!m_MatTransformedImage.empty())
        {
            //update output format if matrix size does not fit to
            if (m_MatTransformedImage.total() * m_MatTransformedImage.elemSize() != m_sOutputImageFormat.m_szMaxByteSize)
            {
                setTypeFromMat(m_WriterDebugBirdVideo, m_MatTransformedImage);
            }
            // write to pin
            writeMatToPin(m_WriterDebugBirdVideo, m_MatTransformedImage, m_pClock->GetStreamTime());
        }

    }
    // output speed SignalValue
    TSignalValue::Data speedSignalValue;
    speedSignalValue.f32Value = 1;
    //LOG_INFO("send %f", speedSignalValue.f32Value);
    RETURN_IF_FAILED(m_SpeedId.writePin(m_WriterSpeed, (void *) &speedSignalValue,
                                           m_pClock->GetStreamTime()));
    RETURN_NOERROR;
}


// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

// ------------- Functions for VIDEO stuff
// setTypeFromMat
// we need this for video outputs
void LaserPointerFollower::setTypeFromMat(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tBool keepFormat)
{
    adtf::streaming::tStreamImageFormat outputFormat;
    outputFormat.m_ui32Height = outputImage.rows;
    outputFormat.m_ui32Width = outputImage.cols;
    outputFormat.m_szMaxByteSize = outputImage.cols * outputImage.rows * outputImage.channels();

    if (!keepFormat)
    {
        if (outputImage.channels() == 1)
        {
            outputFormat.m_strFormatName = ADTF_IMAGE_FORMAT(GREYSCALE_8);
        }
        if (outputImage.channels() == 3)
        {
            outputFormat.m_strFormatName = ADTF_IMAGE_FORMAT(BGR_24);
        }
    }
    outputFormat.m_ui8DataEndianess = PLATFORM_BYTEORDER;

    adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeOutput, outputFormat);

    writer << pTypeOutput;
}

// writeMatToPin
// this function will output our image
void LaserPointerFollower::writeMatToPin(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tTimeStamp streamTime)
{
    //create write buffer
    object_ptr<ISample> pWriteSample;

    if (IS_OK(alloc_sample(pWriteSample, streamTime)))
    {
        object_ptr_locked<ISampleBuffer> pWriteBuffer;
        if (IS_OK(pWriteSample->WriteLock(pWriteBuffer, outputImage.cols * outputImage.rows * outputImage.channels())))
        {
            pWriteBuffer->Write(adtf_memory_buffer<void, tSize>((void*) outputImage.data,
                outputImage.cols * outputImage.rows * outputImage.channels()));
        }
    }

    writer << pWriteSample << flush << trigger;
}

// ChangeType
// we need this for video inputs
tResult LaserPointerFollower::ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
    const adtf::streaming::ant::IStreamType& oType)
{
    if (oType == adtf::streaming::stream_meta_type_image())
    {
        adtf::ucom::object_ptr<const adtf::streaming::IStreamType> pTypeInput;
        // get pType from input reader
        inputPin >> pTypeInput;
        adtf::streaming::get_stream_type_image_format(m_sInputImageFormat, *pTypeInput);

        //set also output format
        adtf::streaming::get_stream_type_image_format(m_sOutputImageFormat, *pTypeInput);
        //we always have a grayscale output image
        m_sOutputImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
        // and set pType also to samplewriter
        m_WriterDebugBirdVideo     << pTypeInput;
    }
    else
    {
        RETURN_ERROR(ERR_INVALID_TYPE);
    }

    RETURN_NOERROR;
}
