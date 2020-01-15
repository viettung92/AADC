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
#include "TrafficLightSimluator.h"
#include <common_helper.h>
#include <property_structs.h>
#include "ScmCommunication.h"
#include <cmath>
#include "tinyxml2.h"


// This will define the filter and expose it via plugin class factory.
// Class cTemplateFilter will be embedded to the filter and called by
// data trigger of "speed" by default.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
    CID_TRAFFICLIGHTSIMULATOR_FILTER,		// references to header file
    "TrafficLightSimluator",              // label
    TrafficLightSimluator,                // class
    adtf::filter::pin_trigger({"inputVideo"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
TrafficLightSimluator::TrafficLightSimluator()
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
    Register(m_WriterDebugMaskVideo, "m_WriterDebugMaskVideo"   , pTypeOutput);

    feedback_struct.registerPin(this, feedback_struct_output, "feedback_struct_output");

    //register callback for type changes
    m_ReaderVideo.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pVideoDataType) -> tResult
    {
        return ChangeType(m_ReaderVideo, *pVideoDataType.Get());
    });

    RegisterPropertyVariable("Frames", m_propFrame);
    m_propFrame = 2;
    m_ui32FrameCounter = 0;
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult TrafficLightSimluator::Configure()
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
tResult TrafficLightSimluator::Process(tTimeStamp)
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
                m_ui32FrameCounter = 0;
                // create a opencv matrix from the media sample buffer
                Mat inputImage (cv::Size(m_sInputImageFormat.m_ui32Width, m_sInputImageFormat.m_ui32Height),
                                CV_8UC3, (uchar*)pReadBuffer->GetPtr());


                Mat mask_red (cv::Size(m_sInputImageFormat.m_ui32Width, m_sInputImageFormat.m_ui32Height),
                                CV_8UC3, Scalar(0, 0, 0));
                Mat mask_green (cv::Size(m_sInputImageFormat.m_ui32Width, m_sInputImageFormat.m_ui32Height),
                                CV_8UC3, Scalar(0, 0, 0));
                // set our global mat to input image
                m_MatInputImage       = inputImage;
                m_GreenImage = m_MatInputImage.clone();
                m_RedImage = m_MatInputImage.clone();

                //cvtColor(m_MatTransformedImage, m_MatTransformedImage, CV_RGB2HSV);
               // inRange(m_MatTransformedImage, Scalar(167, 184, 0), Scalar(235, 253, 55), mask);
              //  inRange(m_MatTransformedImage, Scalar(0, 184, 167), Scalar(55, 253, 235), mask);

                //BGR
                 inRange(m_GreenImage, Scalar(0, 184, 167), Scalar(55, 253, 235), mask_green);
               //  inRange(m_RedImage, Scalar(0x4e, 0x03, 0xa6), Scalar(0x61,0x41,0xff), mask_red);



               //  inRange(m_GreenImage, Scalar(167, 184, 0), Scalar(235, 253, 55), mask_green);
                 inRange(m_RedImage, Scalar(0xa6, 0x03, 0x4e), Scalar(0xff,0x41,0x61), mask_red);



                // Dilation to get better edges

                // Apply the dilation operation -- against random pixels
                //erode(mask, mask, eelement);
                dilate(mask_red, mask_red, element);
                dilate(mask_green, mask_green, element);

                Rect region_of_interest = Rect(X, Y, W, H);
                mask_red = mask_red(region_of_interest);
                mask_green = mask_green(region_of_interest);

                HoughLinesP( mask_green, green_lines, 1, CV_PI/180, 80, 30, 10 );
                HoughLinesP( mask_red, red_lines, 1, CV_PI/180, 80, 30, 10 );

                if(green_lines.size() > red_lines.size()){
                    if(state < 0) state = 0;
                    state += GREEN;
                } else if(red_lines.size() > green_lines.size()){
                    if(state > 0) state = 0;

                    state += RED;

                } else {
                    if  (state < 0) ++state;
                    else if(state > 0) --state;

                }
                for( size_t i = 0; i < green_lines.size(); i++ ){
                    line(m_GreenImage, Point(green_lines[i][0]+X, green_lines[i][1]+Y),
                    Point(green_lines[i][2]+X, green_lines[i][3]+Y), Scalar(122,0,255), 3, 8 );
                }
                for( size_t i = 0; i < red_lines.size(); i++ ){
                    line(m_RedImage, Point(red_lines[i][0]+X, red_lines[i][1]+Y),
                    Point(red_lines[i][2]+X, red_lines[i][3]+Y), Scalar(122,0,255), 3, 8 );
                }
                rectangle(m_GreenImage,Point(X,Y),Point(X+W,Y+H),Scalar(255,255,255),3);
                rectangle(m_RedImage,Point(X,Y),Point(X+W,Y+H),Scalar(255,255,255),3);
                //check for red: TODO
                //if(RED) do nothing
                //else TransmitFeedback(FB_TRAFFIC_LIGHT_GREEN);
                Mat mask, dst;
                vconcat(mask_red, mask_green, mask);
                vconcat(m_RedImage, m_GreenImage, dst);



                if (!mask.empty()){

                    if (mask.total() * mask.elemSize() != m_sOutputImageFormat.m_szMaxByteSize)
                    {
                        setTypeFromMat(m_WriterDebugMaskVideo, mask);
                    }
                    // write to pin
                    writeMatToPin(m_WriterDebugMaskVideo, mask, m_pClock->GetStreamTime());
                }

                if (!dst.empty())
                {
                    //update output format if matrix size does not fit to
                    if (dst.total() * dst.elemSize() != m_sOutputImageFormat.m_szMaxByteSize)
                    {
                        setTypeFromMat(m_WriterDebugBirdVideo, dst);
                    }

                   writeMatToPin(m_WriterDebugBirdVideo, dst, m_pClock->GetStreamTime());
                }
            }

            m_ui32FrameCounter++;
        }
        // output//Write processed Image to Output Pin


    }

    if(state >= GREEN_THR){
        state = 0;
        TransmitFeedback(FB_TRAFFIC_LIGHT_GREEN);
        LOG_SUCCESS("GREEN");
    } else if( state <= RED_THR){
        state = 0;
        LOG_ERROR("RED");
        TransmitFeedback(FB_TRAFFIC_LIGHT_RED);
    }

    RETURN_NOERROR;
}


tResult TrafficLightSimluator::TransmitFeedback(tUInt32 feedback_num){
    boost::lock_guard<boost::mutex> lock(cs_TransmitFeedback);
    TFeedbackStruct::Data feedback;
    feedback.ui8FilterId = F_TRAFFIC_LIGHT_SIMLUATOR;
    feedback.ui32FeedbackStatus = feedback_num;

    return feedback_struct.writePin(feedback_struct_output, (void *) &feedback, m_pClock->GetStreamTime());
}



// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

// ------------- Functions for VIDEO stuff
// setTypeFromMat
// we need this for video outputs
void TrafficLightSimluator::setTypeFromMat(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tBool keepFormat)
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
            outputFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
        }
    }
    outputFormat.m_ui8DataEndianess = PLATFORM_BYTEORDER;

    adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeOutput, outputFormat);

    writer << pTypeOutput;
}

// writeMatToPin
// this function will output our image
void TrafficLightSimluator::writeMatToPin(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tTimeStamp streamTime)
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
tResult TrafficLightSimluator::ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
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
        m_WriterDebugMaskVideo     << pTypeInput;
    }
    else
    {
        RETURN_ERROR(ERR_INVALID_TYPE);
    }

    RETURN_NOERROR;
}
