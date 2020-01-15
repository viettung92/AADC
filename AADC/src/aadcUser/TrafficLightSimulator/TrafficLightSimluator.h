/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. Team FAUtonomous. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#pragma once
#define CID_TRAFFICLIGHTSIMULATOR_FILTER "traffic_light_simluator.filter.user.aadc.cid"

#include <boost/thread.hpp>
// namespaces
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

#define RED_THR  -10
#define GREEN_THR  10
#define RED -1
#define GREEN 1
#define NONE 0
#define X 250
#define Y 500
#define W 530
#define H 130
#define DEBUG_ME
// LaserPointerFollower
class TrafficLightSimluator : public cTriggerFunction
{
private:

    int state = 0;
    // ------- PINS -------------
    // input
    cPinReader m_ReaderVideo;

    // output

    cPinWriter m_WriterDebugBirdVideo;
    cPinWriter m_WriterDebugMaskVideo;
    cPinWriter feedback_struct_output;


    // The clock
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    // image formats for video pins
    adtf::streaming::tStreamImageFormat m_sInputImageFormat;
    adtf::streaming::tStreamImageFormat m_sOutputImageFormat;


    // steering value
    TSignalValue    m_SteeringId;
    TFeedbackStruct feedback_struct;

    // transformed image after bird's eye view transform
    Mat m_RedImage;
    Mat m_GreenImage;
    vector<Vec4i> green_lines;
    vector<Vec4i> red_lines;
    Mat element = getStructuringElement(MORPH_RECT,
                                        Size(5, 5),
                                        Point(3, 3));
    Mat eelement = getStructuringElement(MORPH_RECT,
                                        Size(3, 3),
                                        Point(1, 1));
    // input image
    Mat m_MatInputImage;

    // check free lane
    bool m_bFreeRois[10] = {false};

    adtf::base::property_variable<tUInt32> m_propFrame = 2;

    tUInt32 m_ui32FrameCounter;

    boost::mutex cs_TransmitFeedback;
public:
    /*------------ FUNCTIONS -----------*/
    /*------------ FROM AUDI -----------*/
    // constructor
    TrafficLightSimluator();

    // destructor
    ~TrafficLightSimluator() = default;

    // Configure
    virtual tResult Configure() override;

    // Process
    virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;


    // setTypeFromMat
    void setTypeFromMat(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tBool keepFormat = false);

    // writeMatToPin
    void writeMatToPin(adtf::streaming::cSampleWriter& writer,
                                           const cv::Mat& outputImage, tTimeStamp streamTime);

    tResult TransmitFeedback(tUInt32 feedback_num);
    // ChangeType
    tResult ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
        const adtf::streaming::ant::IStreamType& oType);

}; // LaserPointerFollower
