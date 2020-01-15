/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. Team FAUtonomous. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

This is a helper Header, which will transmit Samples for you! Awesome :)

**********************************************************************/

#pragma once
#include <iomanip>      // std::setw


#include <adtf_filtersdk.h>




//using namespace adtf_util;
//using namespace ddl;
//using namespace adtf::ucom;
//using namespace adtf::base;
//using namespace adtf::streaming;
//using namespace adtf::mediadescription;
//using namespace adtf::filter;
//using namespace adtf::filter::ant;


//#include <aadc_structs.h>
//#include <aadc_user_structs.h>
//#include <aadc_create_mediadescription.h>
//#include <aadc_read_samples.h>

//// transmitTSignalValue
//tResult transmitTSignalValue(cPinWriter& outputPin, iSignalValue &index,
//                            adtf::mediadescription::cSampleCodecFactory& factory,
//                            tSignalValue &value,
//                            tTimeStamp streamTime);
//// transmitTBoolSignalValue
//tResult transmitTBoolSignalValue(cPinWriter& outputPin, iBoolSignalValue &index,
//                            adtf::mediadescription::cSampleCodecFactory& factory,
//                            tBoolSignalValue &value,
//                            tTimeStamp streamTime);

//// transmitTActionStruct
//tResult transmitTActionStruct(cPinWriter& outputPin, iActionStruct &index,
//                            adtf::mediadescription::cSampleCodecFactory& factory,
//                            tActionStruct &value);
//// transmitTAction
//tResult transmitTActionValue(cPinWriter& outputPin, iAction &index,
//                            adtf::mediadescription::cSampleCodecFactory& factory,
//                            tAction &value);

//// transmitTFeedback
//tResult transmitTFeedbackValue(cPinWriter& outputPin, iFeedbackStruct &index,
//                            adtf::mediadescription::cSampleCodecFactory& factory,
//                            tFeedbackStruct &value,
//                            tTimeStamp streamTime);
