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
#define CID_TEMPLATE_FILTER_FILTER "template_filter.filter.user.aadc.cid"

// namespaces
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

// cTemplateFilter
class cTemplateFilter : public cTriggerFunction
{
private:

    /*------------ Index STRUCTS -------------*/
    // see in aadc_dir/include/aadc_{user_}structs.h
    // coding convention: 	o_******Id
    

    TSignalValue o_SpeedSignalId;


    /*------- SAMPLE FACTORIES ---------*/
    // a factory contains all data samples, which we can load into a tSignalValue
    // plus, it is responsible for (de)code the samples
    // coding convention:	m_******SampleFactory

/// !! SAMPLE FACTORIES here not needed anymore (done by our classes..) - EDIT BY NICO !!

    /*-------------- PINS --------------*/
    // create pins for input and output
    // coding convention:	m_Reader******	for input  signals
    //			m_Writer******	for output signals

    // input pins
    cPinReader m_ReaderSpeed;

    // output pins
    cPinWriter m_WriterSpeed;


    /*------------ PROPERTIES ----------*/
    // set property variables
    // coding convention:	m_prop******

    adtf::base::property_variable<tFloat32> m_propBinhsCoolnessFactor = tFloat(100.0f);


    /*------------ VARIABLES -----------*/
    // coding convention:	m_******
    tBool   m_bIsBinhCool;
    tUInt32 m_ui32BinhIsVeryCool;
	TSignalValue m_SpeedSignalId;

    /*! The reference clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

public:
    /*------------ FUNCTIONS -----------*/
    /*------------ FROM AUDI -----------*/
    // constructor
    cTemplateFilter();

    // destructor
    ~cTemplateFilter() = default;

    // Configure
    virtual tResult Configure() override;

    // Process
    virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;

    // ---------- FROM USER ------------*/
    // TransmitSpeed
    tResult TransmitSpeed(TSignalValue::Data inputSignal);

}; // cTemplateFilter
