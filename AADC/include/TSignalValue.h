/**
 * Copyright (c)
 * Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
 * 4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************
 * $Author:: schoen $   $Date:: 2016-01-30 #$
 **********************************************************************
 * $Adapted:: Xiangfei $   $Date:: 2018-08-01 #$ status: not tested
 **********************************************************************/

#ifndef T_SIGNALVALUE_H_
#define T_SIGNALVALUE_H_



using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;



class TSignalValue {

	tInt size;

	struct IDs {
		tBool set;

                tSize f32Value;
                tSize ui32ArduinoTimestamp;
	} ids;

//	ucom::cObjectPtr<IMediaTypeDescription> description;
//	ucom::cObjectPtr<IMediaType> type;

	tBool stageFirstCalled;

public:

	struct Data {
                tUInt32 ui32ArduinoTimestamp;
                tFloat32 f32Value;

		Data() {
                        f32Value = 0;
                        ui32ArduinoTimestamp = 0;
		}
	};



        adtf::mediadescription::cSampleCodecFactory m_TSignalValueSampleFactory;



	TSignalValue() {
                stageFirstCalled = tFalse;
                ids.f32Value = 0;
                ids.ui32ArduinoTimestamp = 0;
                // -------------------------------
                // create pointers for streamtypes
                    object_ptr<IStreamType> pTSignalValueDataType;


                    // (user from 2017 defined)  struct
                    if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTSignalValueDataType, m_TSignalValueSampleFactory)))
                    {
                        // if loading was successful then get index from aadc user description file
                        adtf_ddl::access_element::find_index(m_TSignalValueSampleFactory, cString("f32Value"), ids.f32Value);
                        adtf_ddl::access_element::find_index(m_TSignalValueSampleFactory, cString("ui32ArduinoTimestamp"), ids.ui32ArduinoTimestamp);


                        ids.set = tTrue;
                        stageFirstCalled = tTrue;
                    }
                    else
                    {
                        // did not find datatype in aadc user description file
                        LOG_INFO("no mediadescription for tSignalValue found! :(");
                        ids.set = tFalse;

                    }

	}

//	tResult StageFirst( __exception) {
//		ucom::cObjectPtr<IMediaDescriptionManager> pDescManager;
//		RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

//		//get description for tSignalValue
//		tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
//		RETURN_IF_POINTER_NULL(strDescSignalValue);

//		type = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
//		RETURN_IF_POINTER_NULL(type);

//		//get mediatype description for tSignalValue data type
//		RETURN_IF_FAILED(type->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&description));

//		stageFirstCalled = tTrue;

//		RETURN_NOERROR;
//	}

//	tResult StageGraphReady() {
//		ids.set = tFalse;

//		//allocate memory with the size given by the descriptor tPoseStruct
//		cObjectPtr<IMediaSerializer> serializer;
//		description->GetMediaSampleSerializer(&serializer);
//		size = serializer->GetDeserializedSize();

//		RETURN_NOERROR;
//	}

//	ucom::cObjectPtr<IMediaType> GetMediaType() {
//		return type;
//	}

        tResult Read(Data *data) {
                object_ptr<const ISample> pReadInputTSignalValueSample;

                // decode last sample
                auto oDecoder = m_TSignalValueSampleFactory.MakeDecoderFor(*pReadInputTSignalValueSample);
                //RETURN_IF_FAILED(oDecoder.IsValid());



                // get the IDs for the items in the media sample
                if (ids.set) {

                // get values
                        oDecoder.GetElementValue(ids.f32Value                   , &(data->f32Value              ));
                        oDecoder.GetElementValue(ids.ui32ArduinoTimestamp       , &(data->ui32ArduinoTimestamp  ));

                        RETURN_NOERROR;
                } else{
                    LOG_INFO("Ids are not set! :(");
                }

//		ucom::cObjectPtr<adtf::IMediaCoderExt> pCoderInput;
//		adtf::cScopedMediaDescriptionReadLock descriptionReadLock(description, mediaSample, &pCoderInput);

//		if(pCoderInput == 0) {
//			LOG_WARNING (cString::Format("TSignalValue: pCoderInput is nullpointer"));
//			RETURN_ERROR(ERR_POINTER);
//		}

//		// get the IDs for the items in the media sample
//		if (!ids.set) {
//                        pCoderInput->GetID("f32Value", ids.f32Value);
//                        pCoderInput->GetID("ui32ArduinoTimestamp", ids.ui32ArduinoTimestamp);
//			ids.set = tTrue;
//		}

//		//get values from media sample
//                pCoderInput->Get(ids.f32Value, (tVoid*) &(data->f32Value));
//                pCoderInput->Get(ids.ui32ArduinoTimestamp,(tVoid*) &(data->ui32ArduinoTimestamp));

//		RETURN_NOERROR;
	}

//	tResult Transmit(cOutputPin *outputPin, Data data, tTimeStamp outputTime) {
        tResult Transmit(cPinWriter* outputPin, Data data) {


            object_ptr<ISample> pWriteSample;
            // allocate for our sample
            RETURN_IF_FAILED(alloc_sample(pWriteSample));

            // create coding for signal
            auto codec = m_TSignalValueSampleFactory.MakeCodecFor(pWriteSample);

            // set values
            RETURN_IF_FAILED(codec.SetElementValue(ids.f32Value     , &(data.f32Value)     ));
            RETURN_IF_FAILED(codec.SetElementValue(ids.ui32ArduinoTimestamp     , &(data.ui32ArduinoTimestamp)     ));

            // write to sample
            *outputPin << pWriteSample << flush << trigger;

            // done
            RETURN_NOERROR;




//		if(size == 0 || !stageFirstCalled) {
//			RETURN_AND_LOG_ERROR_STR(ERR_NOT_INITIALISED, cString::Format("TSignalValue: Transmit failed on pin %s due to size not set or not initialized", outputPin->GetName()));
//		}

//		ucom::cObjectPtr<IMediaSample> newMediaSample;
//		if (IS_OK(cMediaAllocHelper::AllocMediaSample((tVoid** )&newMediaSample))) {
//			newMediaSample->AllocBuffer(size);

//			{
//				ucom::cObjectPtr<adtf::IMediaCoderExt> pCoderOutput;
//				adtf::cScopedMediaDescriptionReadLock descriptionWriteLock(description, newMediaSample, &pCoderOutput);

//				if(pCoderOutput == 0) {
//					LOG_WARNING (cString::Format("TSignalValue::Transmit pCoderInput is nullpointer"));
//					RETURN_ERROR(ERR_POINTER);
//				}

//				// get the IDs for the items in the media sample
//				if (!ids.set) {
//                                        pCoderOutput->GetID("f32Value", ids.f32Value);
//                                        pCoderOutput->GetID("ui32ArduinoTimestamp", ids.ui32ArduinoTimestamp);
//					ids.set = tTrue;
//				}

//				// set values in new media sample
//                                pCoderOutput->Set(ids.f32Value, &data.f32Value);
//                                pCoderOutput->Set(ids.ui32ArduinoTimestamp, &data.ui32ArduinoTimestamp);
//			}

//			//transmit media sample over output pin
//			newMediaSample->SetTime(outputTime);
//			outputPin->Transmit(newMediaSample);
//		}

//		RETURN_NOERROR;
	}

//        tResult Transmit(cOutputPin *outputPin, tFloat32 f32Value, tUInt32 ui32ArduinoTimestamp, tTimeStamp outputTime) {
//		TSignalValue::Data data;
//                data.f32Value = f32Value;
//                data.ui32ArduinoTimestamp = ui32ArduinoTimestamp;
//		return Transmit(outputPin, data, outputTime);
//	}

};

#endif // T_SIGNALVALUE_H_
