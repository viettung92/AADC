#include "aadc_transmit_samples.h"


//// transmitTSignalValue
//tResult transmitTSignalValue(cPinWriter& outputPin, iSignalValue &index,
//                            adtf::mediadescription::cSampleCodecFactory& factory,
//                            tSignalValue &value,
//                            tTimeStamp streamTime)
//{
//    object_ptr<ISample> pWriteSample;
//    tResult res;

//    if (IS_OK(res = alloc_sample(pWriteSample, streamTime)))
//    {
//        auto oCodec = factory.MakeCodecFor(pWriteSample);

//        if(IS_FAILED(res = oCodec.IsValid()))
//        {
//            LOG_WARNING("CODEC CREATION FAILED!");
//            RETURN_ERROR(res);
//        }

//        if(IS_FAILED(res = oCodec.SetElementValue(index.f32Value, value.f32Value)))
//        {
//            LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: tSignalValue.f32Value for index %d", index.f32Value));
//            RETURN_ERROR(res);
//        }
//        if(IS_FAILED(res = oCodec.SetElementValue(index.ui32ArduinoTimestamp, value.ui32ArduinoTimestamp)))
//        {

//            LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: tSignalValue.ui32ArduinoTimestamp for index %d", index.ui32ArduinoTimestamp));
//            RETURN_ERROR(res);
//        }
//    }
//    else
//    {
//         LOG_WARNING("transmitTSignalValue: alloc_sample did not work!");
//         RETURN_ERROR(res);

//    }

//    outputPin << pWriteSample << flush << trigger;

//    RETURN_NOERROR;
//}

//// transmitTBoolSignalValue
//tResult transmitTBoolSignalValue(cPinWriter& outputPin, iBoolSignalValue &index,
//                            adtf::mediadescription::cSampleCodecFactory& factory,
//                            tBoolSignalValue &value,
//                            tTimeStamp streamTime)
//{
//    object_ptr<ISample> pWriteSample;
//    tResult res;

//    if (IS_OK(res = alloc_sample(pWriteSample, streamTime)))
//    {
//        auto oCodec = factory.MakeCodecFor(pWriteSample);

//        if(IS_FAILED(res = oCodec.IsValid()))
//        {
//            LOG_WARNING("CODEC CREATION FAILED!");
//            RETURN_ERROR(res);
//        }

//        if(IS_FAILED(res = oCodec.SetElementValue(index.bValue, value.bValue)))
//        {
//            LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: tBoolSignalValue.bValue for index %d", index.bValue));
//            RETURN_ERROR(res);
//        }
//        if(IS_FAILED(res = oCodec.SetElementValue(index.ui32ArduinoTimestamp, value.ui32ArduinoTimestamp)))
//        {

//            LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: tBoolSignalValue.ui32ArduinoTimestamp for index %d", index.ui32ArduinoTimestamp));
//            RETURN_ERROR(res);
//        }
//    }
//    else
//    {

//         LOG_WARNING("transmitTSignalValue: alloc_sample did not work!");
//         RETURN_ERROR(res);

//    }

//    outputPin << pWriteSample << flush << trigger;

//    RETURN_NOERROR;
//}

//// transmitTActionStruct
//tResult transmitTActionStruct(cPinWriter& outputPin, iActionStruct &index,
//                            adtf::mediadescription::cSampleCodecFactory& factory,
//                            tActionStruct &value)
//{
//    object_ptr<ISample> pWriteSample;
//    tResult res;

//    if (IS_OK(res = alloc_sample(pWriteSample)))
//    {
//        auto oCodec = factory.MakeCodecFor(pWriteSample);

//        if(IS_FAILED(res = oCodec.IsValid()))
//        {
//            LOG_WARNING("CODEC CREATION FAILED!");
//            RETURN_ERROR(res);
//        }

//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action1.ui8FilterId          , value.action1.ui8FilterId          ));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action1.subAction.bEnabled   , value.action1.subAction.bEnabled   ));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action1.subAction.bStarted   , value.action1.subAction.bStarted   ));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action1.subAction.ui32Command, value.action1.subAction.ui32Command));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action2.ui8FilterId          , value.action2.ui8FilterId          ));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action2.subAction.bEnabled   , value.action2.subAction.bEnabled   ));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action2.subAction.bStarted   , value.action2.subAction.bStarted   ));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action2.subAction.ui32Command, value.action2.subAction.ui32Command));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action3.ui8FilterId          , value.action3.ui8FilterId          ));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action3.subAction.bEnabled   , value.action3.subAction.bEnabled   ));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action3.subAction.bStarted   , value.action3.subAction.bStarted   ));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action3.subAction.ui32Command, value.action3.subAction.ui32Command));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action4.ui8FilterId          , value.action4.ui8FilterId          ));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action4.subAction.bEnabled   , value.action4.subAction.bEnabled   ));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action4.subAction.bStarted   , value.action4.subAction.bStarted   ));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action4.subAction.ui32Command, value.action4.subAction.ui32Command));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action5.ui8FilterId          , value.action5.ui8FilterId          ));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action5.subAction.bEnabled   , value.action5.subAction.bEnabled   ));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action5.subAction.bStarted   , value.action5.subAction.bStarted   ));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.action5.subAction.ui32Command, value.action5.subAction.ui32Command));
//    }
//    else
//    {

//         LOG_WARNING("transmitTActionStructValue: alloc_sample did not work!");
//         RETURN_ERROR(res);

//     }

//    outputPin << pWriteSample << flush << trigger;

//    RETURN_NOERROR;
//}

//// transmitTAction
//tResult transmitTActionValue(cPinWriter& outputPin, iAction &index,
//                            adtf::mediadescription::cSampleCodecFactory& factory,
//                            tAction &value)
//{
//    object_ptr<ISample> pWriteSample;
//    tResult res;

//    if (IS_OK(res = alloc_sample(pWriteSample)))
//    {
//        auto oCodec = factory.MakeCodecFor(pWriteSample);

//        if(IS_FAILED(res = oCodec.IsValid()))
//        {
//            LOG_WARNING("CODEC CREATION FAILED!");
//            RETURN_ERROR(res);
//        }

//        if(IS_FAILED(res = oCodec.SetElementValue(index.ui8FilterId          , value.ui8FilterId          )))
//        {
//			LOG_WARNING("transmitTActionValue: set ui8FilterId did not work!");
//			RETURN_ERROR(res);
			
//		}
//        if(IS_FAILED(res = (oCodec.SetElementValue(index.subAction.ui32Command   , value.subAction.ui32Command   ))))
//        {
//			LOG_WARNING("transmitTActionValue: set subAction.ui32Command did not work!");
//			RETURN_ERROR(res);
			
//		}
//        if(IS_FAILED(res = (oCodec.SetElementValue(index.subAction.bEnabled   , value.subAction.bEnabled   ))))
//        {
//			LOG_WARNING("transmitTActionValue: set subAction.bEnabled did not work!");
//			RETURN_ERROR(res);
			
//		}
//        if(IS_FAILED(res = (oCodec.SetElementValue(index.subAction.bStarted   , value.subAction.bStarted   ))))
//        {
//			LOG_WARNING("transmitTActionValue: set subAction.bStarted did not work!");
//			RETURN_ERROR(res);
			
//		}
//	}
//    else
//    {

//         LOG_WARNING("transmitTActionValue: alloc_sample did not work!");
//         RETURN_ERROR(res);

//     }

//    outputPin << pWriteSample << flush << trigger;

//    RETURN_NOERROR;
//}



//// transmitTFeedback
//tResult transmitTFeedbackValue(cPinWriter& outputPin, iFeedbackStruct &index,
//                            adtf::mediadescription::cSampleCodecFactory& factory,
//                            tFeedbackStruct &value,
//                            tTimeStamp streamTime){
								
//	object_ptr<ISample> pWriteSample;
//    tResult res;

//    if (IS_OK(res = alloc_sample(pWriteSample)))
//    {
//        auto oCodec = factory.MakeCodecFor(pWriteSample);

//        if(IS_FAILED(res = oCodec.IsValid()))
//        {
//            LOG_WARNING("CODEC CREATION FAILED!");
//            RETURN_ERROR(res);
//        }

//        RETURN_IF_FAILED(oCodec.SetElementValue(index.ui8FilterId          , value.ui8FilterId          ));
//        RETURN_IF_FAILED(oCodec.SetElementValue(index.ui32FeedbackStatus   , value.ui32FeedbackStatus   ));
//	}
//    else
//    {

//         LOG_WARNING("transmitTActionValue: alloc_sample did not work!");
//         RETURN_ERROR(res);

//     }

//    outputPin << pWriteSample << flush << trigger;

//    RETURN_NOERROR;
								
//							}
