#include "aadc_read_samples.h"

// getLastSampleTSignalValue
// This function will read last sample from reader pin and write it to value
//tResult getLastSampleTSignalValue(cPinReader &input,
//                                  iSignalValue &index,
//                                  adtf::mediadescription::cSampleCodecFactory &factory,
//                                  tSignalValue &value)
//{
//    tResult res;
//    object_ptr<const ISample> sample;

//    if (IS_OK(res = input.GetLastSample(sample)))
//    {
//        auto oDecoder = factory.MakeDecoderFor(*sample);

//        if(IS_FAILED(res = oDecoder.IsValid()))
//        {
//            LOG_WARNING("DECODER CREATION FAILED!");
//            RETURN_ERROR(res);
//        }

//        // retrieve the values (using convenience methods that return a variant)
//        if(IS_FAILED(res = oDecoder.GetElementValue(index.f32Value, &(value.f32Value))))
//        {
//            LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: tSignalValue.f32Value for index %d", index.f32Value));
//            RETURN_ERROR(res);
//        }
//        if(IS_FAILED(res = oDecoder.GetElementValue(index.ui32ArduinoTimestamp, &(value.ui32ArduinoTimestamp))))
//        {
//            LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: tSignalValue.ui32ArduinoTimestamp for index %d", index.ui32ArduinoTimestamp));
//            RETURN_ERROR(res);
//        }


//    }
//    else
//    {
//        LOG_WARNING("getLastSampleTSignalValue: getLastSample did not work!");
//        RETURN_ERROR(res);
//    }

//    // done
//    RETURN_NOERROR;
//}


//// getLastSampleTActionStruct
//// This function will read last sample from reader pin and write it to value
//tResult getLastSampleTActionStruct(cPinReader &input,
//                                  iActionStruct &index,
//                                  adtf::mediadescription::cSampleCodecFactory &factory,
//                                  tActionStruct &value)
//{
//    // -----------------------------------------
//    // create pointer for samples
//    object_ptr<const ISample> pReadActionSample;

//    // -----------------------------------------
//    // get the last speed sample
//    if(IS_OK(input.GetLastSample(pReadActionSample)))
//    {
//        // decode last sample
//        auto oDecoder = factory.MakeDecoderFor(*pReadActionSample);
//        RETURN_IF_FAILED(oDecoder.IsValid());

//        // get values
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action1.ui8FilterId          , &(value.action1.ui8FilterId          )));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action1.subAction.bEnabled   , &(value.action1.subAction.bEnabled   )));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action1.subAction.bStarted   , &(value.action1.subAction.bStarted   )));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action1.subAction.ui32Command, &(value.action1.subAction.ui32Command)));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action2.ui8FilterId          , &(value.action2.ui8FilterId          )));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action2.subAction.bEnabled   , &(value.action2.subAction.bEnabled   )));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action2.subAction.bStarted   , &(value.action2.subAction.bStarted   )));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action2.subAction.ui32Command, &(value.action2.subAction.ui32Command)));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action3.ui8FilterId          , &(value.action3.ui8FilterId          )));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action3.subAction.bEnabled   , &(value.action3.subAction.bEnabled   )));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action3.subAction.bStarted   , &(value.action3.subAction.bStarted   )));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action3.subAction.ui32Command, &(value.action3.subAction.ui32Command)));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action4.ui8FilterId          , &(value.action4.ui8FilterId          )));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action4.subAction.bEnabled   , &(value.action4.subAction.bEnabled   )));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action4.subAction.bStarted   , &(value.action4.subAction.bStarted   )));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action4.subAction.ui32Command, &(value.action4.subAction.ui32Command)));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action5.ui8FilterId          , &(value.action5.ui8FilterId          )));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action5.subAction.bEnabled   , &(value.action5.subAction.bEnabled   )));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action5.subAction.bStarted   , &(value.action5.subAction.bStarted   )));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(index.action5.subAction.ui32Command, &(value.action5.subAction.ui32Command)));
//    }

//    // done
//    RETURN_NOERROR;
//}

//// readTActionStruct
//// this helper function will read a tActionStruct and check, if there is a actionSub for specific filter with filterId
//// if not, it will return 0!
//tActionSub readTActionStruct(tUInt8 inputFilterId, tActionStruct inputActionStruct)
//{
//    tActionSub result;

//    // check if filterId is present
//    if(inputActionStruct.action1.ui8FilterId == inputFilterId)
//    {
//        result = inputActionStruct.action1.subAction;
//        return result;
//    }
//    if(inputActionStruct.action2.ui8FilterId == inputFilterId)
//    {
//        result = inputActionStruct.action2.subAction;
//        return result;
//    }
//    if(inputActionStruct.action3.ui8FilterId == inputFilterId)
//    {
//        result = inputActionStruct.action3.subAction;
//        return result;
//    }
//    if(inputActionStruct.action4.ui8FilterId == inputFilterId)
//    {
//        result = inputActionStruct.action4.subAction;
//        return result;
//    }
//    if(inputActionStruct.action5.ui8FilterId == inputFilterId)
//    {
//        result = inputActionStruct.action5.subAction;
//        return result;
//    }

//    result.bEnabled    = false;
//    result.bStarted    = false;
//    result.ui32Command = 0;

//    return result;
//}


//// getLastSampleTAction
//// This function will read last sample from reader pin and write it to value
//tResult getLastSampleTAction(cPinReader &input,
//                                  iAction &index,
//                                  adtf::mediadescription::cSampleCodecFactory &factory,
//                                  tAction &value)
//{
//    // -----------------------------------------
//    // create pointer for samples
//    object_ptr<const ISample> pReadActionSample;
//    tResult res;

//    // -----------------------------------------
//    // get the last speed sample
//    if(IS_OK(input.GetLastSample(pReadActionSample)))
//    {
//        // decode last sample
//        auto oDecoder = factory.MakeDecoderFor(*pReadActionSample);
//        RETURN_IF_FAILED(oDecoder.IsValid());

//        // get values
//        if(IS_FAILED(res = oDecoder.GetElementValue(index.ui8FilterId          , &(value.ui8FilterId          ))))
//        {
//			LOG_WARNING("getLastSampleTAction: get ui8FilterId did not work!");
//			RETURN_ERROR(res);
			
//		}
//        if(IS_FAILED(res = oDecoder.GetElementValue(index.subAction.bEnabled   , &(value.subAction.bEnabled   ))))
//        {
//			LOG_WARNING("getLastSampleTAction: get subAction.bEnabled did not work!");
//			RETURN_ERROR(res);
			
//		}
//        if(IS_FAILED(res = oDecoder.GetElementValue(index.subAction.bStarted   , &(value.subAction.bStarted   ))))
//        {
//			LOG_WARNING("getLastSampleTAction: get subAction.bStarted did not work!");
//			RETURN_ERROR(res);
			
//		}
//        if(IS_FAILED(res = oDecoder.GetElementValue(index.subAction.ui32Command, &(value.subAction.ui32Command))))
//        {
//			LOG_WARNING("getLastSampleTAction: get subAction.ui32Command did not work!");
//			RETURN_ERROR(res);
			
//		}
//    }

//    // done
//    RETURN_NOERROR;
//}
