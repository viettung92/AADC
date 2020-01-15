
#include "aadc_create_mediadescription.h"

//// createTBoolSignalValueMediaDescription
//// this helper function will create media description of tboolsignalvalue for you !
//tResult createTBoolSignalValueMediaDescription(object_ptr<IStreamType> &op, adtf::mediadescription::cSampleCodecFactory &factory, iBoolSignalValue &value, const char* type_name)
//{
//    // get streamtype from aadc.description file
//    if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", op, factory)))
//    {
//        // get value
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("bValue"), value.bValue)))
//        {
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.bValue", type_name));
//        }
//        else
//        {
//            LOG_SUCCESS(cString::Format("FOUND INDEX: %s.bValue = %d", type_name, value.bValue));
//        }

//        // get timestamp
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui32ArduinoTimestamp"), value.ui32ArduinoTimestamp)))
//        {
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui32Timestamp", type_name));
//        }
//        else
//        {
//            LOG_SUCCESS(cString::Format("FOUND INDEX: %s.ui32ArduinoTimestamp = %d", type_name, value.ui32ArduinoTimestamp));
//        }
//    }
//    // failed
//    else
//    {
//        LOG_WARNING(cString::Format("No mediadescription for %s found!", type_name));
//    }

//    // done
//    RETURN_NOERROR;
//}


//// createTSignalValueMediaDescription
//// this helper function will create media description of tsignalvalue for you !
//tResult createTSignalValueMediaDescription(object_ptr<IStreamType> &op, adtf::mediadescription::cSampleCodecFactory &factory, iSignalValue &value, const char* type_name)
//{
//    // get streamtype from aadc.description file
//    if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", op, factory)))
//    {
//        // get value
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32Value"), value.f32Value)))
//        {
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32Value", type_name));
//        }
//        else
//        {
//            LOG_SUCCESS(cString::Format("FOUND INDEX: %s.f32Value  = %d", type_name, value.f32Value));
//        }

//        // get timestamp
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui32ArduinoTimestamp"),
//                                                                value.ui32ArduinoTimestamp)))
//        {
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui32Timestamp", type_name));
//        }
//        else
//        {
//            LOG_SUCCESS(cString::Format("FOUND INDEX: %s.ui32ArduinoTimestamp =  %d", type_name, value.ui32ArduinoTimestamp));
//        }
//    }
//    // failed
//    else
//    {
//        LOG_WARNING(cString::Format("No mediadescription for %s found!", type_name));
//    }

//    // done
//    RETURN_NOERROR;

//}

//// createTFeedbackMediaDescription
//// this helper function will create media description of tfeedback for you !
//tResult createTFeedbackMediaDescription(object_ptr<IStreamType> &op, adtf::mediadescription::cSampleCodecFactory &factory, iFeedbackStruct &value, const char* type_name)
//{
//    // get streamtype from aadc.description file
//    if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tFeedbackStruct", op, factory)))
//    {
//        // get filterid
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui8FilterId"), value.ui8FilterId )))
//        {
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui8FilterId", type_name));
//        }
//        else
//        {
//            LOG_SUCCESS(cString::Format("FOUND INDEX: %s.ui8FilterId = %d", type_name, value.ui8FilterId));
//        }

//        // get feedbackstatus
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui32FeedbackStatus"), value.ui32FeedbackStatus )))
//        {
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui32FeedbackStatus", type_name));
//        }
//        else
//        {
//            LOG_SUCCESS(cString::Format("FOUND INDEX: %s.ui32FeedbackStatus = %d", type_name, value.ui32FeedbackStatus));
//        }
//    }
//    // failed
//    else
//    {
//        LOG_WARNING(cString::Format("No mediadescription for %s found!", type_name));
//    }

//    // done
//    RETURN_NOERROR;

//}
///*
//#pragma pack(push,1)
//typedef struct
//{
//    tSize bEnabled;
//    tSize bStarted;
//    tSize ui32Command;
//} iActionSub;
//#pragma pack(pop)

//#pragma pack(push,1)
//typedef struct
//{
//    tSize      ui8FilterId;
//    iActionSub subAction;
//} iAction;
//#pragma pack(pop)
//*/
//// createTActionMediaDescription
//// this helper function will create media description of taction for you !
//tResult createTActionMediaDescription(object_ptr<IStreamType> &op, adtf::mediadescription::cSampleCodecFactory &factory, iAction &value, const char* type_name)
//{
//    if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tAction", op, factory)))
//    {
//        // if loading was successful then get index from aadc description html file
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui8FilterId")          , (value.ui8FilterId))))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui8FilterId", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("subAction") + cString(".bEnabled")   , value.subAction.bEnabled)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.subAction.bEnabled", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("subAction.bStarted")   , value.subAction.bStarted)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.subAction.bStarted", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("subAction.ui32Command"), value.subAction.ui32Command)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.subAction.ui32Command", type_name));
       
//        LOG_SUCCESS("MediaDescription for tActionStruct was successfully found!");
//    }
//    else
//    {
//            // did not find datatype in description file
//        LOG_WARNING("no mediadescription for tActionStruct found! :(");
//    }

//    // done
//    RETURN_NOERROR;
//}

//// createTActionStructMediaDescription
//// this helper function will create media description of tactionstruct for you !
//tResult createTActionStructMediaDescription(object_ptr<IStreamType> &op, adtf::mediadescription::cSampleCodecFactory &factory, iActionStruct &value, const char* type_name)
//{
//    if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tActionStruct", op, factory)))
//    {
//        // if loading was successful then get index from aadc description html file
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action1.ui8FilterId")          , (value.action1.ui8FilterId))))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action1.ui8FilterId", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action1.subAction.bEnabled")   , value.action1.subAction.bEnabled)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action1.subActtion.bEnabled", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action1.subAction.bStarted")   , value.action1.subAction.bStarted)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action1.subActtion.bStarted", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action1.subAction.ui32Command"), value.action1.subAction.ui32Command)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action1.subActtion.ui32Command", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action2.ui8FilterId")          , value.action2.ui8FilterId)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action2.ui8FilterId", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action2.subAction.bEnabled")   , value.action2.subAction.bEnabled)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action2.subActtion.bEnabled", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action2.subAction.bStarted")   , value.action2.subAction.bStarted)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action2.subActtion.bStarted", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action2.subAction.ui32Command"), value.action2.subAction.ui32Command)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action2.subActtion.ui32Command", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action3.ui8FilterId")          , value.action3.ui8FilterId)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action3.ui8FilterId", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action3.subAction.bEnabled")   , value.action3.subAction.bEnabled)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action3.subActtion.bEnabled", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action3.subAction.bStarted")   , value.action3.subAction.bStarted)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action3.subActtion.bStarted", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action3.subAction.ui32Command"), value.action3.subAction.ui32Command)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action3.subActtion.ui32Command", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action4.ui8FilterId")          , value.action4.ui8FilterId)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action4.ui8FilterId", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action4.subAction.bEnabled")   , value.action4.subAction.bEnabled)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action4.subActtion.bEnabled", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action4.subAction.bStarted")   , value.action4.subAction.bStarted)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action4.subActtion.bStarted", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action4.subAction.ui32Command"), value.action4.subAction.ui32Command)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action4.subActtion.ui32Command", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action5.ui8FilterId")          , value.action5.ui8FilterId)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action5.ui8FilterId", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action5.subAction.bEnabled")   , value.action5.subAction.bEnabled)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action5.subActtion.bEnabled", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action5.subAction.bStarted")   , value.action5.subAction.bStarted)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action5.subActtion.bStarted", type_name));
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("action5.subAction.ui32Command"), value.action5.subAction.ui32Command)))
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.action5.subActtion.ui32Command", type_name));

//        LOG_SUCCESS("MediaDescription for tActionStruct was successfully found!");
//    }
//    else
//    {
//            // did not find datatype in description file
//        LOG_WARNING("no mediadescription for tActionStruct found! :(");
//    }

//    // done
//    RETURN_NOERROR;
//}

//// createTUltrasonicStructMediaDescription
//// this helper function will create media description of tultrasonicstruct for you !
//tResult createTUltrasonicStructMediaDescription(object_ptr<IStreamType> &op, adtf::mediadescription::cSampleCodecFactory &factory, iUltrasonicStruct &value, const char* type_name)
//{
//    // get streamtype from aadc.description file
//    if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tUltrasonicStruct", op, factory)))
//    {
//        // -----------------------tSideLeft------------------------------------------------
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tSideLeft")  + cString(".ui32ArduinoTimestamp"), value.tSideLeft.ui32ArduinoTimestamp )))
//        {
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tSideLeft.ui32ArduinoTimestamp", type_name));
//        }
//        else
//        {
//            LOG_SUCCESS(cString::Format("FOUND INDEX: %s.tSideLeft.ui32ArduinoTimestamp = %d", type_name, value.tSideLeft.ui32ArduinoTimestamp));
//        }
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tSideLeft")  + cString(".f32Value"), value.tSideLeft.f32Value )))
//        {
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tSideLeft.f32Value", type_name));
//        }
//        else
//        {
//            LOG_SUCCESS(cString::Format("FOUND INDEX: %s.tSideLeft.f32Value = %d", type_name, value.tSideLeft.f32Value));
//        }

//        // -----------------------tSideRight------------------------------------------------
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tSideRight") + cString(".ui32ArduinoTimestamp"), value.tSideRight.ui32ArduinoTimestamp )))
//        {
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tSideRight.ui32ArduinoTimestamp", type_name));
//        }
//        else
//        {
//            LOG_SUCCESS(cString::Format("FOUND INDEX: %s.tSideRight.ui32ArduinoTimestamp = %d", type_name, value.tSideRight.ui32ArduinoTimestamp));
//        }
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tSideRight") + cString(".f32Value"), value.tSideRight.f32Value )))
//        {
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tSideRight.f32Value", type_name));
//        }
//        else
//        {
//            LOG_SUCCESS(cString::Format("FOUND INDEX: %s.tSideRight = %d", type_name, value.tSideRight.f32Value));
//        }

//        // -----------------------tRearLeft------------------------------------------------
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tRearLeft") + cString(".ui32ArduinoTimestamp"), value.tRearLeft.ui32ArduinoTimestamp )))
//        {
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tRearLeft.ui32ArduinoTimestamp", type_name));
//        }
//        else
//        {
//            LOG_SUCCESS(cString::Format("FOUND INDEX: %s.tRearLeft = %d", type_name, value.tRearLeft.ui32ArduinoTimestamp));
//        }
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tRearLeft") + cString(".f32Value"), value.tRearLeft.f32Value )))
//        {
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tRearLeft.f32Value", type_name));
//        }
//        else
//        {
//            LOG_SUCCESS(cString::Format("FOUND INDEX: %s.tRearLeft.f32Value = %d", type_name, value.tRearLeft.f32Value));
//        }

//        // -----------------------tRearCenter------------------------------------------------
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tRearCenter") + cString(".ui32ArduinoTimestamp"), value.tRearCenter.ui32ArduinoTimestamp )))
//        {
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tRearCenter.ui32ArduinoTimestamp", type_name));
//        }
//        else
//        {
//            LOG_SUCCESS(cString::Format("FOUND INDEX: %s.tRearCenter.ui32ArduinoTimestamp = %d", type_name, value.tRearCenter.ui32ArduinoTimestamp));

//        }
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tRearCenter") + cString(".f32Value"), value.tRearCenter.f32Value )))
//        {
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tRearCenter.f32Value", type_name));
//        }
//        else
//        {
//            LOG_SUCCESS(cString::Format("FOUND INDEX: %s.tRearCenter.f32Value = %d", type_name, value.tRearCenter.f32Value));

//        }
//        // -----------------------tRearRight------------------------------------------------
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tRearRight") + cString(".ui32ArduinoTimestamp"), value.tRearRight.ui32ArduinoTimestamp )))
//        {
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tRearRight.ui32ArduinoTimestamp", type_name));
//        }
//        else
//        {
//            LOG_SUCCESS(cString::Format("FOUND INDEX: %s.tRearRight = %d", type_name, value.tRearRight.ui32ArduinoTimestamp));
//        }
//        if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tRearRight") + cString(".f32Value"), value.tRearRight.f32Value )))
//        {
//            LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tRearRight.f32Value", type_name));
//        }
//        else
//        {
//            LOG_SUCCESS(cString::Format("FOUND INDEX: %s.tRearRight.f32Value = %d", type_name, value.tRearRight.f32Value));
//        }
//    }
//    // failed
//    else
//    {
//        LOG_WARNING(cString::Format("No mediadescription for %s found!", type_name));
//    }

//    // done
//    RETURN_NOERROR;

//}
