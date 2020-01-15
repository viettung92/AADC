// ------------------------------------
// &Author: Xiangfei &Datum: 22.08.2018
// assumed that aadc_structs.h will be included in aadc_type_interface.h
// otherwise tSignalValue & iSignalValue must be defined here
//-------------------------------------
#pragma once

#include "aadc_type_interface.h"


class TUltrasonicStruct: public AADC_Type {

//#pragma pack(push,1)
//    struct iSignalValue
//    {
//        tSize f32Value;
//        tSize ui32ArduinoTimestamp;

//        iSignalValue(){
//            f32Value = 0;
//            ui32ArduinoTimestamp = 0;
//        }
//    };
//#pragma pack(pop)


#pragma pack(push,1)
    struct ID
    {
        iSignalValue tSideLeft;
        iSignalValue tSideRight;
        iSignalValue tRearLeft;
        iSignalValue tRearCenter;
        iSignalValue tRearRight;

                ID(){
                    tSideLeft.f32Value = 0;
                    tSideLeft.ui32ArduinoTimestamp = 0;
                    tSideRight.f32Value = 0;
                    tSideRight.ui32ArduinoTimestamp = 0;
                    tRearLeft.f32Value = 0;
                    tRearLeft.ui32ArduinoTimestamp = 0;
                    tRearCenter.f32Value = 0;
                    tRearCenter.ui32ArduinoTimestamp = 0;
                    tRearRight.f32Value = 0;
                    tRearRight.ui32ArduinoTimestamp = 0;
                }
    };
#pragma pack(pop)

    ID index;

public:

//#pragma pack(push,1)
//    struct tSignalValue
//    {
//        tFloat32 f32Value;
//        tUInt32 ui32ArduinoTimestamp;

//        tSignalValue(){
//            f32Value = 0;
//            ui32ArduinoTimestamp = 0;
//        }
//    };
//#pragma pack(pop)



#pragma pack(push,1)
    struct Data
    {
        tSignalValue tSideLeft;
        tSignalValue tSideRight;
        tSignalValue tRearLeft;
        tSignalValue tRearCenter;
        tSignalValue tRearRight;

        Data(){
            tSideLeft.f32Value = 0;
            tSideLeft.ui32ArduinoTimestamp = 0;
            tSideRight.f32Value = 0;
            tSideRight.ui32ArduinoTimestamp = 0;
            tRearLeft.f32Value = 0;
            tRearLeft.ui32ArduinoTimestamp = 0;
            tRearCenter.f32Value = 0;
            tRearCenter.ui32ArduinoTimestamp = 0;
            tRearRight.f32Value = 0;
            tRearRight.ui32ArduinoTimestamp = 0;
        }
    };
#pragma pack(pop)

    TUltrasonicStruct(){
        type_name = cString("tUltrasonicStruct");
        // get streamtype from aadc.description file
        if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(type_name, object, factory)))
        {
            // -----------------------tSideLeft------------------------------------------------
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tSideLeft")  + cString(".ui32ArduinoTimestamp"), index.tSideLeft.ui32ArduinoTimestamp )))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tSideLeft.ui32ArduinoTimestamp", type_name));
            }

            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tSideLeft")  + cString(".f32Value"), index.tSideLeft.f32Value )))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tSideLeft.f32Value", type_name));
            }

            // -----------------------tSideRight------------------------------------------------
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tSideRight") + cString(".ui32ArduinoTimestamp"), index.tSideRight.ui32ArduinoTimestamp )))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tSideRight.ui32ArduinoTimestamp", type_name));
            }
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tSideRight") + cString(".f32Value"), index.tSideRight.f32Value )))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tSideRight.f32Value", type_name));
            }

            // -----------------------tRearLeft------------------------------------------------
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tRearLeft") + cString(".ui32ArduinoTimestamp"), index.tRearLeft.ui32ArduinoTimestamp )))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tRearLeft.ui32ArduinoTimestamp", type_name));
            }
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tRearLeft") + cString(".f32Value"), index.tRearLeft.f32Value )))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tRearLeft.f32Value", type_name));
            }

            // -----------------------tRearCenter------------------------------------------------
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tRearCenter") + cString(".ui32ArduinoTimestamp"), index.tRearCenter.ui32ArduinoTimestamp )))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tRearCenter.ui32ArduinoTimestamp", type_name));
            }
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tRearCenter") + cString(".f32Value"), index.tRearCenter.f32Value )))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tRearCenter.f32Value", type_name));
            }
            // -----------------------tRearRight------------------------------------------------
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tRearRight") + cString(".ui32ArduinoTimestamp"), index.tRearRight.ui32ArduinoTimestamp )))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tRearRight.ui32ArduinoTimestamp", type_name));
            }
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tRearRight") + cString(".f32Value"), index.tRearRight.f32Value )))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tRearRight.f32Value", type_name));
            }
        }
        // failed
        else
        {
            LOG_WARNING(cString::Format("No mediadescription for " + type_name + " found!"));
        }
    }

    tResult writePin(cPinWriter &output, void *content_ptr, tTimeStamp streamTime)
    {

        Data &content = *((Data *) content_ptr);
        object_ptr<ISample> sample;
        tResult res;

        if (IS_OK(res = alloc_sample(sample, streamTime)))
        {
            auto oCodec = factory.MakeCodecFor(sample);

            if(IS_FAILED(res = oCodec.IsValid()))
            {
                LOG_WARNING("CODEC CREATION FAILED!");
                RETURN_ERROR(res);
            }
			//Set timestamp
			//content.ui32ArduinoTimestamp = streamTime;

            // side left
            if(IS_FAILED(res = oCodec.SetElementValue(index.tSideLeft.ui32ArduinoTimestamp, content.tSideLeft.ui32ArduinoTimestamp)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.tSideLeft.ui32ArduinoTimestamp for index %d",type_name, index.tSideLeft.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.tSideLeft.f32Value, content.tSideLeft.f32Value)))
            {

                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.tSideLeft.f32Value for index %d", type_name, index.tSideLeft.f32Value));
                RETURN_ERROR(res);
            }

            // side right
            if(IS_FAILED(res = oCodec.SetElementValue(index.tSideRight.ui32ArduinoTimestamp, content.tSideRight.ui32ArduinoTimestamp)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.tSideRight.ui32ArduinoTimestamp for index %d",type_name, index.tSideRight.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.tSideRight.f32Value, content.tSideRight.f32Value)))
            {

                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.tSideRight.f32Value for index %d", type_name, index.tSideRight.f32Value));
                RETURN_ERROR(res);
            }

            // rear left
            if(IS_FAILED(res = oCodec.SetElementValue(index.tRearLeft.ui32ArduinoTimestamp, content.tRearLeft.ui32ArduinoTimestamp)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.tRearLeft.ui32ArduinoTimestamp for index %d",type_name, index.tRearLeft.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.tRearLeft.f32Value, content.tRearLeft.f32Value)))
            {

                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.tRearLeft.f32Value for index %d", type_name, index.tRearLeft.f32Value));
                RETURN_ERROR(res);
            }

            // rear right
            if(IS_FAILED(res = oCodec.SetElementValue(index.tRearRight.ui32ArduinoTimestamp, content.tRearRight.ui32ArduinoTimestamp)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.tRearRight.ui32ArduinoTimestamp for index %d",type_name, index.tRearRight.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.tRearRight.f32Value, content.tRearRight.f32Value)))
            {

                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.tRearRight.f32Value for index %d", type_name, index.tRearRight.f32Value));
                RETURN_ERROR(res);
            }

            // rear center
            if(IS_FAILED(res = oCodec.SetElementValue(index.tRearCenter.ui32ArduinoTimestamp, content.tRearCenter.ui32ArduinoTimestamp)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.tRearCenter.ui32ArduinoTimestamp for index %d",type_name, index.tRearCenter.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.tRearCenter.f32Value, content.tRearCenter.f32Value)))
            {

                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.tRearCenter.f32Value for index %d", type_name, index.tRearCenter.f32Value));
                RETURN_ERROR(res);
            }

        }
        else
        {
            LOG_WARNING("writePin: alloc_sample did not work!");
            RETURN_ERROR(res);

        }

        output << sample << flush << trigger;

        RETURN_NOERROR;
    }

    tResult readPin(cPinReader &input, void *content_ptr, tTimeStamp lastTimestamp = 0){
        Data &content = *((Data *) content_ptr);

        tResult res;
        object_ptr<const ISample> sample;

        if (IS_OK(res = input.GetLastSample(sample)))
        {
            auto oDecoder = factory.MakeDecoderFor(*sample);

            if(IS_FAILED(res = oDecoder.IsValid()))
            {
                LOG_WARNING("DECODER CREATION FAILED!");
                RETURN_ERROR(res);
            }

            // retrieve the values (using convenience methods that return a variant)
            // side left
			
	   if(IS_FAILED(res = oDecoder.GetElementValue(index.tSideLeft.ui32ArduinoTimestamp, &(content.tSideLeft.ui32ArduinoTimestamp))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.tSideLeft.ui32ArduinoTimestamp for index %d", type_name, index.tSideLeft.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            }else if(lastTimestamp == content.tSideLeft.ui32ArduinoTimestamp){
				RETURN_ERROR(ERR_FAILED);
			}

            if(IS_FAILED(res = oDecoder.GetElementValue(index.tSideLeft.f32Value, &(content.tSideLeft.f32Value))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.tSideLeft.f32Value for index %d", type_name, index.tSideLeft.f32Value));
                RETURN_ERROR(res);
            }
            

            // side right
            if(IS_FAILED(res = oDecoder.GetElementValue(index.tSideRight.f32Value, &(content.tSideRight.f32Value))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.tSideRight.f32Value for index %d", type_name, index.tSideRight.f32Value));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.tSideRight.ui32ArduinoTimestamp, &(content.tSideRight.ui32ArduinoTimestamp))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.tSideRight.ui32ArduinoTimestamp for index %d", type_name, index.tSideRight.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            } 

            // rear left
            if(IS_FAILED(res = oDecoder.GetElementValue(index.tRearLeft.f32Value, &(content.tRearLeft.f32Value))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.tRearLeft.f32Value for index %d", type_name, index.tRearLeft.f32Value));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.tRearLeft.ui32ArduinoTimestamp, &(content.tRearLeft.ui32ArduinoTimestamp))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.tRearLeft.ui32ArduinoTimestamp for index %d", type_name, index.tRearLeft.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            }

            // rear right
            if(IS_FAILED(res = oDecoder.GetElementValue(index.tRearRight.f32Value, &(content.tRearRight.f32Value))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.tRearRight.f32Value for index %d", type_name, index.tRearRight.f32Value));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.tRearRight.ui32ArduinoTimestamp, &(content.tRearRight.ui32ArduinoTimestamp))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.tRearRight.ui32ArduinoTimestamp for index %d", type_name, index.tRearRight.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            }

            // rear center
            if(IS_FAILED(res = oDecoder.GetElementValue(index.tRearCenter.f32Value, &(content.tRearCenter.f32Value))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.tRearCenter.f32Value for index %d", type_name, index.tRearCenter.f32Value));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.tRearCenter.ui32ArduinoTimestamp, &(content.tRearCenter.ui32ArduinoTimestamp))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.tRearCenter.ui32ArduinoTimestamp for index %d", type_name, index.tRearCenter.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            }


        }
        else
        {
            LOG_WARNING("readPin: getLastSample did not work!");
            RETURN_ERROR(res);
        }

        RETURN_NOERROR;

    }

};

