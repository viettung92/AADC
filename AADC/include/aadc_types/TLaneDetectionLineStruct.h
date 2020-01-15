// ------------------------------------
// &Author: Binh &Datum: 27.09.2018
// assumed that aadc_structs.h & aadc_user_structs.h are included in aadc_type_interface.h
//-------------------------------------

#pragma once

#include "aadc_type_interface.h"


class TLaneDetectionLineStruct: public AADC_Type 
{
#pragma pack(push,1)
    struct ID
    {
	tSize ui32ArduinoTimestamp;
        tSize i8IsCurve;
        tSize i32Point1X;
        tSize i32Point1Y;
        tSize i32Point2X;
	tSize i32Point2Y;
        tSize i32Point3X;
        tSize i32Point3Y;
        tSize i32Point4X;
	tSize i32Point4Y;

        ID()
        {
	    ui32ArduinoTimestamp = 0;
            i8IsCurve     = 0;
            i32Point1X    = 0;
            i32Point1Y    = 0;
            i32Point2X    = 0;
            i32Point2Y    = 0;
            i32Point3X    = 0;
            i32Point3Y    = 0;
            i32Point4X    = 0;
            i32Point4Y    = 0;
        }
    };
#pragma pack(pop)

    ID index;

public:

#pragma pack(push,1)
    struct Data
    {
	tUInt32 ui32ArduinoTimestamp;
        tInt8   i8IsCurve;
        tInt32  i32Point1X;
        tInt32  i32Point1Y;
        tInt32  i32Point2X;
	tInt32  i32Point2Y;
        tInt32  i32Point3X;
        tInt32  i32Point3Y;
        tInt32  i32Point4X;
	tInt32  i32Point4Y;

        Data()
        {
	    ui32ArduinoTimestamp = 0;
            i8IsCurve     = 0;
            i32Point1X    = 0;
            i32Point1Y    = 0;
            i32Point2X    = 0;
            i32Point2Y    = 0;
            i32Point3X    = 0;
            i32Point3Y    = 0;
            i32Point4X    = 0;
            i32Point4Y    = 0;
        }
    };
#pragma pack(pop)

    TLaneDetectionLineStruct()
    {
        type_name = cString("tLaneDetectionLineStruct");
        // get streamtype from aadc.description file
        if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(type_name, object, factory)))
        {
            // if loading was successful then get index from aadc description html file
            // get timestamp
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui32ArduinoTimestamp"), index.ui32ArduinoTimestamp)))
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui32Timestamp", type_name));
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("i8IsCurve")   , index.i8IsCurve)))
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.i8IsCurve", type_name));
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("i32Point1X")   , index.i32Point1X)))
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.i32Point1X", type_name));
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("i32Point1Y")   , index.i32Point1Y)))
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.i32Point1Y", type_name));
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("i32Point2X"), index.i32Point2X)))
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.i32Point2X", type_name));
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("i32Point2Y"), index.i32Point2Y)))
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.i32Point2Y", type_name));
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("i32Point3X")   , index.i32Point3X)))
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.i32Point3X", type_name));
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("i32Point3Y")   , index.i32Point3Y)))
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.i32Point3Y", type_name));
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("i32Point4X"), index.i32Point4X)))
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.i32Point4X", type_name));
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("i32Point4Y"), index.i32Point4Y)))
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.i32Point4Y", type_name));
          //  LOG_SUCCESS("MediaDescription for tActionStruct was successfully found!");

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
			content.ui32ArduinoTimestamp = streamTime;

            if(IS_FAILED(res = oCodec.SetElementValue(index.ui32ArduinoTimestamp, content.ui32ArduinoTimestamp)))
            {

                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.ui32ArduinoTimestamp for index %d", type_name, index.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.i8IsCurve, content.i8IsCurve)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.i8IsCurve for index %d", type_name, index.i8IsCurve));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.i32Point1X, content.i32Point1X)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.i32Point1X for index %d", type_name, index.i32Point1X));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.i32Point1Y, content.i32Point1Y)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.i32Point1Y for index %d", type_name, index.i32Point1Y));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.i32Point2X, content.i32Point2X)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.i32Point2X for index %d", type_name, index.i32Point2X));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.i32Point2Y, content.i32Point2Y)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.i32Point2Y for index %d", type_name, index.i32Point2Y));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.i32Point3X, content.i32Point3X)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.i32Point3X for index %d", type_name, index.i32Point3X));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.i32Point3Y, content.i32Point3Y)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.i32Point3Y for index %d", type_name, index.i32Point3Y));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.i32Point4X, content.i32Point4X)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.i32Point4X for index %d", type_name, index.i32Point4X));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.i32Point4Y, content.i32Point4Y)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.i32Point4Y for index %d", type_name, index.i32Point4Y));
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



    tResult readPin(cPinReader &input, void *content_ptr, tTimeStamp lastTimestamp = 0)
    {
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
            if(IS_FAILED(res = oDecoder.GetElementValue(index.ui32ArduinoTimestamp, &(content.ui32ArduinoTimestamp))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.ui32ArduinoTimestamp for index %d", type_name, index.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            } 
	    else if(lastTimestamp == content.ui32ArduinoTimestamp)
            {
		RETURN_ERROR(ERR_FAILED);
	    }
            // retrieve the values (using convenience methods that return a variant)
            if(IS_FAILED(res = oDecoder.GetElementValue(index.i8IsCurve, &(content.i8IsCurve))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.i8IsCurve for index %d", type_name, index.i8IsCurve));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.i32Point1X, &(content.i32Point1X))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.i32Point1X for index %d", type_name, index.i32Point1X));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.i32Point1Y, &(content.i32Point1Y))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.i32Point1Y for index %d", type_name, index.i32Point1Y));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.i32Point2X, &(content.i32Point2X))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.i32Point2X for index %d", type_name, index.i32Point2X));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.i32Point2Y, &(content.i32Point2Y))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.i32Point2Y for index %d", type_name, index.i32Point2Y));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.i32Point3X, &(content.i32Point3X))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.i32Point3X for index %d", type_name, index.i32Point3X));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.i32Point3Y, &(content.i32Point3Y))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.i32Point3Y for index %d", type_name, index.i32Point3Y));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.i32Point4X, &(content.i32Point4X))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.i32Point4X for index %d", type_name, index.i32Point4X));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.i32Point4Y, &(content.i32Point4Y))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.i32Point4Y for index %d", type_name, index.i32Point4Y));
                RETURN_ERROR(res);
            }
        }
        else
        {
         //   LOG_WARNING("readPin: getLastSample did not work!");
            RETURN_ERROR(res);
        }
        RETURN_NOERROR;
    }
};

