#pragma once

#include "aadc_type_interface.h"


class TTrafficSign: public AADC_Type 
{


	#pragma pack(push,1)
	struct ID
	{
		tSize i16Identifier;
    		tSize f32x;
    		tSize f32y;
    		tSize f32angle;
		
		ID() {
			i16Identifier = 0;
			f32x = 0;
			f32y = 0;
			f32angle = 0;
		}
	};
	#pragma pack(pop)

    ID index;
	
public:

	#pragma pack(push,1)
	struct Data
	{
		tInt16 i16Identifier;
    		tFloat32 f32x;
    		tFloat32 f32y;
    		tFloat32 f32angle;
		
		Data() 
	{
			i16Identifier = 0;
			f32x = 0;
			f32y = 0;
			f32angle = 0;
		}
	};
	#pragma pack(pop)

    TTrafficSign()
	{
        type_name = cString("tTrafficSign");
        // get streamtype from aadc.description file
        if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(type_name, object, factory)))
        {
            // get i16Identifier
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("i16Identifier"), index.i16Identifier)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.i16Identifier", type_name));
            }

            // get f32x
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32x"),
                                                                    index.f32x)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32x", type_name));
            }

	    // get f32y
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32y"), index.f32y)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32y", type_name));
            }

            // get f32angle
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32angle"),
                                                                    index.f32angle)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32angle", type_name));
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

            if(IS_FAILED(res = oCodec.SetElementValue(index.i16Identifier, content.i16Identifier)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.i16Identifier for index %d",type_name, index.i16Identifier));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.f32x, content.f32x)))
            {

                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.f32x for index %d", type_name, index.f32x));
                RETURN_ERROR(res);
            }

	    if(IS_FAILED(res = oCodec.SetElementValue(index.f32y, content.f32y)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32y for index %d",type_name, index.f32y));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.f32angle, content.f32angle)))
            {

                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.f32angle for index %d", type_name, index.f32angle));
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
            if(IS_FAILED(res = oDecoder.GetElementValue(index.i16Identifier, &(content.i16Identifier))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.i16Identifier for index %d", type_name, index.i16Identifier));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32x, &(content.f32x))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32x for index %d", type_name, index.f32x));
                RETURN_ERROR(res);
            }
	    if(IS_FAILED(res = oDecoder.GetElementValue(index.f32y, &(content.f32y))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32y for index %d", type_name, index.f32y));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32angle, &(content.f32angle))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32angle for index %d", type_name, index.f32angle));
                RETURN_ERROR(res);
            }


        }
        else
        {
            LOG_WARNING(cString::Format("readPin: getLastSample did not work! No valid timestamp submitted : %d",lastTimestamp));
            RETURN_ERROR(res);
        }

        RETURN_NOERROR;

    }



};

