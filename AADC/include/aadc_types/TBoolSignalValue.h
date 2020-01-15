#pragma once

#include "aadc_type_interface.h"


class TBoolSignalValue: public AADC_Type {

	#pragma pack(push,1)
	struct ID
	{
		tSize ui32ArduinoTimestamp;
		tSize bValue;
		
		ID() {
			ui32ArduinoTimestamp = 0;
			bValue = 1;
		}
	};
	
    ID index;
	#pragma pack(pop)
	
public:

	#pragma pack(push,1)
	struct Data
	{
		tUInt32 ui32ArduinoTimestamp;
		tBool bValue;
		
		Data() {
			ui32ArduinoTimestamp = 0;
			bValue = 0;
		}
	};
	#pragma pack(pop)

    TBoolSignalValue(){
        type_name = cString("tBoolSignalValue");
        // get streamtype from aadc.description file
        if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(type_name, object, factory)))
        {
            // get value
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("bValue"), index.bValue)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.bValue", type_name));
            }

            // get timestamp
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui32ArduinoTimestamp"),
                                                                    index.ui32ArduinoTimestamp)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui32Timestamp", type_name));
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
			content.ui32ArduinoTimestamp = streamTime;

            if(IS_FAILED(res = oCodec.SetElementValue(index.bValue, content.bValue)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.bValue for index %d",type_name, index.bValue));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.ui32ArduinoTimestamp, content.ui32ArduinoTimestamp)))
            {

                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.ui32ArduinoTimestamp for index %d", type_name, index.ui32ArduinoTimestamp));
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
            
            if(IS_FAILED(res = oDecoder.GetElementValue(index.ui32ArduinoTimestamp, &(content.ui32ArduinoTimestamp))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.ui32ArduinoTimestamp for index %d", type_name, index.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            } else if(lastTimestamp == content.ui32ArduinoTimestamp){
				RETURN_ERROR(ERR_FAILED);
			}

            // retrieve the values (using convenience methods that return a variant)
            if(IS_FAILED(res = oDecoder.GetElementValue(index.bValue, &(content.bValue))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.bValue for index %d", type_name, index.bValue));
                RETURN_ERROR(res);
            }


        }
        else
        {
      //      LOG_WARNING("readPin: getLastSample did not work!");
            RETURN_ERROR(res);
        }

        RETURN_NOERROR;

    }



};

