// ------------------------------------
// &Author: Xiangfei &Datum: 23.08.2018
// assumed that aadc_structs.h & aadc_user_structs.h are included in aadc_type_interface.h
//-------------------------------------

#pragma once

#include "aadc_type_interface.h"


class TActionStruct: public AADC_Type 
{
#pragma pack(push,1)
    struct ID
    {
		tSize ui32ArduinoTimestamp;
        tSize ui8FilterId;
        tSize bEnabled;
        tSize bStarted;
        tSize ui32Command;

        ID(){
			ui32ArduinoTimestamp = 0;
            ui8FilterId = 0;
            bEnabled    = 0;
            bStarted    = 0;
            ui32Command = 0;
        }
    };
#pragma pack(pop)

    ID index;

public:

#pragma pack(push,1)
    struct Data
    {
		tUInt32 ui32ArduinoTimestamp;
        tUInt8 ui8FilterId;
        tBool bEnabled;
        tBool bStarted;
        tUInt32 ui32Command;

        Data(){
			ui32ArduinoTimestamp = 0;
            ui8FilterId = 0;
            bEnabled    = tFalse;
            bStarted    = tFalse;
            ui32Command = 0;
        }
    };
#pragma pack(pop)

    TActionStruct()
    {
        type_name = cString("tActionStruct");
        // get streamtype from aadc.description file
        if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(type_name, object, factory)))
        {
            // if loading was successful then get index from aadc description html file
            // get timestamp
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui32ArduinoTimestamp"), index.ui32ArduinoTimestamp)))
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui32Timestamp", type_name));
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui8FilterId")   , index.ui8FilterId)))
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui8FilterId", type_name));
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("bEnabled")   , index.bEnabled)))
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.bEnabled", type_name));
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("bStarted")   , index.bStarted)))
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.bStarted", type_name));
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui32Command"), index.ui32Command)))
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui32Command", type_name));
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
            if(IS_FAILED(res = oCodec.SetElementValue(index.ui8FilterId, content.ui8FilterId)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.ui8FilterId for index %d", type_name, index.ui8FilterId));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.bEnabled, content.bEnabled)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.bEnabled for index %d", type_name, index.bEnabled));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.bStarted, content.bStarted)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.bStarted for index %d", type_name, index.bStarted));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.ui32Command, content.ui32Command)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.ui32Command for index %d", type_name, index.ui32Command));
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
            } else if(lastTimestamp == content.ui32ArduinoTimestamp){
				RETURN_ERROR(ERR_FAILED);
			}
            // retrieve the values (using convenience methods that return a variant)
            if(IS_FAILED(res = oDecoder.GetElementValue(index.ui8FilterId, &(content.ui8FilterId))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.ui8FilterId for index %d", type_name, index.ui8FilterId));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.bEnabled, &(content.bEnabled))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.bEnabled for index %d", type_name, index.bEnabled));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.bStarted, &(content.bStarted))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.bStarted for index %d", type_name, index.bStarted));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.ui32Command, &(content.ui32Command))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.ui32Command for index %d", type_name, index.ui32Command));
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


/* we won't use this
    TActionStruct readAction(void *content_ptr, tUInt8 filterID)
    {
        Data &content = *((Data *) content_ptr);
        TActionStruct resultAction;

        if(content.ui8FilterId == filterID)
        {
            resultAction = content;
        }
        else
        {
            // return empty zero result;
            return (tActionStruct());
        }

        tUInt32 filterIDFlag = filterID * 1000;
        if(resultAction.ui32Command < filterIDFlag || resultAction.ui32Command >= filterIDFlag + 1000)
        {
            LOG_WARNING(cString::Format ("TActionStruct.h::Read_Action(): ERROR filter ID %d does not match to command %d", filterID, resultAction.ui32Command));

            // return empty zero result;
            return (tActionSub());
        }

        return resultAction;

    }
    * 
    * */
};

