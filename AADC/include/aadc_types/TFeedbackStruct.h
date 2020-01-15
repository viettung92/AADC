// ------------------------------------
// &Author: Xiangfei &Datum: 22.08.2018
// &new: readPin one more check if ID is consistent with status
//-------------------------------------

#pragma once

#include "aadc_type_interface.h"


class TFeedbackStruct: public AADC_Type {


#pragma pack(push,1)
    struct ID
    {
		//tSize ui32ArduinoTimestamp;
        tSize  ui8FilterId;
        tSize  ui32FeedbackStatus;
		tSize ui32ArduinoTimestamp;

        ID() {
			//ui32ArduinoTimestamp = 0;
            ui8FilterId = 0;
            ui32FeedbackStatus = 0;
            ui32ArduinoTimestamp = 0;
        }
    };
#pragma pack(pop)

    ID index;

public:

#pragma pack(push,1)
    struct Data
    {
		//tUInt32 ui32ArduinoTimestamp;
        tUInt8  ui8FilterId;
        tUInt32 ui32FeedbackStatus;
		tUInt32 ui32ArduinoTimestamp;

        Data() {
			//ui32ArduinoTimestamp=0;
            ui8FilterId = 0;
            ui32FeedbackStatus = 0;
            ui32ArduinoTimestamp = 0;
        }
    };
#pragma pack(pop)

    TFeedbackStruct(){
        type_name = cString("tFeedbackStruct");
        // get streamtype from aadc.description file
        if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(type_name, object, factory)))
        {
            // get Id
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui8FilterId"), index.ui8FilterId)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui8FilterId", type_name));
            }

            // get Status
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui32FeedbackStatus"),
                                                              index.ui32FeedbackStatus)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui32FeedbackStatus", type_name));
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

		//	//Set timestamp
			content.ui32ArduinoTimestamp = streamTime;
			if(IS_FAILED(res = oCodec.SetElementValue(index.ui32ArduinoTimestamp, content.ui32ArduinoTimestamp)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.ui32ArduinoTimestamp for index %d",type_name, index.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            }

            if(IS_FAILED(res = oCodec.SetElementValue(index.ui8FilterId, content.ui8FilterId)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.ui8FilterId for index %d",type_name, index.ui8FilterId));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.ui32FeedbackStatus, content.ui32FeedbackStatus)))
            {

                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.ui32FeedbackStatus for index %d", type_name, index.ui32FeedbackStatus));
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
			//timestamp
            if(IS_FAILED(res = oDecoder.GetElementValue(index.ui32ArduinoTimestamp, &(content.ui32ArduinoTimestamp))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.ui32FeedbackStatus for index %d", type_name, index.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            }
             else if(lastTimestamp == content.ui32ArduinoTimestamp){
				RETURN_ERROR(ERR_FAILED);
			}
			
            // retrieve the values (using convenience methods that return a variant)
            if(IS_FAILED(res = oDecoder.GetElementValue(index.ui8FilterId, &(content.ui8FilterId))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.ui8FilterId for index %d", type_name, index.ui8FilterId));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.ui32FeedbackStatus, &(content.ui32FeedbackStatus))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.ui32FeedbackStatus for index %d", type_name, index.ui32FeedbackStatus));
                RETURN_ERROR(res);
            }

            // check if status belongs to id
            tUInt32 filterIDFlag = content.ui8FilterId * 1000;
            if(content.ui32FeedbackStatus < filterIDFlag || content.ui32FeedbackStatus >= filterIDFlag + 1000) {
                LOG_WARNING(cString::Format ("TFeedbackStruct.h::Read(): ERROR filter ID %d does not match status %d", content.ui8FilterId, content.ui32FeedbackStatus));
            }
        }
        else
        {
            //LOG_WARNING("readPin: getLastSample did not work!");
            RETURN_ERROR(res);
        }

        RETURN_NOERROR;

    }



};

