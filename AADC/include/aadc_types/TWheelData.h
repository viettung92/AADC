// ------------------------------------
// &Author: Nico &Datum: 29.08.2018
//-------------------------------------

#pragma once

#include "aadc_type_interface.h"

class TWheelData: public AADC_Type
{

#pragma pack(push,1)
    struct ID
    {
            tSize ui32ArduinoTimestamp;
            tSize ui32WheelTach;
            tSize i8WheelDir;

           ID()
           {
                ui32ArduinoTimestamp = 0;
                ui32WheelTach = 0;
                i8WheelDir = 0;
           }
    };
#pragma pack(pop)
    ID index;

public:

#pragma pack(push,1)
    struct Data
    {
        tUInt32 ui32ArduinoTimestamp;
        tUInt32 ui32WheelTach;
        tInt8   i8WheelDir;


        Data()
        {
            ui32ArduinoTimestamp = 0;
            ui32WheelTach = 0;
            i8WheelDir = 0;

        }
    };
#pragma pack(pop)


//Constructor
    TWheelData()
    {
        type_name = cString("tWheelData");
        // get streamtype from aadc.description file
        if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(type_name, object, factory)))
        {

            // get timestamp
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui32ArduinoTimestamp"),
                                                              index.ui32ArduinoTimestamp)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui32ArduinoTimestamp", type_name));
            }
            // get WheelTach
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui32WheelTach"), index.ui32WheelTach)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui32WheelTach", type_name));
            }
            // get i8WheelDir
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("i8WheelDir"), index.i8WheelDir)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.i8WheelDir", type_name));
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


            //set Timestamp
            if(IS_FAILED(res = oCodec.SetElementValue(index.ui32ArduinoTimestamp, content.ui32ArduinoTimestamp)))
            {

                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.ui32ArduinoTimestamp for index %d", type_name, index.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            }
            //set WheelTach
            if(IS_FAILED(res = oCodec.SetElementValue(index.ui32WheelTach, content.ui32WheelTach)))
            {

                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.ui32WheelTach for index %d", type_name, index.ui32WheelTach));
                RETURN_ERROR(res);
            }
            //set WheelDir
            if(IS_FAILED(res = oCodec.SetElementValue(index.i8WheelDir, content.i8WheelDir)))
            {

                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.i8WheelDir for index %d", type_name, index.i8WheelDir));
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
            //get Timestamp
            if(IS_FAILED(res = oDecoder.GetElementValue(index.ui32ArduinoTimestamp, &(content.ui32ArduinoTimestamp))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.ui32ArduinoTimestamp for index %d", type_name, index.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            } else if(lastTimestamp == content.ui32ArduinoTimestamp)
            {
				RETURN_ERROR(ERR_FAILED);
			}
            //get WheelTach
            if(IS_FAILED(res = oDecoder.GetElementValue(index.ui32WheelTach, &(content.ui32WheelTach))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.ui32WheelTach for index %d", type_name, index.ui32WheelTach));
                RETURN_ERROR(res);
            }
            //get WheelDir
            if(IS_FAILED(res = oDecoder.GetElementValue(index.i8WheelDir, &(content.i8WheelDir))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.i8WheelDir for index %d", type_name, index.i8WheelDir));
                RETURN_ERROR(res);
            }
			//input.Clear();

        }
        else
        {
          //  LOG_WARNING("readPin: getLastSample did not work!");
            RETURN_ERROR(res);
        }

        RETURN_NOERROR;

    }

};

