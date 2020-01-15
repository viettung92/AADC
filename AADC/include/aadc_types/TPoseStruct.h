// ------------------------------------
// &Author: Xiangfei &Datum: 22.08.2018
//-------------------------------------

#pragma once

#include "aadc_type_interface.h"


class TPoseStruct: public AADC_Type {

#pragma pack(push,1)
    struct ID
    {
        tSize ui32ArduinoTimestamp;
        tSize f32PosX;
        tSize f32PosY;
        tSize f32Roll;  //Rotation um Mittelachse (Hinten->Vorne-Achse) 
        tSize f32Pitch; //Rotation um Querachse (Seite->Seiten-Achse)
        tSize f32Yaw;   //Rotation um Aufwärtachse (Boden->Decken-Achse)
        tSize f32CarSpeed;
        tSize f32Radius;

        ID(){
            ui32ArduinoTimestamp = 0;
            f32PosX = 0;
            f32PosY = 0;
            f32Roll = 0;
            f32Pitch = 0;
            f32Yaw = 0;
            f32CarSpeed = 0;
            f32Radius = 0;
        }
    };
#pragma pack(pop)

    ID index;

public:

#pragma pack(push,1)
    struct Data
    {
        tUInt32 ui32ArduinoTimestamp;
        tFloat32 f32PosX;
        tFloat32 f32PosY;
        tFloat32 f32Roll;
        tFloat32 f32Pitch;
        tFloat32 f32Yaw;
        tFloat32 f32CarSpeed;
        tFloat32 f32Radius;

        Data() {
            ui32ArduinoTimestamp = 0;
            f32PosX = 0;
            f32PosY = 0;
            f32Roll = 0;
            f32Pitch = 0;
            f32Yaw = 0;
            f32CarSpeed = 0;
            f32Radius = 0;
        }
    };
#pragma pack(pop)


    TPoseStruct()
    {
        type_name = cString("tPoseStruct");
        // get streamtype from aadc.description file
        if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(type_name, object, factory)))
        {

            // get timestamp
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui32ArduinoTimestamp"),
                                                              index.ui32ArduinoTimestamp)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui32Timestamp", type_name));
            }

            // get f32PosX
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32PosX"), index.f32PosX)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32PosX", type_name));
            }
            // get f32PosY
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32PosY"), index.f32PosY)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32PosY", type_name));
            }
            // get f32Roll
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32Roll"), index.f32Roll)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32Roll", type_name));
            }
            // get f32Pitch
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32Pitch"), index.f32Pitch)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32Pitch", type_name));
            }
            // get f32Yaw
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32Yaw"), index.f32Yaw)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32Yaw", type_name));
            }
            // get f32CarSpeed
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32CarSpeed"), index.f32CarSpeed)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32CarSpeed", type_name));
            }
            // get f32Radius
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32Radius"), index.f32Radius)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32Radius", type_name));
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



            if(IS_FAILED(res = oCodec.SetElementValue(index.ui32ArduinoTimestamp, content.ui32ArduinoTimestamp)))
            {

                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.ui32ArduinoTimestamp for index %d", type_name, index.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            }

            if(IS_FAILED(res = oCodec.SetElementValue(index.f32PosX, content.f32PosX)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32PosX for index %d",type_name, index.f32PosX));
                RETURN_ERROR(res);
            }

            if(IS_FAILED(res = oCodec.SetElementValue(index.f32PosY, content.f32PosY)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32PosY for index %d",type_name, index.f32PosY));
                RETURN_ERROR(res);
            }

            if(IS_FAILED(res = oCodec.SetElementValue(index.f32Roll, content.f32Roll)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32Roll for index %d",type_name, index.f32Roll));
                RETURN_ERROR(res);
            }

            if(IS_FAILED(res = oCodec.SetElementValue(index.f32Pitch, content.f32Pitch)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32Pitch for index %d",type_name, index.f32Pitch));
                RETURN_ERROR(res);
            }

            if(IS_FAILED(res = oCodec.SetElementValue(index.f32Yaw, content.f32Yaw)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32Yaw for index %d",type_name, index.f32Yaw));
                RETURN_ERROR(res);
            }

            if(IS_FAILED(res = oCodec.SetElementValue(index.f32CarSpeed, content.f32CarSpeed)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32CarSpeed for index %d",type_name, index.f32CarSpeed));
                RETURN_ERROR(res);
            }

            if(IS_FAILED(res = oCodec.SetElementValue(index.f32Radius, content.f32Radius)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32Radius for index %d",type_name, index.f32Radius));
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
                LOG_RESULT(res);
                RETURN_ERROR(res);
            } 

            // retrieve the values (using convenience methods that return a variant)

            if(IS_FAILED(res = oDecoder.GetElementValue(index.ui32ArduinoTimestamp, &(content.ui32ArduinoTimestamp))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.ui32ArduinoTimestamp for index %d", type_name, index.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            } else if(lastTimestamp == content.ui32ArduinoTimestamp){
				RETURN_ERROR(ERR_FAILED);
			}

            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32PosX, &(content.f32PosX))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32PosX for index %d", type_name, index.f32PosX));
                RETURN_ERROR(res);
            }

            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32PosY, &(content.f32PosY))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32PosY for index %d", type_name, index.f32PosY));
                RETURN_ERROR(res);
            }

            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32Roll, &(content.f32Roll))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32Roll for index %d", type_name, index.f32Roll));
                RETURN_ERROR(res);
            }

            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32Pitch, &(content.f32Pitch))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32Pitch for index %d", type_name, index.f32Pitch));
                RETURN_ERROR(res);
            }

            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32Yaw, &(content.f32Yaw))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32Yaw for index %d", type_name, index.f32Yaw));
                RETURN_ERROR(res);
            }

            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32CarSpeed, &(content.f32CarSpeed))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32CarSpeed for index %d", type_name, index.f32CarSpeed));
                RETURN_ERROR(res);
            }

            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32Radius, &(content.f32Radius))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32Radius for index %d", type_name, index.f32Radius));
                RETURN_ERROR(res);
            }
        }
        else
        {
           // LOG_WARNING("readPin: getLastSample did not work!");
            RETURN_ERROR(res);
        }

        RETURN_NOERROR;

    }



};

