// ------------------------------------
// &Author: Nico &Datum: 27.08.2018
//-------------------------------------

#pragma once

#include "aadc_type_interface.h"

class TInerMeasureData: public AADC_Type
{

#pragma pack(push,1)
    struct ID
    {
            tSize ui32ArduinoTimestamp;

            tSize f32APosX;
            tSize f32APosY;
            tSize f32APosZ;

            tSize f32GPosX;
            tSize f32GPosY;
            tSize f32GPosZ;

            tSize f32MPosX;
            tSize f32MPosY;
            tSize f32MPosZ;

            tSize f32Roll;
            tSize f32Pitch;
            tSize f32Yaw;

           ID()
           {

                ui32ArduinoTimestamp = 0;

                f32APosX = 0;
                f32APosY = 0;
                f32APosZ = 0;

                f32GPosX = 0;
                f32GPosY = 0;
                f32GPosZ = 0;

                f32MPosX = 0;
                f32MPosY = 0;
                f32MPosZ = 0;

                f32Roll = 0;
                f32Pitch = 0;
                f32Yaw = 0;
           }
    };
#pragma pack(pop)
    ID index;

public:

#pragma pack(push,1)
    struct Data
    {
        tUInt32 ui32ArduinoTimestamp;

        tFloat32 f32APosX;
        tFloat32 f32APosY;
        tFloat32 f32APosZ;

        tFloat32 f32GPosX;
        tFloat32 f32GPosY;
        tFloat32 f32GPosZ;

        tFloat32 f32MPosX;
        tFloat32 f32MPosY;
        tFloat32 f32MPosZ;

        tFloat32 f32Roll;
        tFloat32 f32Pitch;
        tFloat32 f32Yaw;

        Data()
        {
             ui32ArduinoTimestamp = 0;

             f32APosX = 0;
             f32APosY = 0;
             f32APosZ = 0;

             f32GPosX = 0;
             f32GPosY = 0;
             f32GPosZ = 0;

             f32MPosX = 0;
             f32MPosY = 0;
             f32MPosZ = 0;

             f32Roll = 0;
             f32Pitch = 0;
             f32Yaw = 0;
        }
    };
#pragma pack(pop)



    TInerMeasureData()
    {
        // TODO: following code block still needed?
        //        ids.f32APosX = 0;
        //        ids.f32APosY = 0;
        //        ids.f32APosZ = 0;

        //        ids.f32GPosX = 0;
        //        ids.f32GPosY = 0;
        //        ids.f32GPosZ = 0;

        //        ids.f32MPosX = 0;
        //        ids.f32MPosY = 0;
        //        ids.f32MPosZ = 0;

        //        ids.f32Roll = 0;
        //        ids.f32Pitch = 0;
        //        ids.f32Yaw = 0;


        type_name = cString("tInerMeasUnitData");
        // get streamtype from aadc.description file
        if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(type_name, object, factory)))
        {

            // get timestamp
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui32ArduinoTimestamp"),
                                                              index.ui32ArduinoTimestamp)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui32ArduinoTimestamp", type_name));
            }
            //------- A Pos --------
            // get f32APosX
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32A_x"), index.f32APosX)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32A_x", type_name));
            }
            // get f32APosY
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32A_y"), index.f32APosY)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32A_y", type_name));
            }
            // get f32APosZ
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32A_z"), index.f32APosZ)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32A_z", type_name));
            }
            //------- G Pos --------
            // get f32GPosX
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32G_x"), index.f32GPosX)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32G_x", type_name));
            }
            // get f32GPosY
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32G_y"), index.f32GPosY)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32G_y", type_name));
            }
            // get f32GPosZ
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32G_z"), index.f32GPosZ)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32G_z", type_name));
            }
            //------- M Pos --------
            // get f32MPosX
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32M_x"), index.f32MPosX)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32M_x", type_name));
            }
            // get f32MPosY
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32M_y"), index.f32MPosY)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32M_y", type_name));
            }
            // get f32MPosZ
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32M_z"), index.f32MPosZ)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32M_z", type_name));
            }

            // get f32Roll
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32roll"), index.f32Roll)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32roll", type_name));
            }
            // get f32Pitch
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32pitch"), index.f32Pitch)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32pitch", type_name));
            }
            // get f32Yaw
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32yaw"), index.f32Yaw)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32yaw", type_name));
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
            // ----- A Pos -----
            if(IS_FAILED(res = oCodec.SetElementValue(index.f32APosX, content.f32APosX)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32APosX for index %d",type_name, index.f32APosX));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.f32APosY, content.f32APosY)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32APosY for index %d",type_name, index.f32APosY));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.f32APosZ, content.f32APosZ)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32APosZ for index %d",type_name, index.f32APosZ));
                RETURN_ERROR(res);
            }
            // ----- G Pos -----
            if(IS_FAILED(res = oCodec.SetElementValue(index.f32GPosX, content.f32GPosX)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32GPosX for index %d",type_name, index.f32GPosX));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.f32GPosY, content.f32GPosY)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32GPosY for index %d",type_name, index.f32GPosY));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.f32GPosZ, content.f32GPosZ)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32GPosZ for index %d",type_name, index.f32GPosZ));
                RETURN_ERROR(res);
            }
            // ----- M Pos -----
            if(IS_FAILED(res = oCodec.SetElementValue(index.f32MPosX, content.f32MPosX)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32MPosX for index %d",type_name, index.f32MPosX));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.f32MPosY, content.f32MPosY)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32MPosY for index %d",type_name, index.f32MPosY));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.f32MPosZ, content.f32MPosZ)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32MPosZ for index %d",type_name, index.f32MPosZ));
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

            if(IS_FAILED(res = oDecoder.GetElementValue(index.ui32ArduinoTimestamp, &(content.ui32ArduinoTimestamp))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.ui32ArduinoTimestamp for index %d", type_name, index.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            } else if(lastTimestamp == content.ui32ArduinoTimestamp){
				RETURN_ERROR(ERR_FAILED);
			}
            // ------ A Pos ------
            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32APosX, &(content.f32APosX))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32APosX for index %d", type_name, index.f32APosX));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32APosY, &(content.f32APosY))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32APosY for index %d", type_name, index.f32APosY));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32APosZ, &(content.f32APosZ))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32APosZ for index %d", type_name, index.f32APosZ));
                RETURN_ERROR(res);
            }
            // ------ G Pos ------
            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32GPosX, &(content.f32GPosX))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32GPosX for index %d", type_name, index.f32GPosX));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32GPosY, &(content.f32GPosY))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32GPosY for index %d", type_name, index.f32GPosY));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32GPosZ, &(content.f32GPosZ))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32GPosZ for index %d", type_name, index.f32GPosZ));
                RETURN_ERROR(res);
            }
            // ------ M Pos ------
            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32MPosX, &(content.f32MPosX))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32MPosX for index %d", type_name, index.f32MPosX));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32MPosY, &(content.f32MPosY))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32MPosY for index %d", type_name, index.f32MPosY));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32MPosZ, &(content.f32MPosZ))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32MPosZ for index %d", type_name, index.f32MPosZ));
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
			input.Clear();
        }
        else
        {
         //   LOG_WARNING("readPin: getLastSample did not work!");
            RETURN_ERROR(res);
        }

        RETURN_NOERROR;

    }

};
