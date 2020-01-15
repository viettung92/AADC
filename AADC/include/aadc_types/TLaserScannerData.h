// ------------------------------------
// &Author: Xiangfei &Datum: 22.08.2018 &output array could have problems - not tested
//-------------------------------------
#pragma once

#include "aadc_type_interface.h"
#include "aadc_types/TPolarCoordiante.h"


class TLaserScannerData: public AADC_Type {


#pragma pack(push,1)
    struct ID
    {
        tSize ui32Size;
        tSize tScanArray;

        ID(){
            ui32Size = 0;
            tScanArray = 1;
        }
    };
#pragma pack(pop)
    ID index;

public:

#pragma pack(push,1)
    struct Data
    {
        tUInt32 ui32Size;
        TPolarCoordiante::Data tScanArray[360];

        Data() {
            ui32Size = 0;
        }
    };
#pragma pack(pop)


    TLaserScannerData(){
        type_name = cString("tLaserScannerData");
        // get streamtype from aadc.description file
        if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(type_name, object, factory)))
        {
            // -----------------------ui32Size------------------------------------------------
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui32Size"), index.ui32Size)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui32Size", type_name));
            }

            // -----------------------tScanArray------------------------------------------------
            //if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("tScanArray") , index.tScanArray )))
	    if(IS_FAILED(adtf_ddl::access_element::find_array_index(factory, cString("tScanArray") , index.tScanArray )))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.tScanArray", type_name));
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
			//content.ui32ArduinoTimestamp = streamTime;

            if(IS_FAILED(res = oCodec.SetElementValue(index.ui32Size, content.ui32Size)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.ui32Size for index %d",type_name, index.ui32Size));
                RETURN_ERROR(res);
            }
            // get pointer to translation vector in sample
            int size = content.ui32Size;
            TPolarCoordiante::Data tPC[size];
            for(int i = 0; i < size; ++i ){
                tPC[i].f32Radius = static_cast<tFloat32>(content.tScanArray[i].f32Radius);
                tPC[i].f32Angle = static_cast<tFloat32>(content.tScanArray[i].f32Angle);
            }
            tFloat32 *tVecSample = static_cast<tFloat32*>(oCodec.GetElementAddress(index.tScanArray));
            memcpy(tVecSample, tPC, size);
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
            if(IS_FAILED(res = oDecoder.GetElementValue(index.ui32Size, &(content.ui32Size))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.ui32Size for index %d", type_name, index.ui32Size));
                RETURN_ERROR(res);
            }
            const TPolarCoordiante::Data* pCoordinates = reinterpret_cast<const TPolarCoordiante::Data*>(oDecoder.GetElementAddress(index.tScanArray));

            for(tSize i = 0; i < content.ui32Size; ++i)
            {
                content.tScanArray[i].f32Radius = pCoordinates[i].f32Radius;
                content.tScanArray[i].f32Angle  = pCoordinates[i].f32Angle ;
            }
            //LOG_WARNING("readPin: work!");


        }
        else
        {
            LOG_WARNING(cString::Format("readPin: getLastSample did not work! No valid timestamp submitted : %d",lastTimestamp));
            RETURN_ERROR(res);
        }

        RETURN_NOERROR;

    }



};

