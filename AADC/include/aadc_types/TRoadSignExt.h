// ------------------------------------
// &Author: Xiangfei &Datum: 22.08.2018 &output array could have problems - not tested
//-------------------------------------
#pragma once

#include "aadc_type_interface.h"


class TRoadSignExt: public AADC_Type {
#pragma pack(push,1)
    struct ID
    {

		tSize ui32ArduinoTimestamp;
        tSize i16Identifier;
        tSize f32Imagesize;
        tSize af32TVec;
        tSize af32RVec;

        ID(){
			
			ui32ArduinoTimestamp = 0;
            i16Identifier = 0;
            f32Imagesize = 0;
            af32TVec = 0;
            af32RVec = 0;
        }
    };
#pragma pack(pop)




    ID index;

public:

#pragma pack(push,1)
    struct Data
    {

		
		tUInt32 ui32ArduinoTimestamp;
        tInt16 i16Identifier;
        tFloat32 f32Imagesize;
        tFloat32 af32TVec[3];
        tFloat32 af32RVec[3];

        Data() {

            ui32ArduinoTimestamp = 0;
            i16Identifier = 0;
            f32Imagesize = 0;
            af32TVec[0] = 0;
            af32TVec[1] = 0;
            af32TVec[2] = 0;
            af32RVec[0] = 0;
            af32RVec[1] = 0;
            af32RVec[2] = 0;
        }
    };
#pragma pack(pop)


    TRoadSignExt(){
        type_name = cString("tRoadSignExt");
        // get streamtype from aadc.description file
        if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(type_name, object, factory)))
        {
			// -----------------------ui32ArduinoTimestamp------------------------------------------------

            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui32ArduinoTimestamp"), index.ui32ArduinoTimestamp)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui32ArduinoTimestamp", type_name));
            }

            // -----------------------i16Identifier------------------------------------------------
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("i16Identifier"), index.i16Identifier)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.i16Identifier", type_name));
            }

            // -----------------------f32Imagesize------------------------------------------------
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32Imagesize") , index.f32Imagesize )))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32Imagesize", type_name));
            }

            // -----------------------af32TVec------------------------------------------------
            if(IS_FAILED(adtf_ddl::access_element::find_array_index(factory, cString("af32TVec"), index.af32TVec)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.af32TVec", type_name));
            }

            // -----------------------af32RVec------------------------------------------------
            if(IS_FAILED(adtf_ddl::access_element::find_array_index(factory, cString("af32RVec"), index.af32RVec)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.af32RVec", type_name));
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
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.ui32ArduinoTimestamp for index %d",type_name, index.ui32ArduinoTimestamp));
                RETURN_ERROR(res);
            }

            if(IS_FAILED(res = oCodec.SetElementValue(index.i16Identifier, content.i16Identifier)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.i16Identifier for index %d",type_name, index.i16Identifier));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.f32Imagesize, content.f32Imagesize)))
            {

                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.f32Imagesize for index %d", type_name, index.f32Imagesize));
                RETURN_ERROR(res);
            }
            // following code is taken from aadcDemo/markers/marker_detector.cpp
            // get pointer to translation vector in sample
            int t_size = 3;
            tFloat32 *tVecSample = static_cast<tFloat32*>(oCodec.GetElementAddress(index.af32TVec));
            memset(tVecSample, 0, t_size * sizeof(tFloat32));
            for(int i = 0; i < t_size; ++i ){
                tVecSample[i] = static_cast<tFloat32>(content.af32TVec[i]);
            }

            // get pointer to rotation vector in sample
            int r_size = 3;
            tFloat32 *rVecSample = static_cast<tFloat32*>(oCodec.GetElementAddress(index.af32RVec));
            memset(rVecSample, 0, r_size * sizeof(tFloat32));
            for(int i = 0; i < r_size; ++i ){
                rVecSample[i] = static_cast<tFloat32>(content.af32RVec[i]);
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
                LOG_WARNING("DECODER CREATION FAILED*****!");
                RETURN_ERROR(res);
            }
			
            // retrieve the values (using convenience methods that return a variant)
            if(IS_FAILED(res = oDecoder.GetElementValue(index.ui32ArduinoTimestamp, &(content.ui32ArduinoTimestamp))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.ui32ArduinoTimestamp for index %d", type_name, index.ui32ArduinoTimestamp));
                RETURN_ERROR(res);

            }else if (lastTimestamp == content.ui32ArduinoTimestamp)
            {
				//LOG_WARNING("ROADSIGN TIMESTAMP = 0!");
				RETURN_ERROR(ERR_FAILED);
			}
			
            

            if(IS_FAILED(res = oDecoder.GetElementValue(index.i16Identifier, &(content.i16Identifier))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.i16Identifier for index %d", type_name, index.i16Identifier));
                RETURN_ERROR(res);
            }
            
            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32Imagesize, &(content.f32Imagesize))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32Imagesize for index %d", type_name, index.f32Imagesize));
                RETURN_ERROR(res);
            }

            const tFloat32* pTVec = reinterpret_cast<const tFloat32*>(oDecoder.GetElementAddress(index.af32TVec));
            const tFloat32* pRVec = reinterpret_cast<const tFloat32*>(oDecoder.GetElementAddress(index.af32RVec));

            tSize arrSize = 3;
            for(tSize i = 0; i < arrSize; ++i)
            {
                content.af32TVec[i] = pTVec[i];
                content.af32RVec[i] = pRVec[i];
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

