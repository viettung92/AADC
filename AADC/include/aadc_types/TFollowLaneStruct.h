#pragma once

#include "aadc_type_interface.h"


class TFollowLaneStruct: public AADC_Type 
{


	#pragma pack(push,1)
	struct ID
	{
		tSize ui32ArduinoTimestamp;
		tSize f32DistanceToLane;
		tSize f32DesiredSpeed;
		tSize i32RoadSegment;
		
		ID() {
			ui32ArduinoTimestamp = 0;
			f32DistanceToLane = 0;
			f32DesiredSpeed = 0;
			i32RoadSegment = 0;
		
		}
	};
	#pragma pack(pop)

    ID index;
	
public:

	#pragma pack(push,1)
	struct Data
	{
		tUInt32 ui32ArduinoTimestamp;
		tFloat32 f32DistanceToLane;
		tFloat32 f32DesiredSpeed;
		tInt32 i32RoadSegment;
		
		Data() 
	{
			ui32ArduinoTimestamp = 0;
			f32DistanceToLane = 0;
			f32DesiredSpeed = 0;
			i32RoadSegment = 0;
		}
	};
	#pragma pack(pop)

    TFollowLaneStruct()
	{
        type_name = cString("tFollowLaneStruct");
        // get streamtype from aadc.description file
        if(IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(type_name, object, factory)))
        {
        
            // get timestamp
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("ui32ArduinoTimestamp"),
                                                                    index.ui32ArduinoTimestamp)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.ui32Timestamp", type_name));
            }
            
            // get value
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32DistanceToLane"), index.f32DistanceToLane)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32DistanceToLane", type_name));
            }

            // get timestamp
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("f32DesiredSpeed"),
                                                                    index.f32DesiredSpeed)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.f32DesiredSpeed", type_name));
            }

            // get timestamp
            if(IS_FAILED(adtf_ddl::access_element::find_index(factory, cString("i32RoadSegment"),
                                                                    index.i32RoadSegment)))
            {
                LOG_WARNING(cString::Format("COULD NOT FIND INDEX: %s.i32RoadSegment", type_name));
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
            
            if(IS_FAILED(res = oCodec.SetElementValue(index.f32DistanceToLane, content.f32DistanceToLane)))
            {

                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.f32DistanceToLane for index %d", type_name, index.f32DistanceToLane));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.f32DesiredSpeed, content.f32DesiredSpeed)))
            {
                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %d.f32DesiredSpeed for index %d",type_name, index.f32DesiredSpeed));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oCodec.SetElementValue(index.i32RoadSegment, content.i32RoadSegment)))
            {

                LOG_WARNING(cString::Format("COULD NOT SET ELEMENT VALUE: %s.i32RoadSegment for index %d", type_name, index.i32RoadSegment));
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
            
            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32DistanceToLane, &(content.f32DistanceToLane))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32DistanceToLane for index %d", type_name, index.f32DistanceToLane));
                RETURN_ERROR(res);
            }
            
            if(IS_FAILED(res = oDecoder.GetElementValue(index.f32DesiredSpeed, &(content.f32DesiredSpeed))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.f32DesiredSpeed for index %d", type_name, index.f32DesiredSpeed));
                RETURN_ERROR(res);
            }
            if(IS_FAILED(res = oDecoder.GetElementValue(index.i32RoadSegment, &(content.i32RoadSegment))))
            {
                LOG_WARNING(cString::Format("COULD NOT GET ELEMENT VALUE: %s.i32RoadSegment for index %d", type_name, index.i32RoadSegment));
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
