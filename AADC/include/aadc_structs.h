/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#ifndef AADC_STRUCTS_H
#define AADC_STRUCTS_H
#include "aadc_user_structs.h"

#pragma pack(push,1)
typedef struct
{
    tInt16 i16ActionID;
    tInt16 i16ManeuverEntry;
} tJuryStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tInt16 i16StateID;
    tInt16 i16ManeuverEntry;
} tDriverStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tUInt32 ui32ArduinoTimestamp;
    tFloat32 f32Value;
} tSignalValue;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tUInt32 ui32ArduinoTimestamp;
    tBool bValue;
} tBoolSignalValue;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tUInt32 ui32ArduinoTimestamp;
    tUInt32 ui32WheelTach;
    tInt8 i8WheelDir;
} tWheelData;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
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


} tInerMeasUnitData;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tUInt32 ui32ArduinoTimestamp;
    tInt16 i16Identifier;
    tFloat32 f32Imagesize;
    tFloat32 af32TVec[3];
    tFloat32 af32RVec[3];
} tRoadSignExt;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tFloat32 f32x;
    tFloat32 f32y;
    tFloat32 f32radius;
    tFloat32 f32speed;
    tFloat32 f32heading;
} tPosition;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tFloat32 f32x;
    tFloat32 f32y;
} tObstacle;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tInt16 i16Identifier;
    tFloat32 f32x;
    tFloat32 f32y;
    tFloat32 f32angle;
} tTrafficSign;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tInt16 i16Identifier;
    tFloat32 f32x;
    tFloat32 f32y;
    tUInt16 ui16Status;
} tParkingSpace;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSignalValue tSideLeft;
    tSignalValue tSideRight;
    tSignalValue tRearLeft;
    tSignalValue tRearCenter;
    tSignalValue tRearRight;
} tUltrasonicStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSignalValue tActuatorVoltage;
    tSignalValue tActuatorCell1;
    tSignalValue tActuatorCell2;
    tSignalValue tSensorVoltage;
    tSignalValue tSensorCell1;
    tSignalValue tSensorCell2;
    tSignalValue tSensorCell3;
    tSignalValue tSensorCell4;
    tSignalValue tSensorCell5;
    tSignalValue tSensorCell6;
} tVoltageStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tFloat32 f32Radius;
    tFloat32 f32Angle;
} tPolarCoordiante;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tUInt32 ui32Size;
    tPolarCoordiante tScanArray[360];
} tLaserScannerData;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
   tUInt32 ui32ArduinoTimestamp;
   tFloat32 f32PosX;
   tFloat32 f32PosY;
   tFloat32 f32Roll;
   tFloat32 f32Pitch;
   tFloat32 f32Yaw;
   tFloat32 f32CarSpeed;
   tFloat32 f32Radius;
} tPoseStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
  tInt32 nWidth;
  tInt32 nHeight;
  tInt16 nBitsPerPixel;
  tInt16 nPixelFormat;
  tInt32 nBytesPerLine;
  tInt32 nSize;
  tInt32 nPaletteSize;
} tBitmapFormat;
#pragma pack(pop)

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// The following types are assumed to be known:
// tInt8
// tInt16
// tUInt32
// tFloat32
// tBool
// tUInt16



#pragma pack(push,1)
typedef struct
{
    tSize i16ActionID;
    tSize i16ManeuverEntry;
} iJuryStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSize i16StateID;
    tSize i16ManeuverEntry;
} iDriverStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSize ui32ArduinoTimestamp;
    tSize f32Value;
} iSignalValue;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSize ui32ArduinoTimestamp;
    tSize bValue;
} iBoolSignalValue;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSize ui32ArduinoTimestamp;
    tSize ui32WheelTach;
    tSize i8WheelDir;
} iWheelData;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSize ui32ArduinoTimestamp;
    tSize f32APoseX;
    tSize f32APoseY;
    tSize f32APoseZ;
    tSize f32GPoseX;
    tSize f32GPoseY;
    tSize f32GPoseZ;
    tSize f32MPoseX;
    tSize f32MPoseY;
    tSize f32MPoseZ;
	/*! the roll angle in degree */
    tFloat32 f32Roll;
    /*! the pitch angle in degree */
    tFloat32 f32Pitch;
    /*! the yaw angle in degree */
    tFloat32 f32Yaw;
} iInerMeasUnitData;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSize ui32ArduinoTimestamp;
    tSize i16Identifier;
    tSize f32Imagesize;
    tSize af32TVec;
    tSize af32RVec;
} iRoadSignExt;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSize f32x;
    tSize f32y;
    tSize f32radius;
    tSize f32speed;
    tSize f32heading;
} iPosition;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSize f32x;
    tSize f32y;
} iObstacle;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSize i16Identifier;
    tSize f32x;
    tSize f32y;
    tSize f32angle;
} iTrafficSign;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSize i16Identifier;
    tSize f32x;
    tSize f32y;
    tSize ui16Status;
} iParkingSpace;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    iSignalValue tSideLeft;
    iSignalValue tSideRight;
    iSignalValue tRearLeft;
    iSignalValue tRearCenter;
    iSignalValue tRearRight;
} iUltrasonicStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSize tActuatorVoltage;
    tSize tActuatorCell1;
    tSize tActuatorCell2;
    tSize tSensorVoltage;
    tSize tSensorCell1;
    tSize tSensorCell2;
    tSize tSensorCell3;
    tSize tSensorCell4;
    tSize tSensorCell5;
    tSize tSensorCell6;
} iVoltageStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSize f32Radius;
    tSize f32Angle;
} iPolarCoordiante;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSize ui32Size;
    tSize tScanArray;
} iLaserScannerData;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
   tSize ui32ArduinoTimestamp;
   tSize f32PosX;
   tSize f32PosY;
   tSize f32Roll;
   tSize f32Pitch;
   tSize f32Yaw;
   tSize f32CarSpeed;
   tSize f32Radius;
} iPoseStruct;
#pragma pack(pop)


#pragma pack(push,1)
typedef struct
{
  tSize nWidth;
  tSize nHeight;
  tSize nBitsPerPixel;
  tSize nPixelFormat;
  tSize nBytesPerLine;
  tSize nSize;
  tSize nPaletteSize;
} iBitmapFormat;
#pragma pack(pop)


#endif
