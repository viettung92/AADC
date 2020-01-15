
/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. Team FAUtonomous. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#pragma once

#pragma pack(push,1)
typedef struct
{
    tBool   bEnabled;
    tBool   bStarted;
    tUInt32 ui32Command;
} tActionSub;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tUInt8     ui8FilterId;
    tActionSub subAction;
} tAction;
#pragma pack(pop)

//#pragma pack(push,1)
//typedef struct
//{
//    tAction action1;
//    tAction action2;
//    tAction action3;
//    tAction action4;
//    tAction action5;
//} tActionStruct;
//#pragma pack(pop)


#pragma pack(push,1)
typedef struct
{
        tBool   bEnabled;
        tBool   bStarted;
        tUInt32 ui32Command;
        tUInt8  ui8FilterId;
} tActionStruct;
#pragma pack(pop)


#pragma pack(push,1)
typedef struct
{
    tUInt8  ui8FilterId;
    tUInt32 ui32FeedbackStatus;
} tFeedbackStruct;
#pragma pack(pop)

// --------------- INDEX STRUCTS ---------------

#pragma pack(push,1)
typedef struct
{
    tSize bEnabled;
    tSize bStarted;
    tSize ui32Command;
} iActionSub;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSize      ui8FilterId;
    iActionSub subAction;
} iAction;
#pragma pack(pop)

//#pragma pack(push,1)
//typedef struct
//{
//    iAction action1;
//    iAction action2;
//    iAction action3;
//    iAction action4;
//    iAction action5;
//} iActionStruct;
//#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
        tSize      ui8FilterId;
        tSize bEnabled;
        tSize bStarted;
        tSize ui32Command;
} iActionStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tSize  ui8FilterId;
    tSize  ui32FeedbackStatus;
} iFeedbackStruct;
#pragma pack(pop)

