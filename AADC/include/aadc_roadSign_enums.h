/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-11 16:38:06#$ $Rev:: 63081   $
added by Xiangfei for filter MarkerDetector, without modification
**********************************************************************/


#ifndef ROADSING_ENUMS_H
#define ROADSING_ENUMS_H

namespace roadsignIDs
{
    /*! Values that represent road sign Identifiers. */
    enum roadSignID
    {
        MARKER_ID_UNMARKEDINTERSECTION = 0,
        MARKER_ID_STOPANDGIVEWAY = 1,
        MARKER_ID_PARKINGAREA = 2,
        MARKER_ID_HAVEWAY=3,
        MARKER_ID_AHEADONLY=4,
        MARKER_ID_GIVEWAY=5,
        MARKER_ID_PEDESTRIANCROSSING=6,
        MARKER_ID_ROUNDABOUT=7,
        MARKER_ID_NOOVERTAKING=8,
        MARKER_ID_NOENTRYVEHICULARTRAFFIC=9,
		MARKER_ID_TESTCOURSEA9=10,
        MARKER_ID_ONEWAYSTREET=11,
        MARKER_ID_ROADWORKS=12,
        MARKER_ID_KMH50=13,
        MARKER_ID_KMH100=14,
        
        // Open Chanllenge 1
        MARKER_ID_OC_18=18,
        MARKER_ID_OC_19=19,
        
        // Open Chanllenge 2
        MARKER_ID_OC_20=20,
        MARKER_ID_OC_21=21,
        
        
        // Open Chanllenge 3
        MARKER_ID_OC_22=22,
        MARKER_ID_OC_23=23,
        
        // Open Chanllenge 4
        MARKER_ID_OC_24=24,
        MARKER_ID_OC_25=25,
        
        
        
        
        
        
        
        MARKER_ID_NOMATCH = 99
    };

    const int NUM_ROADSIGNS = 16;
}


#endif 
