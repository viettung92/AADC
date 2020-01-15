/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**********************************************************************
* This filter limits output speed and steering.

* $Adapted by:: Xiangfei#  $Date:: 2018-08-01 12:44:00# status: adapted, not tested
**********************************************************************/

#pragma once

#define CID_SCM_JURY_COMMUNICATION_FILTER "scm_jury_communication.filter.user.aadc.cid"
#include "stdafx.h"
#include "DriverModuleWidget.h"

// namespace
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

#include "ADTF3_helper.h"
#include "stdafx.h"
#include <boost/thread.hpp>
#include <a_utils/core/a_utils_core.h>

// ScmJuryCommunication
class ScmJuryCommunication : public cTriggerFunction
{
private:
    /*------------ STRUCTS -------------*/
    // create structs to hold information we get from SignalValues
    // coding convention: 	o_******Id

    TActionStruct o_TActionStruct;
    TFeedbackStruct o_TFeedbackStruct;

    iJuryStruct m_JuryStructId;
    iDriverStruct m_DriverStructId;

    /*! Sample factory */
    adtf::mediadescription::cSampleCodecFactory m_jurySampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_driverSampleFactory;

    /*-------------- PINS --------------*/
    // create pins for input and output
    // coding convention:	m_Reader******	for input  signals
    //			m_Writer******	for output signals

    // input pins
    cPinReader m_ReaderJuryStruct;
    cPinReader m_ReaderManeuverList;
    cPinReader m_ReaderActionStruct;


    // output pins
    cPinWriter m_WriterDriverStruct;
    cPinWriter m_WriterFeedbackStruct;

    /*------------ PROPERTIES ----------*/
    // set property variables
    // coding convention:	m_prop******
    adtf::base::property_variable<tBool>    m_propBDebugToConsole        = tFalse;
    adtf::base::property_variable<tBool>    m_propBDebugSendStateOutput        = tFalse;
    adtf::base::property_variable<tBool>    m_propBDebugPrintStructure        = tFalse;
    /*! The property TCP port */
        adtf::base::property_variable<tInt>    m_propTCPPort = 1234;
    /*------------ VARIABLES -----------*/
    // coding convention:	m_******
    object_ptr<adtf::services::IReferenceClock> m_pClock; // clock for timestamps
//    object_ptr< adtf::services::IKernel > m_pKernel;
    tJuryStruct m_dataJuryStructIn;
    tDriverStruct m_dataDriverStructOut;
    TActionStruct::Data m_dataActionStructIn;
    TFeedbackStruct::Data m_dataFeedbackStructOut;


    /* Variables and descriptions necessary for input/outputs */
    /* Coder description for maneuver list */
//    cObjectPtr<IMediaTypeDescription> m_pDescManeuverList;


    /*! indicates if bufferIDs were set */
    tBool m_bIDsJuryStructSet;


    tBool m_bIDsDriverStructSet;




/** Variables, parameters and other necessary declarations **/

    /*! whether output to console is enabled or not*/
    tBool m_bDebugModeEnabled;
    /*! whether output of structure to console is enabled or not*/
    tBool m_bPrintStrucModeEnabled;
    /* whether output of method 'sendState' is printed to console (cyclic!) */
    tBool m_bDebugSendStateModeEnabled;

    /* bool showing whether all maneuvers have been successfully completed */
    tBool m_bCompletedJuryList;

    /* flag expressing if jury-START-signal was already sent */
    tBool m_startSignalreceived;
    /* flag expressing if jury-START-signal was already requested */
    tBool m_startSignalrequested;
    /* flag expressing if maneuver list was already sent */
    tBool m_maneuverListLoaded;
    /* flag expressing if first maneuver was already sent */
    tBool m_maneuverFirstTransmitrequested;

    /** LOADING and saving maneuver list **/

    /*! this is the filename of the maneuver list*/
    cString m_strManeuverFileString;

    /*! this is the list with all the loaded sections from the maneuver list*/
    std::vector<tSector> m_sectorList;

    /** Managing the states of the Car via List **/
    /*! holds the current state of the car */
    stateCar m_CarState;

    /*! holds the (global) current maneuver id of the car*/
    tInt16 m_i16CurrentManeuverID;

    /*! holds the current index of the maneuvers in the list in the section */
    tInt16 m_i16ManeuverListIndex;

    /*! holds the current index in the lists of sections */
    tInt16 m_i16SectionListIndex;

//    /*! handle for the timer */
//    tHandle m_hTimer;

    /*! variable to temporarily save the previous, already successfully executed maneuver */
    tUInt32 tmpPrevManeuverInt_fb;




    /** Declaration of Critical Section Parameters **/
    /* Critical Section for Output Pins */
    /*! the critical section of the transmit of carState*/
    boost::mutex m_oCriticalSectionTransmitCarState;
    /*! the critical section of the transmit of ActionStruct */
    boost::mutex m_oCriticalSectionTransmitFeedbackStruct;
    /* Critical Section for Timer Setup */
    boost::mutex m_oCriticalSectionTimerSetup;
    /* Critical Section for CarState changes*/
    boost::mutex m_oCriticalSectionCarStatus;
    /* Critical Section for ManeuverList */
    boost::mutex m_oCriticalSectionManeuverList;
    /* Critical Section for BoolValue expressing whether maneuver list was already Loaded */
    boost::mutex m_oCriticalSectionManListLoaded;
    /* Critical Section for BoolValue expressing whether first transmit was requested */
    boost::mutex m_oCriticalSectionFirstTransmitReq;
    /* Critical Section for BoolValue expressing whether first transmit was requested */
    boost::mutex m_oCriticalSectionStartSignalReq;
    /* Critical Section for BoolValue expressing whether first transmit was requested */
    boost::mutex m_oCriticalSectionStartSignalReceived;
    /* Critical Section for BoolValue expressing whether jury maneuver list was completed */
    boost::mutex m_oCriticalSectionCompletedJuryList;
    /* Critical Section for accessing info regarding the previously executed maneuver */
    boost::mutex m_oCriticalSectionPreviousManeuverAccess;

 // init TimeStamps
 tUInt32 m_ui32LastTimeStampAction;
public:
    /*------------ FUNCTIONS -----------*/
    // constructor
    ScmJuryCommunication();

    // destructor
    ~ScmJuryCommunication() override;

    // configure
    virtual tResult Configure() override;

    // process
    virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;

//    //// from 2018
//    tResult OnSendState(aadc::jury::stateCar stateID, tInt16 i16ManeuverEntry);
//    /*! this functions loads the maneuver list given in the properties
//    * \result Returns a standard result code.
//    */
//    //// from 2018
//    tResult LoadManeuverList();

    //// below from 2018
    /*! The mutex */
    std::mutex m_oMutex;

    /*! The server socket */
    cServerSocket m_serverSocket;
    /*! The stream socket */
    cStreamSocket m_streamSocket;
    /*! The client connection established */
    tBool m_clientConnectionEstablished;
    virtual tResult  OnTimer();

    ::adtf_util::cResult ReceiveTCPData(std::vector<tChar>& data);


    //// below from 2017

    /*! signal for sending the state
    @param i8StateID state to be sent; -2: Startup, -1: Error, 0: Ready, 1: Running, 2: Complete
    @param i16ManeuverEntry current entry to be sent
    */
    tResult SendState(stateCar state, tInt16 i16ManeuverEntry);

    /*! creates the timer for the cyclic transmits*/
    tResult createTimer();

   /*! destroys the timer for the cyclic transmits*/
    tResult destroyTimer();

    /* increments the id of the maneuver id by one and updates the list indexes;
     *  returns retval < 0 in case of error, retval = 0 for normal behaviour, retval > 0 for end of list reached */
    tInt32 incrementManeuverID();

    /*! resets the counters to the start of the current section*/
    tResult resetSection();

   /*! changes the state of the car
    @param newState the new state of the car
    */
    tResult changeState(stateCar newState);
    /* returns the current state of the car */
    stateCar getCarState();

    /*! set the maneuver id and find the correct indexes
    @param maneuverId the id of the maneuver which has to be set*/
    tResult setManeuverID(tInt maneuverId);

    /* returns the current maneuverID index*/
    tInt16 getCurManeuverID();

    /*! this functions loads the maneuver list given in the properties*/
    tResult loadManeuverList();

    /* returns the current maneuver from the maneuverList sent from jury, as uint32 (refer to enumeration in StateControl.h) */
    tUInt32 getCurrentManeuverFromJuryList();

    /* returns the finishing maneuver from the maneuverList sent from jury, that means the maneuver that was executed
     * as the last maneuver on the list, as uint32 (refer to enumeration in StateControl.h) */
    tUInt32 getFinishingManeuverFromJuryList();

    /* decodes the jury-maneuver in 'cString' format and returns the corresponding predefined tUInt32 (refer to enumeration in StateControl.h)*/
    tUInt32 decodeManeuverFromJuryList(cString tmp_currJuryManeuver);

    /* Processes the received ActionStruct from SCM */
    tResult ProcessActionData(TActionStruct::Data  );

    /* Function to transmit the Feedback to SCM */
    tResult TransmitFeedbackStruct(TFeedbackStruct::Data feedbackStruct);

    /* Function to get and set flag whether maneuver list was already loaded */
    tBool getFlagManeuverListLoaded();
    tResult setFlagManeuverListLoaded(tBool value);
    /* Function to get and set flag whether first transmit of maneuver was requested */
    tBool getFlagFirstTransmitRequested();
    tResult setFlagFirstTransmitRequested(tBool value);

    /* Function to get and set flag whether start signal from jury was requested */
    tBool getFlagStartSignalRequested();
    tResult setFlagStartSignalRequested(tBool value);
    /* Function to get and set flag whether start signal from jury was received */
    tBool getFlagStartSignalReceived();
    tResult setFlagStartSignalReceived(tBool value);
    /* Function to get and set flag whether jury list was completed */
    tBool getFlagCompletedJuryList();
    tResult setFlagCompletedJuryList(tBool value);

    /* Function to change or access information regarding the previously executed maneuver */
     tUInt32 getPreviousManeuver();
     tResult setPreviousManeuver(tUInt32 maneuver);
     tResult resetPreviousManeuver();
//     virtual tVoid Destroy ()const;
}; // ScmJuryCommunication
