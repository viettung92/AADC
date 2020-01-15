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

#include <mutex>
#include "stdafx.h"
#include "ScmJuryCommunication.h"
#include "ScmCommunication.h"
//using namespace aadc::jury;
using namespace adtf_util;
#define DEBUG_TO_CONSOLE "Debug::Debug output to console"
#define DEBUG_PRINT_STRUCTURE "Debug::Print Structure to Console"
#define DEBUG_SENDSTATE_OUTPUT "Debug::Print SendState Outputs to Console (cyclic)"
#define SC_PROP_FILENAME "Maneuver File"

// This will define the filter and expose it via plugin class factory.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_SCM_JURY_COMMUNICATION_FILTER,                         // references to header file
        "ScmJuryCommunication",                                   // label
        ScmJuryCommunication,                                      // class
        adtf::filter::pin_trigger({"action_from_scm"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
ScmJuryCommunication::ScmJuryCommunication()
{

    // ------------------------------------------
    SetName("ScmJuryCommunication Constructor");

    // -----------------------------------------
    // set pins
    o_TActionStruct.registerPin(this, m_ReaderActionStruct    , "action_from_scm"     );
    o_TFeedbackStruct.registerPin(this, m_WriterFeedbackStruct   , "feedback_to_scm");

    // -----------------------------------------
    // set property variables
    RegisterPropertyVariable(DEBUG_TO_CONSOLE, m_propBDebugToConsole       );
    RegisterPropertyVariable(DEBUG_SENDSTATE_OUTPUT, m_propBDebugSendStateOutput       );
    RegisterPropertyVariable("Port number", m_propTCPPort);
    RegisterPropertyVariable(DEBUG_PRINT_STRUCTURE, m_propBDebugPrintStructure       );
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult ScmJuryCommunication::Configure()
{
    m_clientConnectionEstablished = tFalse;
    m_bDebugModeEnabled =m_propBDebugToConsole;
    m_bDebugSendStateModeEnabled = m_propBDebugSendStateOutput;
    /* Jury and Maneuver communication*/
    m_i16CurrentManeuverID = 0;
    m_startSignalreceived = tFalse;
    m_maneuverFirstTransmitrequested = tFalse;
    m_maneuverListLoaded = tFalse;
    m_startSignalrequested = tFalse;
    m_bCompletedJuryList = tFalse;
    m_CarState = stateCar_STARTUP;
    /* using the method provides sending the state to the jury if module is already connected */
    //changeState(stateCar_STARTUP);
    tmpPrevManeuverInt_fb = FB_JURY_PREVIOUS_NO_MAN_EXISTING;
    m_bPrintStrucModeEnabled = m_propBDebugPrintStructure;
    /* Jury and Maneuver communication */
    m_i16SectionListIndex = -1;
    m_i16ManeuverListIndex =-1;
    // no ids were set yet
    m_bIDsDriverStructSet = tFalse;
    m_bIDsJuryStructSet = tFalse;

    //????????????
    stateCar tmp_state = getCarState();
    tInt16 tmp_ManID = getCurManeuverID();
    if(m_bDebugSendStateModeEnabled) LOG_WARNING(cString::Format("JCom: TIMER HANDLE CALLED! state is %d, ManID is %d",tmp_state,tmp_ManID));
    SendState(tmp_state,tmp_ManID);

    //????????????
    RETURN_IF_FAILED_DESC(m_serverSocket.Open(m_propTCPPort, cServerSocket::SS_Exclusive),
                          cString::Format("Could not open server socket with port %d", static_cast<tInt>(m_propTCPPort)));
    LOG_INFO(cString::Format("Server Socket was opened with port %d", static_cast<tInt>(m_propTCPPort)));
    RETURN_IF_FAILED_DESC(m_serverSocket.Listen(),
                          cString::Format("Could not listen to port %d",static_cast<tInt>(m_propTCPPort)));
    LOG_INFO(cString::Format("Server Socket now listens on port %d", static_cast<tInt>(m_propTCPPort)));
    if (m_serverSocket.IsConnected(static_cast<tTimeStamp>(5e5)))
    {
        RETURN_IF_FAILED_DESC(m_serverSocket.Accept(m_streamSocket),"Could not access Server socket");
        m_clientConnectionEstablished = tTrue;
        LOG_INFO("TCP Connection was established");
    }
    else
    {
        LOG_ERROR(cString::Format("No client is connected on Port %d", static_cast<tInt>(m_propTCPPort)));
        m_clientConnectionEstablished = tFalse;
    }

    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    LOG_INFO(cString::Format("Configuration finished!"));
    // done
    RETURN_NOERROR;
}




ScmJuryCommunication::~ScmJuryCommunication()
{
    //closes the connections and the server
    m_streamSocket.Close();
    m_serverSocket.Close();
    m_clientConnectionEstablished = tFalse;
}

// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
tResult ScmJuryCommunication::Process(tTimeStamp tmTimeOfTrigger)
{

    // if new data is action signal
    if(IS_OK(o_TActionStruct.readPin(m_ReaderActionStruct, (void *) & m_dataActionStructIn, m_ui32LastTimeStampAction)))
    {
        m_ui32LastTimeStampAction = m_dataActionStructIn.ui32ArduinoTimestamp;

        // process data
        RETURN_IF_FAILED(ProcessActionData(m_dataActionStructIn));
    }

    // done
    RETURN_NOERROR;
}
// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

//// below from 2018
tResult ScmJuryCommunication::OnTimer()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    std::vector<tChar> data;
    RETURN_IF_FAILED(ReceiveTCPData(data));
    const tSize sizeOfJuryStruct = sizeof(tJuryStruct);
    if (data.size() == sizeOfJuryStruct)
    { //jurysruct

        tJuryStruct* juryStruct = (tJuryStruct*) data.data();
        tInt8 i8ActionID = juryStruct->i16ActionID;
        tInt16 i16entry = juryStruct->i16ManeuverEntry;


        //change the state depending on the input
        // action_GETREADY --> stateCar_READY
        // action_START --> stateCar_RUNNING
        // action_STOP --> stateCar_STARTUP
        switch (aadc::jury::juryAction(i8ActionID))
        {
        case action_GETREADY: // no influence on system itself, so no communication with SCM necessary
            if(m_bDebugModeEnabled)  LOG_WARNING(cString::Format("JCom: Received 'Request Ready with maneuver ID %d'",i16entry));
            if(getCarState()==stateCar_STARTUP){
                if(getFlagManeuverListLoaded()){
                    changeState(stateCar_READY);
                    setManeuverID(i16entry);
                }else{
                    LOG_ERROR(cString::Format("JCom: Received 'Request Ready', but maneuver list not loaded yet!"));
                    changeState(stateCar_ERROR);
                }
            }
            else{
                LOG_WARNING(cString::Format("JCom: Received 'Request Ready', but system not in startup state! Thus command will be ignored."));
            }
            break;
        case action_START: // communication with SCM necessary, if button is pressed before awaited from SCM -> saved as flag!
            if(m_bDebugModeEnabled)  LOG_WARNING(cString::Format("JCom: Received: 'Action START with maneuver ID %d'",i16entry));
            /* (re)set previously executed maneuver to 'no_prev_man_existing' */
            resetPreviousManeuver();
            if(getFlagStartSignalReceived() == tFalse){ // false for first time receiving start; set to true afterwards; to false again if STOP is sent
                if (i16entry == getCurManeuverID()){
                    changeState(stateCar_RUNNING);
                }
                else{
                    LOG_WARNING(cString::Format("JCom: The id of the action_START corresponds not with the id of the last action_GETREADY! "
                                                "New ID %d will be used.",i16entry));
                    setManeuverID(i16entry);
                    changeState(stateCar_RUNNING);
                }
                setFlagStartSignalReceived(tTrue);
                if(getFlagStartSignalRequested()){
                    /* Send feedback to SCM if it was requested before */
                    TFeedbackStruct::Data feedback_tmp;
                    feedback_tmp.ui8FilterId = F_JURY_COMMUNICATION;
                    feedback_tmp.ui32FeedbackStatus = FB_JURY_COMMAND_RECEIVED_START;
                    RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));
                    setFlagStartSignalRequested(tFalse);
                    if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: Sent requested feedback to SCM: ID %d, status %d",F_JURY_COMMUNICATION,FB_JURY_COMMAND_RECEIVED_START));
                }
            }
            else{
                if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: Action_START was received, but system already running due to previous start-signal!"));
            }
            break;
        case action_STOP: // communication with SCM over normal feedback for error-handling via Error-Maneuver in SCM
            if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: Received: 'Stop with maneuver ID %d'",i16entry));
            /* (re)set previously executed maneuver to 'no_prev_man_existing' */
            resetPreviousManeuver();
            setFlagStartSignalReceived(tFalse);
            setFlagCompletedJuryList(tFalse);
            changeState(stateCar_STARTUP);
            /* Send feedback to SCM, will have priority-handling; SCM has to stop current maneuver and jump to startup mode, all filters disabled*/
            TFeedbackStruct::Data feedback_tmp;
            feedback_tmp.ui8FilterId = F_JURY_COMMUNICATION;
            feedback_tmp.ui32FeedbackStatus = FB_JURY_COMMAND_RECEIVED_STOP;
            RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));
            break;

            //            case action_getready:
            //                CONSOLE_LOG_INFO(cString::Format("Driver Module: Received Request Ready with maneuver ID %d", i16entry));
            //                emit SendRequestReady(static_cast<int>(i16entry));
            //                break;
            //            case action_start:
            //                CONSOLE_LOG_INFO(cString::Format("Driver Module: Received Run with maneuver ID %d", i16entry));
            //                emit SendRun(static_cast<int>(i16entry));
            //                break;
            //            case action_stop:
            //                CONSOLE_LOG_INFO(cString::Format("Driver Module: Received Stop with maneuver ID %d", i16entry));
            //                emit SendStop(static_cast<int>(i16entry));
            //                break;
        }
    }
    else if (data.size() > 0)
    {//maneuverlist
        m_strManeuverFileString.Set(data.data(),data.size());
        loadManeuverList();
    }

    RETURN_NOERROR;
}


tResult ScmJuryCommunication::ReceiveTCPData(std::vector<tChar>& data)
{
    // no stream connected yet
    if (!m_clientConnectionEstablished)
    {
        // try to connect to client
        if (m_serverSocket.IsConnected(static_cast<tTimeStamp>(2e2)))
        {
            RETURN_IF_FAILED(m_serverSocket.Accept(m_streamSocket));
            LOG_INFO("TCP Connection was established");
            m_clientConnectionEstablished = tTrue;
        }
    }
    else
    {
        if (m_streamSocket.DataAvailable())
        {
            cString strBuffer;
            const tSize bufferSize = 65536;
            tInt bytesRead = 0;
            //make some space for data
            strBuffer.SetBuffer(bufferSize);
            // if read ok
            tResult res = m_streamSocket.Read((void*) strBuffer.GetPtr(), bufferSize, &bytesRead);
            if (IS_OK(res))
            {
                LOG_INFO(cString::Format("Received from client: %s", strBuffer.GetPtr()));
                data.clear();
                data.resize(bytesRead);
                memcpy(data.data(), strBuffer.GetPtr(), bytesRead);
            }
            else
            {
                LOG_INFO("TCP Connection was disconnected");
                m_clientConnectionEstablished = tFalse;
            }
        }
        else
        {
            RETURN_ERROR(ERR_NOT_READY);
        }
    }
    RETURN_NOERROR;
}



//// below from 2017

tResult ScmJuryCommunication::ProcessActionData(TActionStruct::Data inputAction)
{

    TFeedbackStruct::Data feedback_tmp;
    feedback_tmp.ui8FilterId = F_JURY_COMMUNICATION;

    /* set feedback.status according to input inputAction */
    if(inputAction.ui32Command == AC_JURY_ALIVE_AND_READY){
        /* send back status to prove 'up and running' */
        feedback_tmp.ui32FeedbackStatus = FB_JURY_ALIVE_AND_READY;
        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));
    }
    else if(inputAction.ui32Command == AC_JURY_MAN_GET_FIRST){
        /* check if maneuver list was already loaded */
        if(getFlagManeuverListLoaded()){
            /* get new/current maneuver from maneuverList */
            tUInt32 currMan_tmp = getCurrentManeuverFromJuryList();
            if(currMan_tmp == 0){
                RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("JCom: No valid Maneuver could be received from JuryManeuverList!"));
            }
            feedback_tmp.ui32FeedbackStatus = currMan_tmp;
            RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));
        }

        else{
            /* maneuver list not loaded yet, so do nothing until it is loaded;
                     *  after successful loading, feedback will be transmitted to SCM */
            /* set flag for requested transmit of first maneuver to true */
            setFlagFirstTransmitRequested(tTrue);

            if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: first maneuver requested, but maneuver list not loaded yet!"));
        }
    }
    else if(inputAction.ui32Command == AC_JURY_MAN_GET_PREVIOUS){
        /* send back status to with previously executed maneuver */
        feedback_tmp.ui32FeedbackStatus = getPreviousManeuver();
        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));

    }
    else if(inputAction.ui32Command == AC_JURY_MAN_FINISHED_GET_NEXT){
        /* before incrementing maneuver, save successfully executed maneuver as 'previous maneuver' */
        tUInt32 prevMan_tmp = getCurrentManeuverFromJuryList();
        if(prevMan_tmp == 0){
            RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("JCom: SavePreviousMan - No valid Maneuver could be received from JuryManeuverList!"));
        }
        setPreviousManeuver(prevMan_tmp);

        /* increment maneuver ID; returns retval < 0 in case of error, retval > 0 for completed list */
        tInt32 tmp_res = incrementManeuverID();
        if (tmp_res < 0){ // error occured
            RETURN_AND_LOG_ERROR_STR(ERR_END_OF_FILE,cString::Format("JCom: tried to increment maneuver ID, but failed."));
        }
        else if(tmp_res > 0){ // end of maneuverlist was reached
            if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: End of maneuverlist was reached! Switch to state COMPLETE."));
            setFlagCompletedJuryList(tTrue);
            changeState(stateCar_COMPLETE);
            feedback_tmp.ui32FeedbackStatus = FB_JURY_NO_MAN_REMAINING;
            RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));

        }
        else{ // ID could be incremented, tmp_res = 0 was returned
            /* get new/current maneuver from maneuverList */
            tUInt32 currMan_tmp = getCurrentManeuverFromJuryList();
            if(currMan_tmp == 0){
                RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("JCom: No valid Maneuver could be received from JuryManeuverList!"));
            }
            feedback_tmp.ui32FeedbackStatus = currMan_tmp;
            RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));

        }
    }
    else if(inputAction.ui32Command == AC_JURY_MAN_GET_FINISHING){
        /* get finishing maneuver from maneuverList, that means last one on the list (after state completed reached) */
        tUInt32 finishingMan_tmp = getFinishingManeuverFromJuryList();
        /* check if finishing maneuver (coded as tUInt32) can be received, that means != 0 */
        if(finishingMan_tmp == 0){
            RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("JCom: No valid finishing Maneuver could be received from JuryManeuverList, list might not be finished!"));
        }
        feedback_tmp.ui32FeedbackStatus = finishingMan_tmp;
        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));


    }
    else if(inputAction.ui32Command == AC_JURY_SET_CARSTATE_ERROR_AND_RESET){
        /* call methods to change carState accordingly */
        changeState(stateCar_ERROR); // changes state, transmits to jury
        resetSection(); // after error, section has to be started from beginning
        TFeedbackStruct::Data feedback_tmp;
        feedback_tmp.ui8FilterId = F_JURY_COMMUNICATION;
        feedback_tmp.ui32FeedbackStatus = FB_JURY_CARSTATE_ERROR_SECTION_RESET;
        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));


    }
    else if(inputAction.ui32Command == AC_JURY_WAIT_FOR_COMMAND_START){
        /* wait for the START command from jury, send feedback when received */
        if(getFlagStartSignalReceived()){
            TFeedbackStruct::Data feedback_tmp;
            feedback_tmp.ui8FilterId = F_JURY_COMMUNICATION;
            feedback_tmp.ui32FeedbackStatus = FB_JURY_COMMAND_RECEIVED_START;
            RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedbackStruct, (void *) &feedback_tmp, m_pClock->GetStreamTime()));

            /* Reset signal flag for next time */
            setFlagStartSignalReceived(tFalse);
        }
        else{
            /* Do nothing, as Start signal has not been received yet;
                     *  -> waiting for Jury to send action_START */
            if(m_bDebugModeEnabled)  LOG_WARNING(cString::Format("JCom: waiting for jury to send 'action_START'!"));
            setFlagStartSignalRequested(tTrue);
        }

    }
    RETURN_NOERROR;
}


tResult ScmJuryCommunication::loadManeuverList()
{
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionManeuverList);
    m_sectorList.clear();
    cDOM oDOM;
    oDOM.FromString(m_strManeuverFileString);
    cDOMElementRefList oSectorElems;
    cDOMElementRefList oManeuverElems;
    if(m_bPrintStrucModeEnabled) LOG_WARNING(cString::Format("JCom: Received ManeuverList: "));
    //read first Sector Element
    if(IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
    {
        //iterate through sectors
        for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem)
        {
            //if sector found
            tSector sector;
            sector.id = (*itSectorElem)->GetAttributeUInt32("id");
            if(m_bPrintStrucModeEnabled) LOG_WARNING(cString::Format("JCom: --> Sector %d",sector.id));

            if(IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems)))
            {
                //iterate through maneuvers
                for(cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem)
                {
                    tAADC_Maneuver man;
                    man.id = (*itManeuverElem)->GetAttributeUInt32("id");
                    man.action = (*itManeuverElem)->GetAttribute("action");
                    sector.maneuverList.push_back(man);
                    if(m_bPrintStrucModeEnabled) LOG_WARNING(cString::Format("JCom: ----> Maneuver id %d, action: ", man.id) + man.action);
                }
            }

            m_sectorList.push_back(sector);
        }
    }
    if (oSectorElems.size() > 0)
    {
        LOG_WARNING(cString::Format("JCom: Transfer and loading procedure of maneuver file successful."));
        m_i16SectionListIndex = 0;
        m_i16ManeuverListIndex = 0;
        setFlagManeuverListLoaded(tTrue);
    }
    else
    {
        LOG_ERROR(cString::Format("JCom: No valid Maneuver Data found!"));
        m_i16SectionListIndex = -1;
        m_i16ManeuverListIndex =-1;
        RETURN_ERROR(ERR_INVALID_FILE);
    }


    RETURN_NOERROR;
}

/* Function to change or access information regarding the previously executed maneuver */
tUInt32 ScmJuryCommunication::getPreviousManeuver(){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionPreviousManeuverAccess);
    return tmpPrevManeuverInt_fb;
}
tResult ScmJuryCommunication::setPreviousManeuver(tUInt32 curManeuver){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionPreviousManeuverAccess);
    tUInt32 tmp_prevManeuver = 0;
    switch (curManeuver){
    case FB_JURY_MAN_LEFT:
        tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_LEFT;
        break;
    case FB_JURY_MAN_RIGHT:
        tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_RIGHT;
        break;
    case FB_JURY_MAN_STRAIGHT:
        tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_STRAIGHT;
        break;
    case FB_JURY_MAN_PARALLEL_PARKING:
        tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_PARALLEL_PARKING;
        break;
    case FB_JURY_MAN_CROSS_PARKING:
        tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_CROSS_PARKING;
        break;
    case FB_JURY_MAN_PULL_OUT_LEFT:
        tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_PULL_OUT_LEFT;
        break;
    case FB_JURY_MAN_PULL_OUT_RIGHT:
        tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_PULL_OUT_RIGHT;
        break;
    default:// FB_JURY_NO_MAN_REMAINING or anything else -> reset
        tmp_prevManeuver = FB_JURY_PREVIOUS_NO_MAN_EXISTING;
        break;
    }
    tmpPrevManeuverInt_fb = tmp_prevManeuver;
    RETURN_NOERROR;
}
tResult ScmJuryCommunication::resetPreviousManeuver(){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionPreviousManeuverAccess);
    tmpPrevManeuverInt_fb = FB_JURY_PREVIOUS_NO_MAN_EXISTING;
    RETURN_NOERROR;
}


tBool ScmJuryCommunication::getFlagManeuverListLoaded(){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionManListLoaded);
    return m_maneuverListLoaded;
}
tResult ScmJuryCommunication::setFlagManeuverListLoaded(tBool value){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionManListLoaded);
    m_maneuverListLoaded = value;
    RETURN_NOERROR;
}

tBool ScmJuryCommunication::getFlagFirstTransmitRequested(){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionFirstTransmitReq);
    return m_maneuverFirstTransmitrequested;
}
tResult ScmJuryCommunication::setFlagFirstTransmitRequested(tBool value){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionFirstTransmitReq);
    m_maneuverFirstTransmitrequested = value;
    RETURN_NOERROR;
}

tBool ScmJuryCommunication::getFlagStartSignalRequested(){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionStartSignalReq);
    return m_startSignalrequested;
}
tResult ScmJuryCommunication::setFlagStartSignalRequested(tBool value){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionStartSignalReq);
    m_startSignalrequested = value;
    RETURN_NOERROR;
}
tBool ScmJuryCommunication::getFlagStartSignalReceived(){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionStartSignalReceived);
    return m_startSignalreceived;
}
tResult ScmJuryCommunication::setFlagStartSignalReceived(tBool value){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionStartSignalReceived);
    m_startSignalreceived = value;
    RETURN_NOERROR;
}

tBool ScmJuryCommunication::getFlagCompletedJuryList(){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionCompletedJuryList);
    return m_bCompletedJuryList;
}
tResult ScmJuryCommunication::setFlagCompletedJuryList(tBool value){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionCompletedJuryList);
    m_bCompletedJuryList = value;
    RETURN_NOERROR;
}


/* Method for decoding given maneuver from Jury list and returning corresponding predefined tUint32 value */
tUInt32 ScmJuryCommunication::decodeManeuverFromJuryList(cString tmp_juryManeuver){
    tUInt32 juryManeuverInt = 0;
    /* tInt CompareNoCase(const cString& strString) const; */
    if(tmp_juryManeuver.CompareNoCase("left") == 0)
    {
        /* action="left" */
        juryManeuverInt = FB_JURY_MAN_LEFT;
        if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'left'"));
    }
    else if(tmp_juryManeuver.CompareNoCase("right") == 0)
    {
        /* action="right" */
        juryManeuverInt = FB_JURY_MAN_RIGHT;
        if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'right'"));
    }
    else if(tmp_juryManeuver.CompareNoCase("straight") == 0)
    {
        /* action="straight" */
        juryManeuverInt = FB_JURY_MAN_STRAIGHT;
        if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'straight'"));
    }
    else if(tmp_juryManeuver.CompareNoCase("parallel_parking") == 0)
    {
        /* action="parallel_parking" */
        juryManeuverInt = FB_JURY_MAN_PARALLEL_PARKING;
        if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'parallel_parking'"));
    }
    //	else if(tmp_juryManeuver.CompareNoCase("cross_parking") == 0)
    //	{
    //		/* action="cross_parking" */
    //		juryManeuverInt = FB_JURY_MAN_CROSS_PARKING;
    //		if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'cross_parking'"));
    //	}
    // --- handle cross parking with parking spot IDs
    else if(tmp_juryManeuver.StartsWith("cross_parking"))
    {
        // get the ID
        cString parkingSpotIDString = tmp_juryManeuver.Right(2);
        tInt32 parkingSpotID = (parkingSpotIDString.AsInt32() - 1) % 4;

        switch (parkingSpotID)
        {
        case 0:
            juryManeuverInt = FB_JURY_CROSS_PARKING_SPOT_FOUR;
            break;

        case 1:
            juryManeuverInt = FB_JURY_CROSS_PARKING_SPOT_THREE;
            break;

        case 2:
            juryManeuverInt = FB_JURY_CROSS_PARKING_SPOT_TWO;
            break;

        case 3:
            juryManeuverInt = FB_JURY_CROSS_PARKING_SPOT_ONE;
            break;
        }

        /* action="cross_parking" */

        if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'cross_parking' in parking spot ID %d", parkingSpotID));
    }
    // ---
    else if(tmp_juryManeuver.CompareNoCase("pull_out_left") == 0)
    {
        /* action="pull_out_left" */
        juryManeuverInt = FB_JURY_MAN_PULL_OUT_LEFT;
        if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'pull_out_left'"));
    }
    else if(tmp_juryManeuver.CompareNoCase("pull_out_right") == 0)
    {
        /* action="pull_out_right */
        juryManeuverInt = FB_JURY_MAN_PULL_OUT_RIGHT;
        if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'pull_out_right'"));
    }
    else{
        LOG_ERROR(adtf_util::cString::Format("JCom: DecodeJuryManeuver: Maneuver could not be decoded, ERROR occurred!"));
    }

    return juryManeuverInt;
}


/* Method for returning the current maneuver from Jury list */
tUInt32 ScmJuryCommunication::getCurrentManeuverFromJuryList(){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionManeuverList);
    if (getFlagManeuverListLoaded() == tFalse){
        LOG_ERROR( cString::Format("JCom: getCurrentManeuver -  ManeuverList not loaded, could not get next maneuver!"));
        return 0;
    }
    cString tmp_currJuryManeuver = " ";
    tUInt32 currentJuryManeuverInt = 0;

    tmp_currJuryManeuver = m_sectorList[m_i16SectionListIndex].maneuverList[m_i16ManeuverListIndex].action;
    currentJuryManeuverInt = decodeManeuverFromJuryList(tmp_currJuryManeuver);

    return currentJuryManeuverInt;
}


/* Method for returning the finishing maneuver from Jury list, that means the maneuver that was executed as the last maneuver on the list */
tUInt32 ScmJuryCommunication::getFinishingManeuverFromJuryList(){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionManeuverList);
    if (getFlagManeuverListLoaded() == tFalse){
        LOG_ERROR( cString::Format("JCom: getFinishingManeuver -  ManeuverList not loaded, could not get finishing maneuver!"));
        return 0;
    }
    cString tmp_finishingJuryManeuver = " ";
    tUInt32 finishingJuryManeuverInt = 0;
    /* check if the maneuver-list was actually completed */
    if (getFlagCompletedJuryList()){
        /* after completing the maneuver list, the id is was NOT incremented anymore (avoiding index out of bounds) */
        tmp_finishingJuryManeuver = m_sectorList[m_i16SectionListIndex].maneuverList[m_i16ManeuverListIndex].action;
        finishingJuryManeuverInt = decodeManeuverFromJuryList(tmp_finishingJuryManeuver);
    }

    return finishingJuryManeuverInt;
}

/* Method for transmitting CarState and ManeuverEntry state to jury module */
tResult ScmJuryCommunication::SendState(stateCar state, tInt16 i16ManeuverEntry)
{
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionTransmitCarState);
    tDriverStruct driverStruct;
    driverStruct.i16StateID = state;
    driverStruct.i16ManeuverEntry = i16ManeuverEntry;
    if (m_clientConnectionEstablished)
    {
        RETURN_IF_FAILED(m_streamSocket.Write(&driverStruct, sizeof(tDriverStruct)));
    }


    //debug output to console
    if(m_bDebugSendStateModeEnabled)
    {
        switch (state)
        {
        case stateCar_ERROR:
            LOG_WARNING(cString::Format("JCom: Send state: ERROR, Maneuver ID %d",i16ManeuverEntry));
            break;
        case stateCar_READY:
            LOG_WARNING(cString::Format("JCom: Send state: READY, Maneuver ID %d",i16ManeuverEntry));
            break;
        case stateCar_RUNNING:
            LOG_WARNING(cString::Format("JCom: Send state: RUNNING, Maneuver ID %d",i16ManeuverEntry));
            break;
        case stateCar_COMPLETE:
            LOG_WARNING(cString::Format("JCom: Send state: COMPLETE, Maneuver ID %d",i16ManeuverEntry));
            break;
        case stateCar_STARTUP:
            LOG_WARNING(cString::Format("JCom: Send state: STARTUP."));
            break;
        }
    }

    RETURN_NOERROR;
}


/* Method for changing the State of the car; After change, new status is immediately sent to jury;
 *  just used for 'startup', 'error' ,'ready' and 'complete' */
tResult ScmJuryCommunication::changeState(stateCar newState)
{
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionCarStatus);
    // if state is the same do nothing
    if (m_CarState == newState) RETURN_NOERROR;

    // to secure the state is sent at least one time
    SendState(newState, getCurManeuverID());

    // handle the timer depending on the state
    switch (newState)
    {
    case stateCar_ERROR:
        //        destroyTimer();
        LOG_WARNING(cString::Format("JCom: State ERROR reached."));
        break;
    case stateCar_STARTUP:
        //        destroyTimer();
        LOG_WARNING(cString::Format("JCom: State STARTUP reached."));
        break;
    case stateCar_READY:
        //        createTimer();
        LOG_WARNING(cString::Format("JCom: State READY reached (ID %d).",getCurManeuverID()));
        break;
    case stateCar_RUNNING:
        if (m_CarState!=stateCar_READY)
            LOG_WARNING(cString::Format("JCom: Invalid state change to state 'RUNNING'. State 'READY' was not reached before. Nevertheless, change will be executed."));
        LOG_WARNING(cString::Format("JCom: State RUNNING reached."));
        break;
    case stateCar_COMPLETE:
        //        destroyTimer();
        LOG_WARNING(cString::Format("JCom: State COMPLETE reached."));
        break;
    }

    m_CarState = newState;
    RETURN_NOERROR;
}



stateCar ScmJuryCommunication::getCarState(){
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionCarStatus);
    return m_CarState;
}

/* Method returns the current maneuverID, that means the ID of the currently active maneuver that is being processed */
tInt16 ScmJuryCommunication::getCurManeuverID()
{
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionManeuverList);
    return m_i16CurrentManeuverID;
}


/* Methods increments the current maneuver ID after maneuver is successfully finished */
tInt32 ScmJuryCommunication::incrementManeuverID()
{
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionManeuverList);
    tInt32 retval = -1;
    // check if list was successfully loaded during initialization
    if (m_i16ManeuverListIndex!=-1 && m_i16SectionListIndex!=-1)
    {
        retval = 0;
        // check if end of section is reached
        // if NOT reached:
        if (m_sectorList[m_i16SectionListIndex].maneuverList.size()>tUInt(m_i16ManeuverListIndex+1))
        {
            // increment only maneuver index, as there are still maneuvers to process in current section
            m_i16ManeuverListIndex++;
            // increment the current ManeuverID, representing the global position in list (above sector borders)
            m_i16CurrentManeuverID++;
        }
        else //end of section is reached
        {
            // end of section was reached and another section is in list
            if (m_sectorList.size() >tUInt(m_i16SectionListIndex+1))
            {
                //reset maneuver index to zero and increment section list index, since new sector is entered now
                m_i16SectionListIndex++;
                m_i16ManeuverListIndex = 0; // start at beginning of sector, with maneuverIndex zero
                m_i16CurrentManeuverID++; // just incremented, since it represents global position (corresponding to ID)
                if (m_sectorList[m_i16SectionListIndex].maneuverList[m_i16ManeuverListIndex].id !=m_i16CurrentManeuverID)
                {
                    LOG_ERROR("JCom: inconsistency in maneuverfile detected, IDs don't match. Please check the file!");
                }
            }
            else
            {
                /* End of maneuver list is reached, all sectors completed */
                if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: End of maneuverlist reached, cannot increment any more. List completed!"));
                retval = 1;
                /* Change carState to Completed, go to startup or Completed-maneuver in SCM! */
            }
        }
    }
    else
    {
        LOG_ERROR("JCom: could not set new maneuver ID because no maneuver list was loaded!");
    }

    if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: Increment Maneuver ID: Sectionindex is %d, Maneuverindex is %d, ID is %d",m_i16SectionListIndex,m_i16ManeuverListIndex,m_i16CurrentManeuverID));

    return retval;
}

/* Method for reseting section;
 *  After execution, the maneuver that has to be processed next is reset to the first maneuver in the current sector */
tResult ScmJuryCommunication::resetSection()
{
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionManeuverList);
    //maneuver list index to zero, and current maneuver id to first element in list
    m_i16ManeuverListIndex=0;
    m_i16CurrentManeuverID = m_sectorList[m_i16SectionListIndex].maneuverList[m_i16ManeuverListIndex].id;

    if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: Reset section: Sectionindex is %d, Maneuverindex is %d, ID is %d",m_i16SectionListIndex,m_i16ManeuverListIndex,m_i16CurrentManeuverID));

    RETURN_NOERROR;
}

/* Method for directly setting the maneuver ID;
 *  searches saved maneuverlist for provided 'maneuverId' and sets the sectionListindex and ManeuverListIndex accordingly */
tResult ScmJuryCommunication::setManeuverID(tInt maneuverId)
{
    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionManeuverList);
    //look for the right section id and write it to section combobox
    for(unsigned int i = 0; i < m_sectorList.size(); i++)
    {
        for(unsigned int j = 0; j < m_sectorList[i].maneuverList.size(); j++)
        {
            if(maneuverId == m_sectorList[i].maneuverList[j].id)
            {
                m_i16SectionListIndex = i;
                m_i16ManeuverListIndex = j;
                m_i16CurrentManeuverID = maneuverId;
                if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: Sectionindex is %d, Maneuverindex is %d, ID is %d",m_i16SectionListIndex,m_i16ManeuverListIndex,m_i16CurrentManeuverID));
                break;
            }
        }
    }
    RETURN_NOERROR;
}

/* Method for creating a timer with name m_hTimer with 0.5 seconds */
tResult ScmJuryCommunication::createTimer()
{
    //    // creates timer with 0.5 sec
    //    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionTimerSetup);
    //    // additional check necessary because input jury structs can be mixed up because every signal is sent three times
    //    if (m_hTimer == NULL)
    //    {
    //        m_hTimer = m_pKernel->TimerCreate(tTimeStamp(0.5*1000000), 0, static_cast<IRunnable*>(this),
    //                                        NULL, NULL, 0, 0, adtf_util::cString::Format("%s.timer", OIGetInstanceName()));
    //    }
    //    else
    //    {
    //        LOG_ERROR("JCom: Timer is already running. Unable to create a new one.");
    //    }
    RETURN_NOERROR;
}

/* Method for destroying the timer m_hTimer*/
tResult ScmJuryCommunication::destroyTimer()
{
    //    boost::lock_guard<boost::mutex> lock(m_oCriticalSectionTimerSetup);
    //    //destroy timer
    //    if (m_hTimer != NULL)
    //    {
    //        tResult nResult = m_hTimer.Destroy();
    //        if (IS_FAILED(nResult))
    //        {
    //            LOG_ERROR("JCom: Unable to destroy the timer.");
    ////            THROW_ERROR(nResult);
    //        }
    //        m_hTimer = NULL;
    //    }
    //    //check if handle for some unknown reason still exists
    //    else
    //    {
    //        //LOG_WARNING("JCom: Timer handle not set, but I should destroy the timer. Try to find a timer with my name.");
    //        tHandle hFoundHandle = m_pKernel->FindHandle(adtf_util::cString::Format("%s.timer", OIGetInstanceName()));
    //        if (hFoundHandle)
    //        {
    //            tResult nResult = hFoundHandle.Destroy();
    //            if (IS_FAILED(nResult))
    //            {
    //                LOG_ERROR("JCom: Unable to destroy the specified timer.");
    ////                THROW_ERROR(nResult);
    //            }
    //        }
    //    }

    RETURN_NOERROR;
}
