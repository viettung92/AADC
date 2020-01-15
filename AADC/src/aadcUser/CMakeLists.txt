cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

cmake_policy(SET CMP0002 NEW)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0011 NEW)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1z -Wall -Wextra -Werror")


# The project and cmake vars must only be defined in case that only the base
# filter should be build.
if(NOT AADC_BUILD_ALL_IN_ONE)
    # define a project name
    project(aadc_user)

    # CMAKE_BUILD_TYPE is empty on Unix systems, but needed by ADTF. Otherwise the ADTF libs
    # can't be found and linked.
    if(UNIX AND NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
                "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
                FORCE)
    endif(UNIX AND NOT CMAKE_BUILD_TYPE)
    

endif(NOT AADC_BUILD_ALL_IN_ONE)

#-------General------------

set(AADC_FOUND FALSE)
set(AADC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../)
find_package(AADC REQUIRED)
if (AADC_FOUND)
   message("-- AADC found")
else(AADC_FOUND)
   message(FATAL_ERROR "-- AADC not found")
endif (AADC_FOUND)

find_package(ADTF REQUIRED)
message(STATUS "Found ADTF: ${ADTF_VERSION}")

#--------------add private macros-----------------
include(${AADC_DIR}/AADC_PRIVATE.cmake)
include(${AADC_DIR}/AADCConfigVersion.cmake)

#--------------sources----------------------------
include_directories(${AADC_DIR}/include)
include_directories(${CMAKE_INSTALL_INCLUDE})




add_subdirectory(MarkerDetectorNew) #Working
#add_subdirectory(TemplateFilter)
#add_subdirectory(playground)

#add_subdirectory(ImageProcessing/LineFilter) #Working
#add_subdirectory(ImageProcessing/LineSpecifier) # Working
add_subdirectory(LaneDetection) #Working
add_subdirectory(ActionStop) #process
add_subdirectory(SelectSpeed) #Working
add_subdirectory(EmergencyBreak) #NOT WORKING
add_subdirectory(SpeedTransmitter) #wroking
add_subdirectory(scmJuryCommunication) #Working
#add_subdirectory(FTO_NegateSpeedController)
add_subdirectory(UltrasonicACC) #Working

#add_subdirectory(MarkerDetector)

#add_subdirectory(MarkerDetectorNew)
#add_subdirectory(IntersectionDetectorFilter)
add_subdirectory(OutputLimiter) #Working
add_subdirectory(UltrasonicMeanFilter) #Working
#add_subdirectory(ActionStructSender)#process
#add_subdirectory(ActionStructReceiver)#process
#add_subdirectory(UltrasonicCheck)

add_subdirectory(LightControl) #Working
add_subdirectory(FTO_CarPose) #Working
add_subdirectory(FTO_MoveToPoint) #Working
#add_subdirectory(TrafficLightSimulator)
add_subdirectory(FTO_PIDController) #Working 

add_subdirectory(StateControlManagement) #Working
add_subdirectory(DriverModule) #Working
#add_subdirectory(SpeedTransmitter)
#add_subdirectory(synchronizer_filter)
#add_subdirectory(SCMTestLimitSpeed)
#add_subdirectory(TemplateFilter)





#add_subdirectory(OC_StateControlManagement)
#add_subdirectory(OC_MarkerDetectorNew)









