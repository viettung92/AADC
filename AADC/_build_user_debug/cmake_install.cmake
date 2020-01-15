# Install script for directory: /home/aadc/AADC/src/aadcUser

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/aadc/AADC/src/aadcUser/../../_install/linux64")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/aadc/AADC/_build_user_debug/MarkerDetectorNew/cmake_install.cmake")
  include("/home/aadc/AADC/_build_user_debug/LaneDetection/cmake_install.cmake")
  include("/home/aadc/AADC/_build_user_debug/ActionStop/cmake_install.cmake")
  include("/home/aadc/AADC/_build_user_debug/SelectSpeed/cmake_install.cmake")
  include("/home/aadc/AADC/_build_user_debug/EmergencyBreak/cmake_install.cmake")
  include("/home/aadc/AADC/_build_user_debug/SpeedTransmitter/cmake_install.cmake")
  include("/home/aadc/AADC/_build_user_debug/scmJuryCommunication/cmake_install.cmake")
  include("/home/aadc/AADC/_build_user_debug/UltrasonicACC/cmake_install.cmake")
  include("/home/aadc/AADC/_build_user_debug/OutputLimiter/cmake_install.cmake")
  include("/home/aadc/AADC/_build_user_debug/UltrasonicMeanFilter/cmake_install.cmake")
  include("/home/aadc/AADC/_build_user_debug/LightControl/cmake_install.cmake")
  include("/home/aadc/AADC/_build_user_debug/FTO_CarPose/cmake_install.cmake")
  include("/home/aadc/AADC/_build_user_debug/FTO_MoveToPoint/cmake_install.cmake")
  include("/home/aadc/AADC/_build_user_debug/FTO_PIDController/cmake_install.cmake")
  include("/home/aadc/AADC/_build_user_debug/StateControlManagement/cmake_install.cmake")
  include("/home/aadc/AADC/_build_user_debug/DriverModule/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/aadc/AADC/_build_user_debug/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
