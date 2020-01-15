cmake_minimum_required(VERSION 3.10.0)

project(ScmJuryCommunication)

if (NOT TARGET adtf::ui)
    find_package(ADTF COMPONENTS filtersdk ui)
endif()
find_package(adtfui REQUIRED)
set_property(GLOBAL PROPERTY AUTOGEN_TARGETS_FOLDER AutoMoc)
adtf_use_qt(Widgets Gui)




include_directories(${Boost_INCLUDE_DIRS})
#include_directories(${OpenCV_INCLUDE_DIRS})

set (SOURCEFILES    	stdafx.h 
			ScmJuryCommunication.h
			ScmJuryCommunication.cpp
Maneuverlist.h
          	StateControl.h
DriverModuleWidget.cpp

DriverModuleWidget.h
			)
# Instruct CMake to run moc automatically when needed.
set (CMAKE_AUTOMOC ON)
set (CMAKE_AUTOUIC ON)
set (CMAKE_AUTORCC ON)
adtf_add_filter(${PROJECT_NAME}
				${SOURCEFILES}
)


set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOGEN_TARGET_DEPENDS ${HEADERS})

target_link_libraries(${PROJECT_NAME} PUBLIC adtf::ui Qt5::Core Qt5::Gui Qt5::Widgets ${Boost_LIBRARIES} )
target_include_directories(${PROJECT_NAME} PUBLIC .)


# Specify where it should be installed to
adtf_install_target(${PROJECT_NAME} bin)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
DEPENDENT_PLUGINS
        "adtf_xsystem"
 )
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${Boost_LIBRARIES})
#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)
