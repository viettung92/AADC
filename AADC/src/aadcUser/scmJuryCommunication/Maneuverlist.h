#ifndef MANEUVERLIST_H
#define MANEUVERLIST_H

#include "stdafx.h"
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

struct tAADC_Maneuver
{
    int id;
    cString action;
};
struct tManeuver
{
    int id;
    cString action;
};
struct tSector
{
    int id;
    std::vector<tAADC_Maneuver> maneuverList;
};

#endif
