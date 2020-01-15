#pragma once

#include <string.h>
#include <iostream>
#include <ctime>
#include <string>
#include "tinyxml2.h"

using std::string;

template <typename T>
    bool IsInBounds(const T& value, const T& low, const T& high) {
    return !(value < low) && (value < high);
}
template <typename T>
    bool clamp(T &value, const T& low, const T& high) {
        if(value < low){
            value = low;
            return true;
        }
        if(value > high){
            value = high;
            return true;
        }
        return false;
}

string add_date_to_filename(string filename, string ending){

    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%d-%m-%Y %H:%M:%S",timeinfo);
    std::string date(buffer);

    return filename + "_" + date + "." + ending;
}

