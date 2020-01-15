#pragma once


#include <iomanip>      // std::setw

#include <adtf_filtersdk.h>


using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace adtf::filter::ant;


#include "aadc_structs.h"
#include "aadc_user_structs.h"


class AADC_Type {
   //protected:
   public:
    adtf::mediadescription::cSampleCodecFactory factory;
    object_ptr<IStreamType> object;
    cString type_name;
   //public:
    //Register function ist eine member function von adtf::filter::ant::cTriggerFunction, kann
    //also nur vom Filter selbst aufgerufen werden (was auch Sinn macht)
    //es gibt zwei möglichkeiten: entweder man übergibt den filter selbst als pointer (per this)
    //oder der Filter holt sich von der AADC_Type klasse das object und registriert sich selbst
    //ich hab mich für ersteres entschieden weil einfacher für den entanwender (filter programmierer)
    //  -  wo bleibt da die Demokratie!! :'( (filter anwender)
	//  -  Demokratie ist überbewertet und überhaupt ist meine Meinung die beste als mögliche :D
	
    tResult registerPin(cTriggerFunction *trigger, cPinWriter &output, cString PinName)
    {
        trigger->Register(output, PinName, object);
        RETURN_NOERROR;
    }

    tResult registerPin(cTriggerFunction *trigger, cPinReader &input, cString PinName)
    {
        trigger->Register(input, PinName, object);
        RETURN_NOERROR;
    }


	object_ptr<IStreamType> &getObject(){ return object;}

    virtual tResult writePin(cPinWriter &output, void *content_ptr, tTimeStamp streamTime) = 0;
    virtual tResult readPin(cPinReader &input, void *content_ptr, tTimeStamp streamTime) = 0;

};

/*About the pragma pack(...) in the DATA structs used from the classes (from stackoverflow):
 * https://stackoverflow.com/questions/3318410/pragma-pack-effect
#pragma packinstructs the compiler to pack structure members with particular alignment. Most compilers, when you declare a struct, will insert padding between members to ensure that they are aligned to appropriate addresses in memory (usually a multiple of the type's size). This avoids the performance penalty (or outright error) on some architectures associated with accessing variables that are not aligned properly. [...] The most common use case for the #pragma is when working with hardware devices where you need to ensure that the compiler does not insert padding into the data and each member follows the previous one.
#pragma pack(push, 1) // exact fit - no padding
struct MyStruct
{
  char b;
  int a;
  int array[2];
};
#pragma pack(pop) //back to whatever the previous packing mode was

#pragma pack(push[,n]) pushes the current alignment setting on an internal stack and then optionally sets the new alignment.
#pragma pack(pop) restores the alignment setting to the one saved at the top of the internal stack (and removes that stack entry). Note that #pragma pack([n]) does not influence this internal stack; thus it is possible to have #pragma pack(push) followed by multiple #pragma pack(n) instances and finalized by a single #pragma pack(pop).
*/
