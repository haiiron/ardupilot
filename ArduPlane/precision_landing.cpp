//
// functions to support precision landing
//

#include "Plane.h"

void Plane::init_precland()
{ plane.precland.init(400); }

void Plane::update_precland()
{
    // 두 번째 매개변수로 false를 전달하면 alt는 사용되지 않음.
    return precland.update(rangefinder_state.alt_cm_glitch_protected, false); //rangefinder_alt_ok() 아마 0 나올듯..?
}