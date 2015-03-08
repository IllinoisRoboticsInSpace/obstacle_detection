#include "TrigLookup.hpp"


/**These should be lookup tables for trig functions**/
/**use the static keyword for persistent data in a function**/


float sinL(float radiansCCW)//takes an angle interpeted as radians in the CCW direction
{
    return sin(radiansCCW);
}
float cosL(float radiansCCW)//takes an angle interpeted as radians in the CCW direction
{
    return cos(radiansCCW);
}
float tanL(float radiansCCW)//takes an angle interpeted as radians in the CCW direction
{
    return tan(radiansCCW);
}
