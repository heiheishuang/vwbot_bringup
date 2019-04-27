#include <vwbot_bringup/utils.h>
#include <cmath>

using namespace std;

int16_t round_float(float number)
{
    return (number > 0.0) ? int16_t(floor(number + 0.5)) : int16_t(ceil(number - 0.5));
}
