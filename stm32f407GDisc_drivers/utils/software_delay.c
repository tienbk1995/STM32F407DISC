#include "software_delay.h"

void SoftwareDelay(uint32_t tick)
{
    for (uint32_t i = 0; i <= tick; i++);
}