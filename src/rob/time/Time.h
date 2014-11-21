
#ifndef H_ROB_TIME_H
#define H_ROB_TIME_H

#include "../Types.h"

namespace rob
{

    uint32_t GetTicks();
    uint32_t GetTime();

    void Delay(uint32_t milliseconds);

} // rob

#endif // H_ROB_TIME_H

