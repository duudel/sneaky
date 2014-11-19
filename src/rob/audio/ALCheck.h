
#ifndef H_ROB_AL_CHECK_H
#define H_ROB_AL_CHECK_H

#define AL_CHECK rob::ALCheck(__FILE__, __LINE__)

namespace rob
{

    void ALCheck(const char * const file, const int line);

} // rob

#endif // H_ROB_AL_CHECK_H
