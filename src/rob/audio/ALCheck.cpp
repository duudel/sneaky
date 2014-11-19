
#include "ALCheck.h"
#include "../Log.h"

#include <AL/al.h>

namespace rob
{

    static const char* ALErrorString(ALenum err)
    {
        switch (err)
        {
        case AL_NO_ERROR: return "No error";
        case AL_INVALID_NAME: return "Invalid name";
        case AL_INVALID_ENUM: return "Invalid enum";
        case AL_INVALID_VALUE: return "Invalid value";
        case AL_INVALID_OPERATION: return "Invalid operation";
        case AL_OUT_OF_MEMORY: return "Out of memory";
        default:
            return "Unknown error";
        }
    }

    void ALCheck(const char * const file, const int line)
    {
        ALenum err = ::alGetError();
        if (err != AL_NO_ERROR)
        {
            log::Error("AL error: ", ALErrorString(err), " in ", file, ":", line);
        }
    }

} // rob
