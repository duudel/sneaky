
#include "Assert.h"
#include "Log.h"

namespace rob
{

    void Assert(const char * const x, const char * const file, const int line)
    {
        log::Print("Assert: ", x, " in ", file, ":", line, "\n");
        ((int*)nullptr)[0] = 42;
    }

    void Warn(const char * const x, const char * const file, const int line)
    {
        log::Warning(x, " in ", file, ":", line, "\n");
    }

} // rob
