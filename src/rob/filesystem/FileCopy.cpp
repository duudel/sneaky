
#include "FileCopy.h"

#include <fstream>

namespace rob
{

    void FileCopy(const char * const from, const char * const to)
    {
        std::ifstream in(from, std::ios_base::binary);
        std::ofstream out(to, std::ios_base::binary);
        while (in && out)
        {
            char buf[512];
            in.read(buf, 512);
            size_t size = in.gcount();
            out.write(buf, size);
        }
    }

} // rob
