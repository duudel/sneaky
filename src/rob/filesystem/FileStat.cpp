
#include "FileStat.h"

#include <sys/stat.h>

namespace rob
{

    bool FileExists(const char* const path)
    {
        struct ::_stat s;
        return (::_stat(path, &s) == 0);
    }

    bool IsDirectory(const char * const path)
    {
        struct ::_stat s;
        if (::_stat(path, &s) == 0)
        {
            return S_ISDIR(s.st_mode);
        }
        return false;
    }

    time_t GetModifyTime(const char * const path)
    {
        struct ::_stat s;
        if (::_stat(path, &s) == 0)
        {
            return s.st_mtime;
        }
        return time_t(0);
    }

    size_t GetFileSize(const char * const path)
    {
        struct ::_stat s;
        if (::_stat(path, &s) == 0)
        {
            return s.st_size;
        }
        return 0;
    }

} // rob
