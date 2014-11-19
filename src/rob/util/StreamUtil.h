
#ifndef H_ROB_STREAM_UTIL_H
#define H_ROB_STREAM_UTIL_H

#include <istream>

namespace rob
{

    template <class T>
    inline T ReadValue(std::istream &file)
    {
        T value;
        file.read(reinterpret_cast<char*>(&value), sizeof(T));
        return value;
    }

    template <size_t N>
    inline size_t ReadString(std::istream &file, char (&buffer)[N])
    {
        file.getline(buffer, N, '\0');
        return file.gcount();
    }

} // rob

#endif // H_ROB_STREAM_UTIL_H

