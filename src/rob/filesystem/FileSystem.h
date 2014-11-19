
#ifndef H_ROB_FILE_SYSTEM_H
#define H_ROB_FILE_SYSTEM_H

#include <cstdio>

namespace rob
{
namespace fs
{

    typedef std::FILE* File;

    File OpenToRead(const char * const filename);
    File OpenToWrite(const char * const filename);
    File OpenToAppend(const char * const filename);
    void Close(File file);

    void Read(File file, char *data, size_t dataSize);

    template <class T>
    void Read(File file, T &value)
    { Read(file, reinterpret_cast<char*>(&value), sizeof(T)); }

    void Write(File file, const char *data, size_t dataSize);

    template <class T>
    void Write(File file, const T &value)
    { Write(file, reinterpret_cast<const char*>(&value), sizeof(T)); }

} // fs
} // rob

#endif // H_ROB_FILE_SYSTEM_H

