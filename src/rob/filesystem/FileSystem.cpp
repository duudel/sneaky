
#include "FileSystem.h"

namespace rob
{
namespace fs
{

    File OpenToRead(const char * const filename)
    { return std::fopen(filename, "rb"); }

    File OpenToWrite(const char * const filename)
    { return std::fopen(filename, "wb"); }

    File OpenToAppend(const char * const filename)
    { return std::fopen(filename, "ab"); }

    void Close(File file)
    { std::fclose(file); }

    void Read(File file, char *data, size_t dataSize)
    { std::fread(data, 1, dataSize, file); }

    void Write(File file, const char *data, size_t dataSize)
    { std::fwrite(data, 1, dataSize, file); }

} // fs
} // rob
