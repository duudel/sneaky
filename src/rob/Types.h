
#ifndef H_ROB_TYPES_H
#define H_ROB_TYPES_H

namespace rob
{

    typedef unsigned int size_t;

    typedef signed char         int8_t;
    typedef unsigned char       uint8_t;
    typedef signed short        int16_t;
    typedef unsigned short      uint16_t;
    typedef signed int          int32_t;
    typedef unsigned int        uint32_t;
    typedef signed long long    int64_t;
    typedef unsigned long long  uint64_t;

    typedef uint64_t Time_t;

    template <size_t S>
    struct sized_int;

    template <> struct sized_int<1>
    {
        typedef int8_t type;
        typedef uint8_t utype;
    };

    template <> struct sized_int<2>
    {
        typedef int16_t type;
        typedef uint16_t utype;
    };

    template <> struct sized_int<4>
    {
        typedef int32_t type;
        typedef uint32_t utype;
    };

    template <> struct sized_int<8>
    {
        typedef int64_t type;
        typedef uint64_t utype;
    };

    typedef sized_int<sizeof(void*)>::type  intptr_t;
    typedef sized_int<sizeof(void*)>::utype uintptr_t;

} // rob

using rob::size_t;

using rob::int8_t;
using rob::uint8_t;
using rob::int16_t;
using rob::uint16_t;
using rob::int32_t;
using rob::uint32_t;
using rob::int64_t;
using rob::uint64_t;

using rob::intptr_t;
using rob::uintptr_t;

#endif // H_ROB_TYPES_H

