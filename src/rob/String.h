
#ifndef H_ROB_STRING_H
#define H_ROB_STRING_H

#include "Types.h"
#include <cstring>
#include <cstdio>

namespace rob
{

    inline size_t StringLength(const char *str)
    { return std::strlen(str); }

    inline size_t CopyString(char *dest, const char *src, size_t num)
    {
        size_t i = 0;
        for (; i < num && *src; i++, src++)
            dest[i] = *src;
        return i;
    }

    template <size_t N>
    void CopyString(char (&dest)[N], const char *src)
    { CopyString(dest, src, N); }

    inline size_t CopyStringN(char *dest, const char *src, size_t num)
    {
        size_t i = 0;
        for (; i < num && *src; i++, src++)
            dest[i] = *src;
        for (; i < num; i++)
            dest[i] = 0;
        return i;
    }

    template <size_t N>
    void CopyStringN(char (&dest)[N], const char *src)
    { CopyStringN(dest, src, N); }

    template <class ...Args>
    size_t StringPrintF(char *buf, size_t bufSize, const char * const fmt, Args ...args)
    { return snprintf(buf, bufSize, fmt, args...); }

    template <size_t N, class ...Args>
    size_t StringPrintF(char (&buf)[N], const char * const fmt, Args ...args)
    { return snprintf(buf, N, fmt, args...); }




    inline uint32_t DecodeUtf8(const char *&str, const char * const end)
    {
        if (str == end) return 0;

        const uint32_t c1 = uint8_t(*str++);
        if (c1 > 0x80 && c1 < 0xC0) // In the middle of a character
        {
            while (str != end)
            {
                const uint32_t c1 = uint8_t(*str++);
                if (c1 <= 0x80 || c1 >= 0xC0)
                    break;
            }
            return '?';
        }

        if (c1 < 0x80)
            return c1;

        if (str == end) return '?';
        const uint32_t c2 = uint8_t(*str++);
        if (c1 < 0xE0)
            return (c2 & 0x3F) | ((c1 & 0x1F) << 6);

        if (str == end) return '?';
        const uint32_t c3 = uint8_t(*str++);
        if (c1 < 0xF0)
            return (c3 & 0x3F) | ((c2 & 0x3F) << 6) | ((c1 & 0x1F) << 12);

        if (str == end) return '?';
        const uint32_t c4 = uint8_t(*str++);
        if (c1 < 0xF8)
            return (c4 & 0x3F) | ((c3 & 0x3F) << 6) | ((c2 & 0x3F) << 12) | ((c1 & 0x7) << 18);

        if (str == end) return '?';
        const uint32_t c5 = uint8_t(*str++);
        if (c1 < 0xFC)
            return (c5 & 0x3F) | ((c4 & 0x3F) << 6) | ((c3 & 0x3F) << 12) | ((c2 & 0x3F) << 18) | ((c1 & 0x3) << 24);

        if (str == end) return '?';
        const uint32_t c6 = uint8_t(*str++);
        return (c6 & 0x3F) | ((c5 & 0x3F) << 6) | ((c4 & 0x3F) << 12) | ((c3 & 0x3F) << 18) | ((c2 & 0x3F) << 24) | ((c1 & 0x1) << 30);
    }

    inline size_t CopyUtf8Codepoint(const char *&str, const char * const end, char *&buf, const char * const bufEnd)
    {
        if (str == end) return 0;
        const uint32_t c1 = uint8_t(*str++);
        if (c1 > 0x80 && c1 < 0xC0) // In the middle of a character
        {
            while (str != end)
            {
                const uint32_t c1 = uint8_t(*str++);
                if (c1 <= 0x80 || c1 >= 0xC0)
                    break;
            }
            return 0;
        }

        if (c1 < 0x80)
        {
            if (buf == bufEnd) return 0;
            *buf++ = c1;
            return 1;
        }

        if (str == end) return 0;
        const uint32_t c2 = uint8_t(*str++);
        if (c1 < 0xE0)
        {
            if (bufEnd - buf < 2) return 0;
            *buf++ = c1;
            *buf++ = c2;
            return 2;
        }

        if (str == end) return 0;
        const uint32_t c3 = uint8_t(*str++);
        if (c1 < 0xF0)
        {
            if (bufEnd - buf < 3) return 0;
            *buf++ = c1;
            *buf++ = c2;
            *buf++ = c3;
            return 3;
        }

        if (str == end) return 0;
        const uint32_t c4 = uint8_t(*str++);
        if (c1 < 0xF8)
        {
            if (bufEnd - buf < 4) return 0;
            *buf++ = c1;
            *buf++ = c2;
            *buf++ = c3;
            *buf++ = c4;
            return 4;
        }

        if (str == end) return 0;
        const uint32_t c5 = uint8_t(*str++);
        if (c1 < 0xFC)
        {
            if (bufEnd - buf < 5) return 0;
            *buf++ = c1;
            *buf++ = c2;
            *buf++ = c3;
            *buf++ = c4;
            *buf++ = c5;
            return 5;
        }

        if (str == end) return 0;
        if (bufEnd - buf < 6) return 0;
        const uint32_t c6 = uint8_t(*str++);
        *buf++ = c1;
        *buf++ = c2;
        *buf++ = c3;
        *buf++ = c4;
        *buf++ = c5;
        *buf++ = c6;
        return 6;
    }

    inline uint32_t SkipUtf8Right(const char *&str, const char * const end, size_t *sz = 0)
    {
        if (sz) *sz = 0;
        if (str == end) return 0;

        const uint32_t c1 = uint8_t(*str++);
        if (c1 > 0x80 && c1 < 0xC0) // In the middle of a character
        {
            while (str != end)
            {
                const uint32_t c1 = uint8_t(*str++);
                if (c1 <= 0x80 || c1 >= 0xC0)
                    break;
            }
            return 0;
        }

        if (c1 < 0x80)
        {
            if (sz) *sz = 1;
            return c1;
        }

        if (str == end) return 0;
        const uint32_t c2 = uint8_t(*str++);
        if (c1 < 0xE0)
        {
            if (sz) *sz = 2;
            return (c2 & 0x3F) | ((c1 & 0x1F) << 6);
        }

        if (str == end) return 0;
        const uint32_t c3 = uint8_t(*str++);
        if (c1 < 0xF0)
        {
            if (sz) *sz = 3;
            return (c3 & 0x3F) | ((c2 & 0x3F) << 6) | ((c1 & 0x1F) << 12);
        }

        if (str == end) return 0;
        const uint32_t c4 = uint8_t(*str++);
        if (c1 < 0xF8)
        {
            if (sz) *sz = 4;
            return (c4 & 0x3F) | ((c3 & 0x3F) << 6) | ((c2 & 0x3F) << 12) | ((c1 & 0x7) << 18);
        }

        if (str == end) return 0;
        const uint32_t c5 = uint8_t(*str++);
        if (c1 < 0xFC)
        {
            if (sz) *sz = 5;
            return (c5 & 0x3F) | ((c4 & 0x3F) << 6) | ((c3 & 0x3F) << 12) | ((c2 & 0x3F) << 18) | ((c1 & 0x3) << 24);
        }

        if (str == end) return 0;
        const uint32_t c6 = uint8_t(*str++);
        if (sz) *sz = 6;
        return (c6 & 0x3F) | ((c5 & 0x3F) << 6) | ((c4 & 0x3F) << 12) | ((c3 & 0x3F) << 18) | ((c2 & 0x3F) << 24) | ((c1 & 0x1) << 30);
    }

    inline bool IsUtf8Trail(uint8_t c) { return (c >> 6) == 0x2; }

    inline uint32_t SkipUtf8Left(const char *&str, const char * const start, size_t *sz = 0)
    {
        if (sz) *sz = 0;
        if (str == start) return 0;
        str--;
        size_t size = 1;
        while (IsUtf8Trail(*str)) { str--; size++; }
        if (sz) *sz = size;
        const char *s = str;
        return DecodeUtf8(s, 0);
    }

    /// Copies UTF8 string from \c src  to \c dst. The \c dstSize
    /// specifies the total size of the \c dst buffer. The \c dst will
    /// be filled with '\0', if \c scr is not long enough to fill it.
    /// Returns the number of bytes copied.
    inline size_t CopyUtf8_N(char *dest, const char *src, size_t destSize)
    {
        const size_t slen = StringLength(src);
        const char * const end = src + slen;
        const char * const dend = dest + destSize;
        size_t sz = 0;
        while (src != end && dest != dend)
            sz += CopyUtf8Codepoint(src, end, dest, dend);
        while (dest != dend)
            *dest++ = 0;
        return sz;
    }

    /// Copies UTF8 string from \c src  to \c dst. The \c dstSize
    /// specifies the total size of the \c dst buffer. The amount
    /// of copied bytes is \c min(srclength, dstSize).
    /// Returns the number of bytes copied.
    inline size_t CopyUtf8(char *dest, const char *src, size_t destSize)
    {
        const size_t slen = StringLength(src);
        const char * const end = src + slen;
        const char * const dend = dest + destSize;
        size_t sz = 0;
        while (src != end && dest != dend)
            sz += CopyUtf8Codepoint(src, end, dest, dend);
        return sz;
    }



    struct AsciiTraits
    {
        static uint32_t Decode(const char *&str, const char * const end)
        { return (str != end) ? *str++ : 0; }

        static size_t Copy(char *dest, const char *src, size_t destSize)
        { return CopyString(dest, src, destSize); }

        static size_t CopyN(char *dest, const char *src, size_t destSize)
        { return CopyStringN(dest, src, destSize); }

        static uint32_t SkipRight(const char *&str, const char * const end, size_t *sz = 0)
        {
            if (sz) *sz = 0;
            if (str == end) return 0;
            if (sz) *sz = 1;
            return Decode(str, end);
        }

        static uint32_t SkipLeft(const char *&str, const char * const start, size_t *sz = 0)
        {
            if (sz) *sz = 0;
            if (str == start) return 0;
            if (sz) *sz = 1;
            return *--str;
        }
    };

    struct Utf8Traits
    {
        static uint32_t Decode(const char *&str, const char * const end)
        { return DecodeUtf8(str, end); }

        static size_t Copy(char *dest, const char *src, size_t dstSize)
        { return CopyUtf8(dest, src, dstSize); }

        static size_t CopyN(char *dest, const char *src, size_t dstSize)
        { return CopyUtf8_N(dest, src, dstSize); }

        static uint32_t SkipRight(const char *&str, const char * const end, size_t *sz = 0)
        { return SkipUtf8Right(str, end, sz); }

        static uint32_t SkipLeft(const char *&str, const char * const start, size_t *sz = 0)
        { return SkipUtf8Left(str, start, sz); }
    };

} // rob

#endif // H_ROB_STRING_H
