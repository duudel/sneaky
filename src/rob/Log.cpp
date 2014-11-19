
#include "Log.h"

#include <cstdio>
//#include <SDL2/SDL_log.h>

namespace rob
{
namespace log
{

    void PrintValue(char value)
    { ::printf("%c", value); }

    void PrintValue(signed char value)
    { ::printf("%hi", short(value)); }

    void PrintValue(unsigned char value)
    { ::printf("%hu", (unsigned short)(value)); }

    void PrintValue(short value)
    { ::printf("%hi", value); }

    void PrintValue(unsigned short value)
    { ::printf("%hu", value); }

    void PrintValue(int value)
    { ::printf("%i", value); }

    void PrintValue(unsigned int value)
    { ::printf("%u", value); }

    void PrintValue(long long value)
    { ::printf("%li", long(value)); }

    void PrintValue(unsigned long long value)
    { ::printf("%lu", (unsigned long)(value)); }

    void PrintValue(float value)
    { ::printf("%f", value); }

    void PrintValue(double value)
    { ::printf("%f", value); }

    void PrintValue(const char *value)
    { ::printf("%s", value); }

    void PrintValue(char *value)
    { ::printf("%s", value); }

    void PrintValue(const void *value)
    { ::printf("%p", value); }

    void PrintValue(void *value)
    { ::printf("%p", value); }

//    void Print(char value)
//    { ::SDL_Log("%c", value); }
//
//    void Print(signed char value)
//    { ::SDL_Log("%ihh", value); }
//
//    void Print(unsigned char value)
//    { ::SDL_Log("%uhh", value); }
//
//    void Print(short value)
//    { ::SDL_Log("%ih", value); }
//
//    void Print(unsigned short value)
//    { ::SDL_Log("%uh", value); }
//
//    void Print(int value)
//    { ::SDL_Log("%i", value); }
//
//    void Print(unsigned int value)
//    { ::SDL_Log("%u", value); }
//
//    void Print(long long value)
//    { ::SDL_Log("%ill", value); }
//
//    void Print(unsigned long long value)
//    { ::SDL_Log("%ull", value); }
//
//    void Print(const char *value)
//    { ::SDL_Log("%s", value); }

} // log
} // rob
