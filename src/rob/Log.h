
#ifndef H_ROB_LOG_H
#define H_ROB_LOG_H

namespace rob
{
namespace log
{

    void PrintValue(char value);
    void PrintValue(signed char value);
    void PrintValue(unsigned char value);
    void PrintValue(short value);
    void PrintValue(unsigned short value);
    void PrintValue(int value);
    void PrintValue(unsigned int value);
    void PrintValue(long long value);
    void PrintValue(unsigned long long value);
    void PrintValue(float value);
    void PrintValue(double value);
    void PrintValue(const char *value);
    void PrintValue(char *value);
    void PrintValue(const void *value);
    void PrintValue(void *value);

    template <class T>
    void PrintValue(const T &value)
    {
        PrintValue(value.c_str());
    }

    template <class T>
    void Print(T&& value)
    {
        PrintValue(value);
    }

    template <class T, class... Args>
    void Print(T&& value, Args&& ...args)
    {
        PrintValue(value);
        Print(args...);
    }

    template <class... Args>
    void Trace(Args&& ...args)
    {
    #ifdef ROB_TRACE
        PrintValue("Trace: ");
        Print(args...);
        PrintValue('\n');
    #endif // ROB_TRACE
    }

    template <class... Args>
    void Debug(Args&& ...args)
    {
    #ifdef ROB_DEBUG
        PrintValue("Debug: ");
        Print(args...);
        PrintValue('\n');
    #endif
    }

    template <class... Args>
    void Info(Args&& ...args)
    {
        PrintValue("Info: ");
        Print(args...);
        PrintValue('\n');
    }

    template <class... Args>
    void Warning(Args&& ...args)
    {
        PrintValue("Warning: ");
        Print(args...);
        PrintValue('\n');
    }

    template <class... Args>
    void Error(Args&& ...args)
    {
        PrintValue("Error: ");
        Print(args...);
        PrintValue('\n');
    }

} // log
} // rob

#endif // H_ROB_LOG_H

