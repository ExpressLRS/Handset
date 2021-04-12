
#pragma once

#include <stdio.h>

class SerialWrapper
{
public:
    void print(const char * str);
    void println(const char * str);
    void print(unsigned int x);
    void println(unsigned int x);
    void println(void);

};

// =============================================================
// impl for inlining

void SerialWrapper::print(const char * str)
{
    printf("%s", str);
}

void SerialWrapper::println(const char * str)
{
    printf("%s\n\r", str);
}

void SerialWrapper::print(const unsigned int x)
{
    printf("%u", x);
}

void SerialWrapper::println(const unsigned int x)
{
    printf("%u\n\r", x);
}

void SerialWrapper::println(void)
{
    printf("\n\r");
}




// The usage we're trying to mimick had a global instance called 'Serial'
SerialWrapper Serial;

