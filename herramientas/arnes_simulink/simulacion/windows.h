/* Imitación de windows.h: sólo el contador de alta resolución */
#pragma once
#include <ctime>
typedef long long LONGLONG;
typedef union { LONGLONG QuadPart; } LARGE_INTEGER;
static inline int QueryPerformanceFrequency(LARGE_INTEGER *f) { f->QuadPart = 1000000000LL; return 1; }
static inline int QueryPerformanceCounter(LARGE_INTEGER *c)
{
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    c->QuadPart = (LONGLONG)t.tv_sec * 1000000000LL + t.tv_nsec;
    return 1;
}
