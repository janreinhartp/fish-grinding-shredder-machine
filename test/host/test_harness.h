#pragma once
#include <stdio.h>

extern int g_failures;

#define EXPECT(cond)                                                         \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

#define EXPECT_EQ_INT(a, b)                                                  \
    do {                                                                     \
        long long _a = (long long)(a);                                       \
        long long _b = (long long)(b);                                       \
        if (_a != _b) {                                                      \
            fprintf(stderr, "FAIL %s:%d: %s (%lld) != %s (%lld)\n",          \
                    __FILE__, __LINE__, #a, _a, #b, _b);                     \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)
