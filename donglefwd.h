#ifndef DONGLEFWD_H
#define DONGLEFWD_H
#include <stdio.h>

// #define DEBUG
// #define VERBOSE

int bDebug(FILE *stream, const char* format, ...);
int bInfo(FILE *stream, const char* format, ...);

typedef struct MOBOTdongle MOBOTdongle;

#endif
