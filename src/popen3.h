#ifndef POPEN3_H
#define POPEN3_H

#include <stdio.h>
#include <unistd.h>

typedef struct {
    pid_t pid;
    FILE* in;
    FILE* out;
    FILE* err;
} process_t;

/* Basically the same thing as popen(3), but file streams for all three
 * standard I/O streams are captured. Referring to the return value's data
 * members:
 *   - process_t::pid will be -1 on error
 *   - writing to process_t::in will send data to the process' stdin
 *   - writing to process_t::out will send data to the process' stdout
 *   - writing to process_t::err will send data to the process' stderr
 *
 * The return value of this command must be passed to pclose3().
 */
process_t popen3 (const char* cmd);
int pclose3 (process_t proc);

#endif
