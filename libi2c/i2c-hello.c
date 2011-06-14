/****************************************************************************
*
*   Copyright (c) 2006 Dave Hylands     <dhylands@gmail.com>
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation.
*
*   Alternatively, this software may be distributed under the terms of BSD
*   license.
*
*   See README and COPYING for more details.
*
****************************************************************************/
/**
*
*   @file   i2c-hello.c
*
*   @brief  Simple i2c program
*
****************************************************************************/

// ---- Include Files -------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <sys/timeb.h>

#include "i2c-dev.h"
#include "i2c-api.h"
#include "i2c-io-api.h"


// ---- Public Variables ----------------------------------------------------

// ---- Private Constants and Types -----------------------------------------

// ---- Private Variables ---------------------------------------------------

int gI2cAddr    = 0x55;

// ---- Private Function Prototypes -----------------------------------------

// ---- Functions -----------------------------------------------------------

//***************************************************************************
/**
*   Main entry point
*/

int main( int argc, char **argv )
{
    const char *i2cDevName = "/dev/i2c-3";
    int         i2cDev;
    uint8_t     cmd;
    uint8_t    result;
    int         rc;

    // Try to open the i2c device

    if (( i2cDev = open( i2cDevName, O_RDWR )) < 0 )
    {
        fprintf(stderr, "Error  opening '%s': %s\n", i2cDevName, strerror( errno ));
        exit( 1 );
    }

    // Indicate which slave we wish to speak to

    I2cSetSlaveAddress( i2cDev, gI2cAddr, I2C_NO_CRC );

    // Now issue the read command

    cmd = 0x35;
    result = 0xaa;

    if (( rc = I2cWriteByte( i2cDev, cmd, result )) != 0 )
    {
        fprintf(stderr, "I2cWriteByte failed: %d\n", rc );
        fprintf(stderr, "errno: %d\n", errno);
        exit( 1 );
    }

    // The gumstix is little endian, and most i2c devices send the data
    // in big-endian format, so we need to byte swap.

    //result = ( result >> 8 ) | ( result << 8 );

    //printf( "Command: 0x%02x returned a result of 0x%04x\n", cmd, result );

    close( i2cDev );

    return 0;

} // main

