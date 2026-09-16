//==============================================================================
//
//  sensor_epsonSpi.h - Epson IMU sensor definitions for SPI interface
//
//
//  THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  NONINFRINGEMENT, SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A
//  PARTICULAR PURPOSE. IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE
//  OR CLAIM, ARISING FROM OR IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE
//  SOFTWARE.
//
//==============================================================================
#pragma once

#include <stdio.h>

#include "hcl.h"
#include "hcl_gpio.h"
#include "hcl_spi.h"
#include "sensor_epsonCommon.h"

// SPI Interface Timing
// TREADRATE = 40 us min, TCYCLERATE @ 0.5MHz = 32 uS, TSTALL must be at least
// (40us - 32us) or 20uS which ever is GREATER (20uS)
#define EPSON_STALL 20   // Microseconds,
#define BURST_STALL1 45  // Microseconds
#define BURST_STALL2 4   // Microseconds

// Required delay between bus cycles for serial timings
#define epsonStall() seDelayMicroSecs(EPSON_STALL)

// For delay after issuing Burst Read Cmd
#define burstStall1() seDelayMicroSecs(BURST_STALL1)
// For delay on consecutive cycles after Burst Read Cmd
#define burstStall2() seDelayMicroSecs(BURST_STALL2)

#define selEpson() gpioClr(EPSON_CS)
#define deselEpson() gpioSet(EPSON_CS)
