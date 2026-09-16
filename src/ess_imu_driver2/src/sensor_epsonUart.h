//==============================================================================
//
//  sensor_epsonUart.h - Epson IMU sensor definitions for UART interface
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
#include "hcl_uart.h"
#include "sensor_epsonCommon.h"

// UART Interface Timing
// TWRITERATE/TREADRATE = 200us min @ 460800 BAUD, 1 command = 3 bytes =
// 3 * 22us = 66us TSTALL = 200us - 66us = 134us
#define EPSON_STALL 134  // Microseconds

// Required delay between bus cycles for serial timings
#define epsonStall() seDelayMicroSecs(EPSON_STALL)
