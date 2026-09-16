//==============================================================================
//
//	hcl.h - Seiko Epson Hardware Control Library
//
//	This layer of indirection is added to allow the sample code to call
//  generic	functions to work on multiple hardware platforms
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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifndef OK
#define OK (true)
#endif

#ifndef NG
#define NG (false)
#endif

#define FATAL                                                          \
  do {                                                                 \
    fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", __LINE__, \
            __FILE__, errno, strerror(errno));                         \
    exit(1);                                                           \
  } while (0)

#ifdef __cplusplus
extern "C" {
#endif

int seInit(void);
int seRelease(void);
void seDelayMS(uint32_t millis);
void seDelayMicroSecs(uint32_t micros);

#ifdef __cplusplus
}
#endif
