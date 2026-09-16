//==============================================================================
//
//  main_helper.c - Epson IMU helper functions for console utilities
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

#include <stdio.h>

#include "main_helper.h"
#include "sensor_epsonCommon.h"

/*****************************************************************************
** Function name:       printHeaderRow
** Description:         Output header row based on EpsonOptions
** Parameters:          fp - file pointer to send data
**                      EpsonOptions* - pointer to struct describing the IMU
**                                      configuration.
** Return value:        None
** Notes:
*****************************************************************************/
void printHeaderRow(FILE* fp, const struct EpsonOptions* options) {
  fprintf(fp, "\r\nsample[dec]");
  if (options->flag_out) {
    fprintf(fp, ", ndflags[hex]");
  }

  if (options->temp_out) {
    fprintf(fp, ", tempC[degC]");
  }

  if (options->gyro_out) {
    fprintf(fp, ", gx[deg/s], gy[deg/s], gz[deg/s]");
  }

  if (options->accel_out) {
    fprintf(fp, ", ax[mG], ay[mG], az[mG]");
  }

  if (options->gyro_delta_out) {
    fprintf(fp, ", dax[deg], day[deg], daz[deg]");
  }

  if (options->accel_delta_out) {
    fprintf(fp, ", dvx[m/s], dvy[m/s], dvz[m/s]");
  }

  if (options->qtn_out) {
    fprintf(fp, ", qtn0, qtn1, qtn2, qtn3");
  }

  if (options->atti_out) {
    fprintf(fp, ", ang1[deg], ang2[deg], ang3[deg]");
  }

  if (options->gpio_out) {
    fprintf(fp, ", gpio[hex]");
  }

  if (options->count_out) {
    fprintf(fp, ", count");
  }
  fprintf(fp, "\n");
}

/*****************************************************************************
** Function name:       printSensorRow
** Description:         Prints formatted row of sensor data based on
**                      EpsonOptions
** Parameters:          fp - file pointer to send data
**                      EpsonOptions* - pointer to struct describing the IMU
**                                      configuration.
**                      EpsonData* - pointer to struct that contains scaled
**                                   sensor
**                                   data
**                      sample_count - index for the sample row
** Return value:        none
** Notes:
******************************************************************************/
void printSensorRow(FILE* fp, const struct EpsonOptions* options,
                    const struct EpsonData* epson_data, int sample_count) {
  fprintf(fp, "%08d", sample_count);
  if (options->flag_out) {
    fprintf(fp, ", %04x", epson_data->ndflags);
  }

  if (options->temp_out) {
    fprintf(fp, ", %8.3f", epson_data->temperature);
  }

  if (options->gyro_out) {
    fprintf(fp, ", %8.5f, %8.5f, %8.5f", epson_data->gyro_x * RAD2DEG,
            epson_data->gyro_y * RAD2DEG, epson_data->gyro_z * RAD2DEG);
  }

  if (options->accel_out) {
    fprintf(fp, ", %8.5f, %8.5f, %8.5f", epson_data->accel_x * MPS22MG,
            epson_data->accel_y * MPS22MG, epson_data->accel_z * MPS22MG);
  }

  if (options->gyro_delta_out) {
    fprintf(fp, ", %8.5f, %8.5f, %8.5f", epson_data->gyro_delta_x * RAD2DEG,
            epson_data->gyro_delta_y * RAD2DEG,
            epson_data->gyro_delta_z * RAD2DEG);
  }

  if (options->accel_delta_out) {
    fprintf(fp, ", %8.5f, %8.5f, %8.5f", epson_data->accel_delta_x,
            epson_data->accel_delta_y, epson_data->accel_delta_z);
  }

  if (options->qtn_out) {
    fprintf(fp, ", %8.8f, %8.8f, %8.8f, %8.8f", epson_data->qtn0,
            epson_data->qtn1, epson_data->qtn2, epson_data->qtn3);
  }

  if (options->atti_out) {
    fprintf(fp, ", %8.3f, %8.3f, %8.3f", epson_data->ang1 * RAD2DEG,
            epson_data->ang2 * RAD2DEG, epson_data->ang3 * RAD2DEG);
  }

  if (options->gpio_out) {
    fprintf(fp, ", %04x", epson_data->gpio);
  }

  if (options->count_out) {
    fprintf(fp, ", %09d", epson_data->count);
  }
  fprintf(fp, "\n");
}
