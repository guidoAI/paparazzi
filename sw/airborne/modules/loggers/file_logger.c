/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger.h"

#include <stdio.h>
#include "std.h"

#include "subsystems/imu.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance/guidance_indi.h"
#include "state.h"
#include "subsystems/actuators.h"
#include "generated/modules.h"
// #include "modules/calibration/mag_calib_ukf.c"

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif

#ifndef FILE_LOGGER_DATETIME_NAME
#define FILE_LOGGER_DATETIME_NAME 0
#endif

#if FILE_LOGGER_DATETIME_NAME
#include <time.h>
#endif

/** The file pointer */
static FILE *file_logger = NULL;

/** Start the file logger and open a new file */
void file_logger_start(void)
{
  uint32_t counter = 0;
  char filename[512];

  // Check for available files
#if FILE_LOGGER_DATETIME_NAME
  time_t timer;
  char date_buffer[26];
  struct tm* tm_info;
  timer = time(NULL);
  tm_info = localtime(&timer);
  strftime(date_buffer, 26, "%Y_%m_%d__%H_%M_%S", tm_info);
  sprintf(filename, "%s/%s_%s_%05d.csv", STRINGIFY(FILE_LOGGER_PATH), AIRFRAME_NAME, date_buffer, counter);
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);
    counter++;
    sprintf(filename, "%s/%s_%s_%05d.csv", STRINGIFY(FILE_LOGGER_PATH), AIRFRAME_NAME, date_buffer, counter);
  }
  file_logger = fopen(filename, "w");
#else
  sprintf(filename, "%s/%s_%05d.csv", STRINGIFY(FILE_LOGGER_PATH),STRINGIFY(AIRFRAME_NAME), counter);
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);
    counter++;
    sprintf(filename, "%s/%s_%05d.csv", STRINGIFY(FILE_LOGGER_PATH),STRINGIFY(AIRFRAME_NAME), counter);
  }
  file_logger = fopen(filename, "w");
#endif
  uint8_t max_retries = 10;
  uint8_t cur_try = 0;
  while(file_logger == NULL && cur_try < max_retries){
	  cur_try++;
  }
  if (file_logger != NULL) {
    fprintf(
      file_logger,
         "counter,"
         "in_flight,"
         "quat_qi,"
         "quat_qx,"
         "quat_qy,"
         "quat_qz,"
         "actuators_bebop_rpm_ref_0,"
         "actuators_bebop_rpm_ref_1,"
         "actuators_bebop_rpm_ref_2,"
         "actuators_bebop_rpm_ref_3,"
         "actuators_bebop_rpm_obs_0,"
         "actuators_bebop_rpm_obs_1,"
         "actuators_bebop_rpm_obs_2,"
         "actuators_bebop_rpm_obs_3,"
         "imu_temperature,"
         "imu_bebop_filtered_temperature,"
         "imu_gyro_unscaled_p,"
         "imu_gyro_unscaled_q,"
         "imu_gyro_unscaled_r,"
         "imu_gyro_p,"
         "imu_gyro_q,"
         "imu_gyro_r,"
         "imu_accel_unscaled_x,"
         "imu_accel_unscaled_y,"
         "imu_accel_unscaled_z,"
         "imu_accel_x,"
         "imu_accel_y,"
         "imu_accel_z,"
         "imu_mag_unscaled_x,"
         "imu_mag_unscaled_y,"
         "imu_mag_unscaled_z,"
         "imu_mag_x,"
         "imu_mag_y,"
         "imu_mag_z,"
         "magneto_psi_f,"
         "eulerAngles_psi,"
         "mag_calib_state_0,"
         "mag_calib_state_1,"
         "mag_calib_state_2,"
         "mag_calib_state_3,"
         "mag_calib_state_4,"
         "mag_calib_state_5,"
         "mag_calib_state_6,"
         "mag_calib_state_7,"
         "mag_calib_state_8,"
         //"mag_calib_state_9,"
         //"mag_calib_state_10,"
         //"mag_calib_state_11"
          "sp_accel_x,"
          "sp_accel_y,"
          "sp_accel_z\n"
    );
    logger_file_file_logger_periodic_status = MODULES_RUN;
  }
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void)
{
  if (file_logger != NULL) {
    fclose(file_logger);
    file_logger = NULL;
  }
  logger_file_file_logger_periodic_status = MODULES_IDLE;
}

/** Log the values to a csv file */
void file_logger_periodic(void)
{
  if (file_logger == NULL) {
    return;
  }
  static uint32_t counter;
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  struct FloatEulers* eulerAngles = stateGetNedToBodyEulers_f();

  fprintf(file_logger,
          //"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%0.4f,%0.4f,%d,%d,%d,%0.4f,%0.4f,%0.4f,%d,%d,%d,%0.4f,%0.4f,%0.4f,%d,%d,%d,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f\n",
          "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%0.4f,%0.4f,%d,%d,%d,%0.4f,%0.4f,%0.4f,%d,%d,%d,%0.4f,%0.4f,%0.4f,%d,%d,%d,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f\n",
          counter,
          autopilot.in_flight,
          quat->qi,
          quat->qx,
          quat->qy,
          quat->qz,
          actuators_bebop.rpm_ref[0],
          actuators_bebop.rpm_ref[1],
          actuators_bebop.rpm_ref[2],
          actuators_bebop.rpm_ref[3],
          actuators_bebop.rpm_obs[0],
          actuators_bebop.rpm_obs[1],
          actuators_bebop.rpm_obs[2],
          actuators_bebop.rpm_obs[3],
          imu_bebop.mpu.temp,
          imu_bebop_filtered_temperature,
          imu.gyro_unscaled.p,
          imu.gyro_unscaled.q,
          imu.gyro_unscaled.r,
          RATE_FLOAT_OF_BFP(imu.gyro.p),
          RATE_FLOAT_OF_BFP(imu.gyro.q),
          RATE_FLOAT_OF_BFP(imu.gyro.r),
          imu.accel_unscaled.x,
          imu.accel_unscaled.y,
          imu.accel_unscaled.z,
          ACCEL_FLOAT_OF_BFP(imu.accel.x),
          ACCEL_FLOAT_OF_BFP(imu.accel.y),
          ACCEL_FLOAT_OF_BFP(imu.accel.z),
          imu.mag_unscaled.x,
          imu.mag_unscaled.y,
          imu.mag_unscaled.z,
          MAG_FLOAT_OF_BFP(imu.mag.x),
          MAG_FLOAT_OF_BFP(imu.mag.y),
          MAG_FLOAT_OF_BFP(imu.mag.z),
          0.0f,
          eulerAngles->psi,
          0.0f,
          0.0f,
          0.0f,
          0.0f,
          0.0f,
          0.0f,
          0.0f,
          0.0f,
          0.0f,
          //mag_calib.state[9],
          //mag_calib.state[10],
          //mag_calib.state[11],
          0.0f,
          0.0f,
          0.0f
         );
  counter++;
}
