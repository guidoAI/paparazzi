/*
 * Copyright (C) Kimberly McGuire
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/stereocam/stereocam2state/stereocam2state.c"
 * @author Kimberly McGuire
 * This module sends the data retreived from an external stereocamera modules, to the state filter of the drone. This is done so that the guidance modules can use that information for couadcopter
 */

#include "modules/stereocam/stereocam2state/stereocam2state.h"

#include "subsystems/abi.h"
#include "subsystems/datalink/telemetry.h"

#ifndef STEREOCAM2STATE_SENDER_ID
#define STEREOCAM2STATE_SENDER_ID ABI_BROADCAST
#endif

#ifndef STEREOCAM2STATE_RECEIVED_DATA_TYPE
#define STEREOCAM2STATE_RECEIVED_DATA_TYPE 1
#endif

#include "subsystems/datalink/telemetry.h"

// define for textons:
#define n_textons_stereoboard 10

void stereocam_to_state(void);

void stereo_to_state_init(void)
{

}

void stereo_to_state_periodic(void)
{
  if (stereocam_data.fresh) {
	  stereocam_to_state();
    stereocam_data.fresh = 0;
  }
}

void stereocam_to_state(void)
{

  // Get info from stereocam data
  // 0 = stereoboard's #define SEND_EDGEFLOW
  // 1 = stereoboard's #define SEND_DIVERGENCE_AND_TEXTONS
#if STEREOCAM2STATE_RECEIVED_DATA_TYPE == 0
  // opticflow
  int16_t div_x = (int16_t)stereocam_data.data[0] << 8;
  div_x |= (int16_t)stereocam_data.data[1];
  int16_t flow_x = (int16_t)stereocam_data.data[2] << 8;
  flow_x |= (int16_t)stereocam_data.data[3];
  int16_t div_y = (int16_t)stereocam_data.data[4] << 8;
  div_y |= (int16_t)stereocam_data.data[5];
  int16_t flow_y = (int16_t)stereocam_data.data[6] << 8;
  flow_y |= (int16_t)stereocam_data.data[7];

  float fps = (float)stereocam_data.data[9];
  //int8_t agl = stereocam_data.data[8]; // in cm

  // velocity
  int16_t vel_x_int = (int16_t)stereocam_data.data[10] << 8;
  vel_x_int |= (int16_t)stereocam_data.data[11];
  int16_t vel_y_int = (int16_t)stereocam_data.data[12] << 8;
  vel_y_int |= (int16_t)stereocam_data.data[13];

  int16_t RES = 100;

  float vel_x = (float)vel_x_int / RES;
  float vel_y = (float)vel_y_int / RES;

  // Derotate velocity and transform from frame to body coordinates
  // TODO: send resolution directly from stereocam

  float vel_body_x = - vel_x;
  float vel_body_y = vel_y;

  //Send velocity estimate to state
  //TODO:: Make variance dependable on line fit error, after new horizontal filter is made
  uint32_t now_ts = get_sys_time_usec();

  if (!(abs(vel_body_x) > 0.5 || abs(vel_body_x) > 0.5))
  {
    AbiSendMsgVELOCITY_ESTIMATE(STEREOCAM2STATE_SENDER_ID, now_ts,
                                vel_body_x,
                                vel_body_y,
                                0.0f,
                                0.3f
                               );
  }

  // Reusing the OPTIC_FLOW_EST telemetry messages, with some values replaced by 0

  uint16_t dummy_uint16 = 0;
  int16_t dummy_int16 = 0;
  float dummy_float = 0;

  DOWNLINK_SEND_OPTIC_FLOW_EST(DefaultChannel, DefaultDevice, &fps, &dummy_uint16, &dummy_uint16, &flow_x, &flow_y, &dummy_int16, &dummy_int16,
		  &vel_x, &vel_y,&dummy_float, &dummy_float, &dummy_float);
#else
  // get the textons from the message:
  uint8_t i;
  uint8_t histogram[n_textons_stereoboard];
  for(i = 0; i < n_textons_stereoboard; i++) {
    histogram[i] = stereocam_data.data[i];
  }
  // send to ground station:
  DOWNLINK_SEND_TEXTONS(DefaultChannel, DefaultDevice, &histogram[0], &histogram[1], &histogram[2], &histogram[3], &histogram[4], &histogram[5],
                             &histogram[6], &histogram[7], &histogram[8], &histogram[9]);
  // broadcast internally:
  AbiSendMsgTEXTONS(STEREOCAM2STATE_SENDER_ID, histogram[0], histogram[1], histogram[2], histogram[3], histogram[4], histogram[5], histogram[6], histogram[7], histogram[8], histogram[9]);

  // get the divergence from the message:
  float divergence = (float)stereocam_data.data[n_textons_stereoboard] - 128;
  // TODO: multiply divergence with a magical factor:

  uint8_t dummy_uint8 = 0;
  uint16_t dummy_uint16 = 0;
  int16_t dummy_int16 = 0;
  float dummy_float = 0;

  // send to ground station:
  DOWNLINK_SEND_OPTIC_FLOW_EST(DefaultChannel, DefaultDevice, &dummy_float, &dummy_uint16, &dummy_uint16, &dummy_int16, &dummy_int16, &dummy_int16, &dummy_int16,
		  &dummy_float, &dummy_float, &divergence, &dummy_float, &dummy_float);

  // broadcast internally:
  uint32_t now_ts = get_sys_time_usec(); // include std.h?
  AbiSendMsgOPTICAL_FLOW(STEREOCAM2STATE_SENDER_ID, now_ts,
                           dummy_int16,
                           dummy_int16,
                           dummy_int16,
                           dummy_int16,
                           dummy_uint8,// FIXME, scale to some quality measure 0-255
                           divergence,
                           dummy_float);

#endif

}
