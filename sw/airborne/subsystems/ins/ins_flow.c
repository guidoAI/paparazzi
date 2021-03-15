/*
 * Copyright (C) 2021 Guido de Croon <g.c.h.e.decroon@tudelft.nl>
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
 */

/**
 * @file subsystems/ins/ins_flow.c
 *
 * "Inertial" navigation system.
 */

#include "ins_flow.h"
#include "subsystems/abi.h"
#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "math/pprz_algebra_float.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "mcu_periph/sys_time.h"

#define DEBUG_INS_FLOW 1
#if DEBUG_INS_FLOW
#include "stdio.h"
#include "math/pprz_simple_matrix.h"
#define DEBUG_PRINT  printf
#define DEBUG_MAT_PRINT MAT_PRINT
//#define DEBUG_MAT_PRINT(...)
#else
#define DEBUG_PRINT(...)
#define DEBUG_MAT_PRINT(...)
#endif

#ifndef AHRS_ICQ_OUTPUT_ENABLED
#define AHRS_ICQ_OUTPUT_ENABLED TRUE
#endif
PRINT_CONFIG_VAR(AHRS_ICQ_OUTPUT_ENABLED)

/* default Gyro to use in INS */
#ifndef INS_FLOW_GYRO_ID
#define INS_FLOW_GYRO_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_FLOW_GYRO_ID)

/* default Accelerometer to use in INS */
#ifndef INS_FLOW_ACCEL_ID
#define INS_FLOW_ACCEL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_FLOW_ACCEL_ID)

/* default GPS to use in INS */
#ifndef INS_FLOW_GPS_ID
#define INS_FLOW_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(INS_FLOW_GPS_ID)

/* Use optical flow estimates */
#ifndef INS_OPTICAL_FLOW_ID
#define INS_OPTICAL_FLOW_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_OPTICAL_FLOW_ID)

// reading RPMs:
#ifndef INS_RPM_ID
#define INS_RPM_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(INS_RPM_ID)

/* All registered ABI events */
static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event gps_ev;
static abi_event body_to_imu_ev;
static abi_event ins_optical_flow_ev;
static abi_event ins_RPM_ev; ///< The sonar ABI event

/* All ABI callbacks */
static void gyro_cb(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro);
static void accel_cb(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel);
static void body_to_imu_cb(uint8_t sender_id, struct FloatQuat *q_b2i_f);
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);
void ins_optical_flow_cb(uint8_t sender_id, uint32_t stamp, int16_t flow_x,
                                   int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, float quality, float size_divergence);
static void ins_rpm_cb(uint8_t sender_id, uint16_t * rpm, uint8_t num_act);

/* Static local functions */
//static bool ahrs_icq_output_enabled;
static uint32_t ahrs_icq_last_stamp;
static uint8_t ahrs_flow_id = AHRS_COMP_ID_FLOW;  ///< Component ID for FLOW

static void set_body_state_from_quat(void);

struct InsFlow {

  // data elements for gps passthrough:
  struct LtpDef_i  ltp_def;
  bool ltp_initialized;

  /* output LTP NED */
  struct NedCoor_i ltp_pos;
  struct NedCoor_i ltp_speed;
  struct NedCoor_i ltp_accel;

  // vision measurements:
  float optical_flow_x;
  float optical_flow_y;
  float divergence;
  float vision_time; // perhaps better to use microseconds (us) instead of float in seconds
  bool new_flow_measurement;

  // RPMs:
  uint16_t RPM[8]; // max an octocopter
  uint8_t RPM_num_act;

};
struct InsFlow ins_flow;


// Kalman filter parameters and variables:
#define N_STATES_OF_KF 5

#define OF_V_IND 0
#define OF_ANGLE_IND 1
#define OF_ANGLE_DOT_IND 2
#define OF_Z_IND 3
#define OF_Z_DOT_IND 4

#define N_MEAS_OF_KF 2

#define OF_LAT_FLOW_IND 0
#define OF_DIV_FLOW_IND 1

float OF_X[N_STATES_OF_KF] = {0.};
float OF_Q[N_STATES_OF_KF][N_STATES_OF_KF] = {{0.}};
float OF_P[N_STATES_OF_KF][N_STATES_OF_KF] = {{0.}};
float OF_R[N_MEAS_OF_KF][N_MEAS_OF_KF] = {{0.}};

#define OF_N_ROTORS 4
float RPM_FACTORS[OF_N_ROTORS];

float of_time;
float of_prev_time;

/*
struct InsFlowState {
  // vector representation of state:
  // v, angle, angle_dot, z, z_dot


};
struct InsFlowState ins_flow_state;
*/

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#include "mcu_periph/sys_time.h"
#include "state.h"

// attitude part:
static void send_quat(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  pprz_msg_send_AHRS_QUAT_INT(trans, dev, AC_ID,
                              &ahrs_icq.weight,
                              &ahrs_icq.ltp_to_imu_quat.qi,
                              &ahrs_icq.ltp_to_imu_quat.qx,
                              &ahrs_icq.ltp_to_imu_quat.qy,
                              &ahrs_icq.ltp_to_imu_quat.qz,
                              &(quat->qi),
                              &(quat->qx),
                              &(quat->qy),
                              &(quat->qz),
                              &ahrs_flow_id);
}

static void send_euler(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Eulers ltp_to_imu_euler;
  int32_eulers_of_quat(&ltp_to_imu_euler, &ahrs_icq.ltp_to_imu_quat);
  struct Int32Eulers *eulers = stateGetNedToBodyEulers_i();
  pprz_msg_send_AHRS_EULER_INT(trans, dev, AC_ID,
                               &ltp_to_imu_euler.phi,
                               &ltp_to_imu_euler.theta,
                               &ltp_to_imu_euler.psi,
                               &(eulers->phi),
                               &(eulers->theta),
                               &(eulers->psi),
                               &ahrs_flow_id);

}

static void send_bias(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_AHRS_GYRO_BIAS_INT(trans, dev, AC_ID,
                                   &ahrs_icq.gyro_bias.p, &ahrs_icq.gyro_bias.q,
                                   &ahrs_icq.gyro_bias.r, &ahrs_flow_id);
}

static void send_geo_mag(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatVect3 h_float;
  h_float.x = MAG_FLOAT_OF_BFP(ahrs_icq.mag_h.x);
  h_float.y = MAG_FLOAT_OF_BFP(ahrs_icq.mag_h.y);
  h_float.z = MAG_FLOAT_OF_BFP(ahrs_icq.mag_h.z);
  pprz_msg_send_GEO_MAG(trans, dev, AC_ID,
                        &h_float.x, &h_float.y, &h_float.z, &ahrs_flow_id);
}

static void send_filter_status(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t mde = 3;
  uint16_t val = 0;
  if (!ahrs_icq.is_aligned) { mde = 2; }
  uint32_t t_diff = get_sys_time_usec() - ahrs_icq_last_stamp;
  /* set lost if no new gyro measurements for 50ms */
  if (t_diff > 50000) { mde = 5; }
  pprz_msg_send_STATE_FILTER_STATUS(trans, dev, AC_ID, &ahrs_flow_id, &mde, &val);
}

// ins part
static void send_ins(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_INS(trans, dev, AC_ID,
                    &ins_flow.ltp_pos.x, &ins_flow.ltp_pos.y, &ins_flow.ltp_pos.z,
                    &ins_flow.ltp_speed.x, &ins_flow.ltp_speed.y, &ins_flow.ltp_speed.z,
                    &ins_flow.ltp_accel.x, &ins_flow.ltp_accel.y, &ins_flow.ltp_accel.z);
}

/*static void send_ins_z(struct transport_tx *trans, struct link_device *dev)
{
  static float fake_baro_z = 0.0;
  pprz_msg_send_INS_Z(trans, dev, AC_ID,
                      (float *)&fake_baro_z, &ins_flow.ltp_pos.z,
                      &ins_flow.ltp_speed.z, &ins_flow.ltp_accel.z);
}*/

static void send_ins_ref(struct transport_tx *trans, struct link_device *dev)
{
  static float fake_qfe = 0.0;
  if (ins_flow.ltp_initialized) {
    pprz_msg_send_INS_REF(trans, dev, AC_ID,
                          &ins_flow.ltp_def.ecef.x, &ins_flow.ltp_def.ecef.y, &ins_flow.ltp_def.ecef.z,
                          &ins_flow.ltp_def.lla.lat, &ins_flow.ltp_def.lla.lon, &ins_flow.ltp_def.lla.alt,
                          &ins_flow.ltp_def.hmsl, (float *)&fake_qfe);
  }
}


#endif

/*
static bool ahrs_icq_enable_output(bool enable)
{
  ahrs_icq_output_enabled = enable;
  return ahrs_icq_output_enabled;
}*/

/* Initialize the flow ins */
void ins_flow_init(void)
{

  //ahrs_icq_output_enabled = AHRS_ICQ_OUTPUT_ENABLED;
  ahrs_icq_init();
  //ahrs_register_impl(ahrs_icq_enable_output);

  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;
  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);
  struct LtpDef_i ltp_def;
  ltp_def_from_ecef_i(&ltp_def, &ecef_nav0);

  ltp_def_from_ecef_i(&ins_flow.ltp_def, &ecef_nav0);
  ins_flow.ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ins_flow.ltp_def);
  ins_flow.ltp_initialized = true;
  ins_flow.new_flow_measurement = false;

  // Extended Kalman filter:
  // initialize the state:
  OF_X[OF_Z_IND] = 1.0; // nonzero z

  // R-matrix, measurement noise (TODO: make params)
  OF_R[OF_LAT_FLOW_IND][OF_LAT_FLOW_IND] = 0.02;
  OF_R[OF_DIV_FLOW_IND][OF_DIV_FLOW_IND] = 0.02;
  // Q-matrix, actuation noise (TODO: make params)
  OF_Q[OF_V_IND][OF_V_IND] = 0.1;
  OF_Q[OF_ANGLE_IND][OF_ANGLE_IND] = 1.0 * (M_PI / 180.0f);
  OF_Q[OF_ANGLE_DOT_IND][OF_ANGLE_DOT_IND] = 100.0 * (M_PI / 180.0f);
  OF_Q[OF_Z_IND][OF_Z_IND] = 0.1;
  OF_Q[OF_Z_DOT_IND][OF_Z_DOT_IND] = 3.0;
  // P-matrix:
  OF_P[OF_V_IND][OF_V_IND] = 1.0;
  OF_P[OF_ANGLE_IND][OF_ANGLE_IND] = 10.0 * (M_PI / 180.0f);
  OF_P[OF_ANGLE_DOT_IND][OF_ANGLE_DOT_IND] = 10.0 * (M_PI / 180.0f);
  OF_P[OF_Z_IND][OF_Z_IND] = 1.0;
  OF_P[OF_Z_DOT_IND][OF_Z_DOT_IND] = 1.0;

  // based on a fit, factor * rpm^2:
  RPM_FACTORS[0] = 0.15*1E-7;
  RPM_FACTORS[1] = 0.17*1E-7;
  RPM_FACTORS[2] = 0.10*1E-7;
  RPM_FACTORS[3] = 0.12*1E-7;

  of_time = get_sys_time_float();
  of_prev_time = get_sys_time_float();

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_QUAT_INT, send_quat);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_EULER_INT, send_euler);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AHRS_GYRO_BIAS_INT, send_bias);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GEO_MAG, send_geo_mag);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STATE_FILTER_STATUS, send_filter_status);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS, send_ins);
  //register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_Z, send_ins_z);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INS_REF, send_ins_ref);
#endif

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_GYRO_INT32(INS_FLOW_GYRO_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(INS_FLOW_ACCEL_ID, &accel_ev, accel_cb);
  AbiBindMsgGPS(INS_FLOW_GPS_ID, &gps_ev, gps_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(ABI_BROADCAST, &body_to_imu_ev, body_to_imu_cb);
  AbiBindMsgOPTICAL_FLOW(INS_OPTICAL_FLOW_ID, &ins_optical_flow_ev, ins_optical_flow_cb);
  AbiBindMsgRPM(INS_RPM_ID, &ins_RPM_ev, ins_rpm_cb);
}

void ins_reset_local_origin(void)
{
  ltp_def_from_ecef_i(&ins_flow.ltp_def, &gps.ecef_pos);
  ins_flow.ltp_def.lla.alt = gps.lla_pos.alt;
  ins_flow.ltp_def.hmsl = gps.hmsl;
  stateSetLocalOrigin_i(&ins_flow.ltp_def);
  ins_flow.ltp_initialized = true;
}

void ins_optical_flow_cb(uint8_t sender_id UNUSED, uint32_t stamp, int16_t flow_x UNUSED,
                                   int16_t flow_y UNUSED,
                                   int16_t flow_der_x, int16_t flow_der_y, float quality UNUSED, float size_divergence)
{

  // TODO: make parameters:
  float subpixel_factor = 10.0f;
  float focal_x = 347.22;
  float new_time = ((float)stamp) / 1e6;
  float fps = 1.0f/(new_time-ins_flow.vision_time);
  ins_flow.optical_flow_x = (((float)flow_der_x)*fps)/(subpixel_factor*focal_x);
  ins_flow.optical_flow_y = (((float)flow_der_y)*fps)/(subpixel_factor*focal_x);
  ins_flow.divergence = 1.27*size_divergence*fps;
  //printf("Reading %f, %f, %f\n", optical_flow_x, optical_flow_y, divergence_vision);
  ins_flow.vision_time = new_time;
  ins_flow.new_flow_measurement = true;

}

void print_ins_flow_state(void) {
  printf("v = %f, angle = %f, angle_dot = %f, z = %f, z+_dot = %f.\n",
	 OF_X[OF_V_IND], OF_X[OF_ANGLE_IND], OF_X[OF_ANGLE_DOT_IND], OF_X[OF_Z_IND], OF_X[OF_Z_DOT_IND]);
}

void ins_flow_update(void)
{
  // we first make the simplest version, i.e., no gyro measurement, no moment estimate:

  if(!autopilot_in_flight()) {
      return;
  }



  // get the new time:
  of_time = get_sys_time_float();
  float dt = of_time - of_prev_time;
  DEBUG_PRINT("dt = %f.\n", dt);
  if(dt > 1.0f) {
      dt = 0.01f;
  }

  // predict the thrust and moment:
  float thrust = 0.0f;
  for(int i = 0; i < OF_N_ROTORS; i++) {
      thrust += RPM_FACTORS[i] * ins_flow.RPM[i]*ins_flow.RPM[i];
  }
  thrust -= 3.5f;
  DEBUG_PRINT("Thrust = %f\n", thrust);

  float mass = 0.400; // TODO: make parameter
  float moment = 0.0f; // for now assumed to be 0
  float Ix = 0.0018244; // TODO: make parameter
  float g = 9.81; // TODO: get a good definition from pprz

  // propagate the state with Euler integration:
  printf("Before prediction: ");
  print_ins_flow_state();
  // make sure that the right hand state terms appear before they change:
  OF_X[OF_V_IND] += dt * (thrust * sin(OF_X[OF_ANGLE_IND]) / mass);
  OF_X[OF_Z_IND] += dt * OF_X[OF_Z_DOT_IND];
  OF_X[OF_Z_DOT_IND] += dt * (thrust * cos(OF_X[OF_ANGLE_IND]) / mass - g);
  OF_X[OF_ANGLE_IND] += dt * OF_X[OF_ANGLE_DOT_IND];
  OF_X[OF_ANGLE_DOT_IND] += dt * (moment / Ix);

  // ensure that z is not 0 (or lower)
  if(OF_X[OF_Z_IND] < 1e-3) {
      OF_X[OF_Z_IND] = 1e-3;
  }

  printf("After prediction: ");
  print_ins_flow_state();

  // prepare the update and correction step:
  // we have to recompute these all the time, as they depend on the state:
  // discrete version of state transition matrix F: (ignoring t^2)
  float F[N_STATES_OF_KF][N_STATES_OF_KF] = {{0.}};
  for(int i = 0; i < N_STATES_OF_KF; i++) {
      F[i][i] = 1.0f;
  }
  F[OF_V_IND][OF_ANGLE_IND] = dt*(thrust*cos(OF_X[OF_ANGLE_IND])/mass);
  F[OF_ANGLE_IND][OF_ANGLE_DOT_IND] = dt*1.0f;
  F[OF_Z_IND][OF_Z_DOT_IND] = dt*1.0f;
  F[OF_Z_DOT_IND][OF_ANGLE_IND] = dt*(-thrust*sin(OF_X[OF_ANGLE_IND])/mass);

  // G matrix (whatever it may be):
  float G[N_STATES_OF_KF][N_STATES_OF_KF] = {{0.}};
  for(int i = 0; i < N_STATES_OF_KF; i++) {
        G[i][i] = dt;
  }

  // Jacobian observation matrix H:
  float H[N_MEAS_OF_KF][N_STATES_OF_KF] = {{0.}};
  // lateral flow:
  H[OF_LAT_FLOW_IND][OF_V_IND] = -cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/ OF_X[OF_Z_IND];
  H[OF_LAT_FLOW_IND][OF_ANGLE_IND] = OF_X[OF_V_IND]*sin(2*OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND]
					+ OF_X[OF_Z_DOT_IND]*cos(2*OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND];
  H[OF_LAT_FLOW_IND][OF_ANGLE_DOT_IND] = 1.0f;
  H[OF_LAT_FLOW_IND][OF_Z_IND] = OF_X[OF_V_IND]*cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/(OF_X[OF_Z_IND]*OF_X[OF_Z_IND])
				  - OF_X[OF_Z_DOT_IND]*sin(2*OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND];
  H[OF_LAT_FLOW_IND][OF_Z_DOT_IND] = sin(2*OF_X[OF_ANGLE_IND])/(2*OF_X[OF_Z_IND]);
  // divergence:
  H[OF_DIV_FLOW_IND][OF_V_IND] = -sin(2*OF_X[OF_ANGLE_IND])/(2*OF_X[OF_Z_IND]);
  H[OF_DIV_FLOW_IND][OF_ANGLE_IND] = -OF_X[OF_V_IND]*cos(OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND]
					+ OF_X[OF_Z_DOT_IND]*sin(2*OF_X[OF_ANGLE_IND]) / OF_X[OF_Z_IND];
  H[OF_DIV_FLOW_IND][OF_ANGLE_DOT_IND] = 0.0f;
  H[OF_DIV_FLOW_IND][OF_Z_IND] = OF_X[OF_V_IND]*sin(2*OF_X[OF_ANGLE_IND])/(2*OF_X[OF_Z_IND]*OF_X[OF_Z_IND])
				  + OF_X[OF_Z_DOT_IND]*cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/(OF_X[OF_Z_IND]*OF_X[OF_Z_IND]);
  H[OF_DIV_FLOW_IND][OF_Z_DOT_IND] = -cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND];

  // propagate uncertainty:
  // TODO: make pointers that don't change to init:
  MAKE_MATRIX_PTR(Phi, F, N_STATES_OF_KF);
  MAKE_MATRIX_PTR(P, OF_P, N_STATES_OF_KF);
  MAKE_MATRIX_PTR(Gamma, G, N_STATES_OF_KF);
  MAKE_MATRIX_PTR(Q, OF_Q, N_STATES_OF_KF);
  MAKE_MATRIX_PTR(R, OF_R, N_MEAS_OF_KF);
  MAKE_MATRIX_PTR(Jac, H, N_MEAS_OF_KF);

  DEBUG_PRINT("Phi:\n");
  DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, Phi);

  DEBUG_PRINT("P:\n");
  DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, P);

  DEBUG_PRINT("Gamma:\n");
  DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, G);

  DEBUG_PRINT("Q:\n");
  DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, Q);

  DEBUG_PRINT("R:\n");
  DEBUG_MAT_PRINT(N_MEAS_OF_KF, N_MEAS_OF_KF, R);

  DEBUG_PRINT("Jacobian:\n");
  DEBUG_MAT_PRINT(N_MEAS_OF_KF, N_STATES_OF_KF, Jac);

  // Corresponding MATLAB statement:    :O
  // P_k1_k = Phi_k1_k*P*Phi_k1_k' + Gamma_k1_k*Q*Gamma_k1_k';
  float _PhiT[N_STATES_OF_KF][N_STATES_OF_KF];
  MAKE_MATRIX_PTR(PhiT, _PhiT, N_STATES_OF_KF);
  float _P_PhiT[N_STATES_OF_KF][N_STATES_OF_KF];
  MAKE_MATRIX_PTR(PPhiT, _P_PhiT, N_STATES_OF_KF);
  float _Phi_P_PhiT[N_STATES_OF_KF][N_STATES_OF_KF];
  MAKE_MATRIX_PTR(PhiPPhiT, _Phi_P_PhiT, N_STATES_OF_KF);

  float_mat_transpose(PhiT, Phi, N_STATES_OF_KF, N_STATES_OF_KF);
  float_mat_mul(PPhiT, P, PhiT, N_STATES_OF_KF, N_STATES_OF_KF, N_STATES_OF_KF);
  float_mat_mul(PhiPPhiT, Phi, PPhiT, N_STATES_OF_KF, N_STATES_OF_KF, N_STATES_OF_KF);

  DEBUG_PRINT("Phi*P*PhiT:\n");
  DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, PhiPPhiT);

  float _GT[N_STATES_OF_KF][N_STATES_OF_KF];
  MAKE_MATRIX_PTR(GT, _GT, N_STATES_OF_KF);
  float _Q_GT[N_STATES_OF_KF][N_STATES_OF_KF];
  MAKE_MATRIX_PTR(QGT, _Q_GT, N_STATES_OF_KF);
  float _G_Q_GT[N_STATES_OF_KF][N_STATES_OF_KF];
  MAKE_MATRIX_PTR(GQGT, _G_Q_GT, N_STATES_OF_KF);

  float_mat_transpose(GT, Gamma, N_STATES_OF_KF, N_STATES_OF_KF);
  float_mat_mul(QGT, Q, GT, N_STATES_OF_KF, N_STATES_OF_KF, N_STATES_OF_KF);
  float_mat_mul(GQGT, Gamma, QGT, N_STATES_OF_KF, N_STATES_OF_KF, N_STATES_OF_KF);

  DEBUG_PRINT("Gamma*Q*GammaT:\n");
  DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, GQGT);

  float_mat_sum(P, PhiPPhiT, GQGT, N_STATES_OF_KF, N_STATES_OF_KF);
  DEBUG_PRINT("P:\n");
  DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, P);

  // if new measurement, correct state:
  if(ins_flow.new_flow_measurement) {

    // determine Kalman gain:
    // MATLAB statement:
    // S_k = Hx*P_k1_k*Hx' + R;
    float _JacT[N_STATES_OF_KF][N_MEAS_OF_KF];
    MAKE_MATRIX_PTR(JacT, _JacT, N_STATES_OF_KF);
    float _P_JacT[N_STATES_OF_KF][N_MEAS_OF_KF];
    MAKE_MATRIX_PTR(PJacT, _P_JacT, N_STATES_OF_KF);
    float _Jac_P_JacT[N_MEAS_OF_KF][N_MEAS_OF_KF];
    MAKE_MATRIX_PTR(JacPJacT, _Jac_P_JacT, N_MEAS_OF_KF);

    float_mat_transpose(JacT, Jac, N_MEAS_OF_KF, N_STATES_OF_KF);
    float_mat_mul(PJacT, P, JacT, N_STATES_OF_KF, N_STATES_OF_KF, N_MEAS_OF_KF);
    float_mat_mul(JacPJacT, Jac, PJacT, N_MEAS_OF_KF, N_STATES_OF_KF, N_MEAS_OF_KF);

    DEBUG_PRINT("Jac*P*JacT:\n");
    DEBUG_MAT_PRINT(N_MEAS_OF_KF, N_MEAS_OF_KF, JacPJacT);

    float _S[N_MEAS_OF_KF][N_MEAS_OF_KF];
    MAKE_MATRIX_PTR(S, _S, N_MEAS_OF_KF);
    float_mat_sum(S, JacPJacT, R, N_MEAS_OF_KF, N_MEAS_OF_KF);

    DEBUG_PRINT("S:\n");
    DEBUG_MAT_PRINT(N_MEAS_OF_KF, N_MEAS_OF_KF, S);

    // MATLAB statement:
    // K_k1 = P_k1_k*Hx' * inv(S_k);
    float _K[N_STATES_OF_KF][N_MEAS_OF_KF];
    MAKE_MATRIX_PTR(K, _K, N_STATES_OF_KF);
    float _INVS[N_MEAS_OF_KF][N_MEAS_OF_KF];
    MAKE_MATRIX_PTR(INVS, _INVS, N_MEAS_OF_KF);
    float_mat_invert(INVS, S, N_MEAS_OF_KF);
    float_mat_mul(K, PJacT, INVS, N_STATES_OF_KF, N_MEAS_OF_KF, N_MEAS_OF_KF);

    DEBUG_PRINT("K:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, N_MEAS_OF_KF, K);

    // Correct the state:
    // MATLAB:
    // Z_expected = [-v*cos(theta)*cos(theta)/z + zd*sin(2*theta)/(2*z) + thetad;
    //			(-v*sin(2*theta)/(2*z)) - zd*cos(theta)*cos(theta)/z];
    float Z_expected[N_MEAS_OF_KF];
    Z_expected[OF_LAT_FLOW_IND] = -OF_X[OF_V_IND]*cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND]
				   + OF_X[OF_Z_DOT_IND]*sin(2*OF_X[OF_ANGLE_IND])/(2*OF_X[OF_Z_IND])
				   + OF_X[OF_ANGLE_DOT_IND];
    Z_expected[OF_DIV_FLOW_IND] = -OF_X[OF_V_IND]*sin(2*OF_X[OF_ANGLE_IND])/(2*OF_X[OF_Z_IND])
				  -OF_X[OF_Z_DOT_IND]*cos(OF_X[OF_ANGLE_IND])*cos(OF_X[OF_ANGLE_IND])/OF_X[OF_Z_IND];

    //  i_k1 = Z - Z_expected;
    float innovation[N_MEAS_OF_KF][1];
    // TODO: should this be optical_flow_x or y for roll?
    innovation[OF_LAT_FLOW_IND][0] = ins_flow.optical_flow_x - Z_expected[OF_LAT_FLOW_IND];
    DEBUG_PRINT("Expected flow: %f, Real flow: %f.\n", Z_expected[OF_LAT_FLOW_IND], ins_flow.optical_flow_x);
    innovation[OF_DIV_FLOW_IND][0] = ins_flow.divergence - Z_expected[OF_DIV_FLOW_IND];
    MAKE_MATRIX_PTR(I, innovation, N_MEAS_OF_KF);
    DEBUG_PRINT("Expected div: %f, Real div: %f.\n", Z_expected[OF_DIV_FLOW_IND], ins_flow.divergence);

    // X_k1_k1 = X_k1_k + K_k1*(i_k1);
    float _KI[N_STATES_OF_KF][1];
    MAKE_MATRIX_PTR(KI, _KI, N_STATES_OF_KF);
    float_mat_mul(KI, K, I, N_STATES_OF_KF, N_MEAS_OF_KF, 1);

    DEBUG_PRINT("K*innovation:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, 1, KI);

    printf("PRE: v = %f, angle = %f\n", OF_X[OF_V_IND], OF_X[OF_ANGLE_IND]);
    for(int i = 0; i < N_STATES_OF_KF; i++) {
	OF_X[i] += KI[i][0];
    }
    printf("POST v: %f, angle = %f\n", OF_X[OF_V_IND], OF_X[OF_ANGLE_IND]);

    struct FloatEulers* eulers = stateGetNedToBodyEulers_f();
    printf("Angles (deg): ahrs = %f, ekf = %f.\n", (180.0f/M_PI)*eulers->phi, (180.0f/M_PI)*OF_X[OF_ANGLE_IND]);

    // P_k1_k1 = (eye(Nx) - K_k1*Hx)*P_k1_k*(eye(Nx) - K_k1*Hx)' + K_k1*R*K_k1'; % Joseph form of the covariance update equation
    float _KJac[N_STATES_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(KJac, _KJac, N_STATES_OF_KF);
    float_mat_mul(KJac, K, Jac, N_STATES_OF_KF, N_MEAS_OF_KF, N_STATES_OF_KF);
    float _eye[N_STATES_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(eye, _eye, N_STATES_OF_KF);
    float_mat_diagonal_scal(eye, 1.0, N_STATES_OF_KF);
    float _eKJac[N_STATES_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(eKJac, _eKJac, N_STATES_OF_KF);
    float_mat_diff(eKJac, eye, KJac, N_STATES_OF_KF, N_STATES_OF_KF);
    float _eKJacT[N_STATES_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(eKJacT, _eKJacT, N_STATES_OF_KF);
    float_mat_transpose(eKJacT, eKJac, N_STATES_OF_KF, N_STATES_OF_KF);
    // (eye(Nx) - K_k1*Hx)*P_k1_k*(eye(Nx) - K_k1*Hx)'
    float_mat_mul(P, P, eKJacT, N_STATES_OF_KF, N_STATES_OF_KF, N_STATES_OF_KF);
    float_mat_mul(P, eKJac, P, N_STATES_OF_KF, N_STATES_OF_KF, N_STATES_OF_KF);

    // K_k1*R*K_k1'
    // TODO: check all MAKE_MATRIX that they mention the number of ROWS!
    float _KT[N_MEAS_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(KT, _KT, N_MEAS_OF_KF);
    float_mat_transpose(KT, K, N_STATES_OF_KF, N_MEAS_OF_KF);
    float _RKT[N_MEAS_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(RKT, _RKT, N_MEAS_OF_KF);
    float_mat_mul(RKT, R, KT, N_MEAS_OF_KF, N_MEAS_OF_KF, N_STATES_OF_KF);
    float _KRKT[N_STATES_OF_KF][N_STATES_OF_KF];
    MAKE_MATRIX_PTR(KRKT, _KRKT, N_STATES_OF_KF);
    float_mat_mul(KRKT, K, RKT, N_STATES_OF_KF, N_MEAS_OF_KF, N_STATES_OF_KF);

    // summing the two parts:
    float_mat_sum(P, P, KRKT, N_STATES_OF_KF, N_STATES_OF_KF);

    DEBUG_PRINT("P corrected:\n");
    DEBUG_MAT_PRINT(N_STATES_OF_KF, N_STATES_OF_KF, P);

    // indicate that the measurement has been used:
    ins_flow.new_flow_measurement = false;
  }

  // update the time:
  of_prev_time = of_time;
}

static void gyro_cb(uint8_t __attribute__((unused)) sender_id,
                    uint32_t stamp, struct Int32Rates *gyro)
{
  ahrs_icq_last_stamp = stamp;
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_PROPAGATE_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS_ICQ propagation.")
  /* timestamp in usec when last callback was received */
  static uint32_t last_stamp = 0;

  if (last_stamp > 0 && ahrs_icq.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_icq_propagate(gyro, dt);
    set_body_state_from_quat();
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_PROPAGATE_FREQUENCY for AHRS_ICQ propagation.")
  PRINT_CONFIG_VAR(AHRS_PROPAGATE_FREQUENCY)
  if (ahrs_icq.status == AHRS_ICQ_RUNNING) {
    const float dt = 1. / (AHRS_PROPAGATE_FREQUENCY);
    ahrs_icq_propagate(gyro, dt);
    set_body_state_from_quat();
  }
#endif
}

static void accel_cb(uint8_t __attribute__((unused)) sender_id,
                     uint32_t __attribute__((unused)) stamp,
                     struct Int32Vect3 *accel)
{
#if USE_AUTO_AHRS_FREQ || !defined(AHRS_CORRECT_FREQUENCY)
  PRINT_CONFIG_MSG("Calculating dt for AHRS int_cmpl_quat accel update.")
  static uint32_t last_stamp = 0;
  if (last_stamp > 0 && ahrs_icq.is_aligned) {
    float dt = (float)(stamp - last_stamp) * 1e-6;
    ahrs_icq_update_accel(accel, dt);
    set_body_state_from_quat();
  }
  last_stamp = stamp;
#else
  PRINT_CONFIG_MSG("Using fixed AHRS_CORRECT_FREQUENCY for AHRS int_cmpl_quat accel update.")
  PRINT_CONFIG_VAR(AHRS_CORRECT_FREQUENCY)
  if (ahrs_icq.is_aligned) {
    const float dt = 1. / (AHRS_CORRECT_FREQUENCY);
    ahrs_icq_update_accel(accel, dt);
    set_body_state_from_quat();
  }
#endif
}

/** Rotate angles and rates from imu to body frame and set state */
static void set_body_state_from_quat(void)
{
  /* Compute LTP to BODY quaternion */
  struct Int32Quat ltp_to_body_quat;
  struct Int32Quat *body_to_imu_quat = orientationGetQuat_i(&ahrs_icq.body_to_imu);
  int32_quat_comp_inv(&ltp_to_body_quat, &ahrs_icq.ltp_to_imu_quat, body_to_imu_quat);
  /* Set state */
  stateSetNedToBodyQuat_i(&ltp_to_body_quat);

  /* compute body rates */
  struct Int32Rates body_rate;
  struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&ahrs_icq.body_to_imu);
  int32_rmat_transp_ratemult(&body_rate, body_to_imu_rmat, &ahrs_icq.imu_rate);
  /* Set state */
  stateSetBodyRates_i(&body_rate);
}

static void ins_rpm_cb(uint8_t sender_id, uint16_t * rpm, uint8_t num_act)
{
  ins_flow.RPM_num_act = num_act;
  for(int i = 0; i < num_act; i++) {
      ins_flow.RPM[i] = rpm[i];
  }
}


/* Update INS based on GPS information */
static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp,
                   struct GpsState *gps_s)
{

  if (gps_s->fix < GPS_FIX_3D) {
    return;
  }
  if (!ins_flow.ltp_initialized) {
    ins_reset_local_origin();
  }

  /* simply scale and copy pos/speed from gps */
  struct NedCoor_i gps_pos_cm_ned;
  ned_of_ecef_point_i(&gps_pos_cm_ned, &ins_flow.ltp_def, &gps_s->ecef_pos);
  INT32_VECT3_SCALE_2(ins_flow.ltp_pos, gps_pos_cm_ned,
                      INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
  stateSetPositionNed_i(&ins_flow.ltp_pos);

  struct NedCoor_i gps_speed_cm_s_ned;
  ned_of_ecef_vect_i(&gps_speed_cm_s_ned, &ins_flow.ltp_def, &gps_s->ecef_vel);
  INT32_VECT3_SCALE_2(ins_flow.ltp_speed, gps_speed_cm_s_ned,
                      INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);
  stateSetSpeedNed_i(&ins_flow.ltp_speed);

  /*
  bool vel_ned_valid = bit_is_set(gps_s->valid_fields, GPS_VALID_VEL_NED_BIT);
  uint8_t nsats = gps_s->num_sv;
  */
}

/* Save the Body to IMU information */
static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{
  ahrs_icq_set_body_to_imu_quat(q_b2i_f);
}