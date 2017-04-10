/*
 * Copyright (C) 2015 Guido de Croon.
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ctrl/optical_flow_landing.h
 * @brief This module implements optical flow landings in which the divergence is kept constant.
 * When using a fixed gain for control, the covariance between thrust and divergence is tracked,
 * so that the drone knows when it has arrived close to the landing surface. Then, a final landing
 * procedure is triggered. It can also be set to adaptive gain control, where the goal is to continuously
 * gauge the distance to the landing surface. In this mode, the drone will oscillate all the way down to
 * the surface.
 *
 * de Croon, G.C.H.E. (2016). Monocular distance estimation with optical flow maneuvers and efference copies:
 * a stability-based strategy. Bioinspiration & biomimetics, 11(1), 016004.
 * <http://iopscience.iop.org/article/10.1088/1748-3190/11/1/016004>
 *
 */

// variables for in message:
float divergence;
float divergence_vision;
float divergence_vision_dt;
float normalized_thrust;
float cov_div;
float pstate;
float pused;
float istate;
float dstate;
float dt;
int vision_message_nr;
int previous_message_nr;
int landing;
float previous_err;
float previous_cov_err;

// for the exponentially decreasing gain:
int elc_phase;
long elc_time_start;
float elc_p_gain_start;
float elc_i_gain_start;
float elc_d_gain_start;


// minimum value of the P-gain for divergence control
// adaptive control will not be able to go lower
#define MINIMUM_GAIN 0.1

// used for automated landing:
#include "firmwares/rotorcraft/autopilot.h"
#include "subsystems/navigation/common_flight_plan.h"
#include "subsystems/datalink/telemetry.h"

// used for calculating velocity from height measurements:
#include <time.h>
long previous_time;
long module_enter_time;

// sending the divergence message to the ground station:
static void send_divergence(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_DIVERGENCE(trans, dev, AC_ID,
                           &divergence, &divergence_vision_dt, &normalized_thrust,
                           &cov_div, &pstate, &pused, &(of_landing_ctrl.agl));
}

#include "modules/ctrl/optical_flow_landing.h"
#include "generated/airframe.h"
#include "paparazzi.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/stabilization.h"

/* Default sonar/agl to use */
#ifndef OPTICAL_FLOW_LANDING_AGL_ID
#define OPTICAL_FLOW_LANDING_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(OPTICAL_FLOW_LANDING_AGL_ID)

/* Use optical flow estimates */
#ifndef OPTICAL_FLOW_LANDING_OPTICAL_FLOW_ID
#define OPTICAL_FLOW_LANDING_OPTICAL_FLOW_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(OPTICAL_FLOW_LANDING_OPTICAL_FLOW_ID)

// Other default values:
#ifndef OPTICAL_FLOW_LANDING_PGAIN
#define OPTICAL_FLOW_LANDING_PGAIN 0.50
#endif

#ifndef OPTICAL_FLOW_LANDING_IGAIN
#define OPTICAL_FLOW_LANDING_IGAIN 0.005
#endif

#ifndef OPTICAL_FLOW_LANDING_DGAIN
#define OPTICAL_FLOW_LANDING_DGAIN 0.50
#endif

#ifndef OPTICAL_FLOW_LANDING_VISION_METHOD
#define OPTICAL_FLOW_LANDING_VISION_METHOD 1
#endif

#ifndef OPTICAL_FLOW_LANDING_CONTROL_METHOD
#define OPTICAL_FLOW_LANDING_CONTROL_METHOD 2
#endif

#ifndef OPTICAL_FLOW_LANDING_COV_METHOD
#define OPTICAL_FLOW_LANDING_COV_METHOD 0
#endif

static abi_event agl_ev; ///< The altitude ABI event
static abi_event optical_flow_ev;

/// Callback function of the ground altitude
static void vertical_ctrl_agl_cb(uint8_t sender_id __attribute__((unused)), float distance);
// Callback function of the optical flow estimate:
static void vertical_ctrl_optical_flow_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, int16_t flow_x, int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, float quality, float size_divergence, float dist);

struct OpticalFlowLanding of_landing_ctrl;

void vertical_ctrl_module_init(void);
void vertical_ctrl_module_run(bool in_flight);

/**
 * Initialize the optical flow landing module
 */
void vertical_ctrl_module_init(void)
{
  unsigned int i;

  of_landing_ctrl.agl = 0.0f;
  of_landing_ctrl.agl_lp = 0.0f;
  of_landing_ctrl.vel = 0.0f;
  of_landing_ctrl.divergence_setpoint = 0.0f;
  of_landing_ctrl.cov_set_point = -0.025f;
  of_landing_ctrl.cov_limit = 0.005f; //1.0f; // for cov(uz,div)
  of_landing_ctrl.lp_factor = 0.95f;
  of_landing_ctrl.pgain = OPTICAL_FLOW_LANDING_PGAIN;
  of_landing_ctrl.igain = OPTICAL_FLOW_LANDING_IGAIN;
  of_landing_ctrl.dgain = OPTICAL_FLOW_LANDING_DGAIN;
  of_landing_ctrl.sum_err = 0.0f;
  of_landing_ctrl.d_err = 0.0f;
  of_landing_ctrl.nominal_thrust = 0.630f; // 0.710f; //0.666f; // 0.640 with small battery
  of_landing_ctrl.VISION_METHOD = OPTICAL_FLOW_LANDING_VISION_METHOD;
  of_landing_ctrl.CONTROL_METHOD = OPTICAL_FLOW_LANDING_CONTROL_METHOD;
  of_landing_ctrl.COV_METHOD = OPTICAL_FLOW_LANDING_COV_METHOD;
  of_landing_ctrl.delay_steps = 40;
  of_landing_ctrl.pgain_adaptive = 10.0;
  of_landing_ctrl.igain_adaptive = 0.25;
  of_landing_ctrl.dgain_adaptive = 0.00;
  of_landing_ctrl.reduction_factor_elc = 0.5f;

  struct timespec spec;
  clock_gettime(CLOCK_MONOTONIC, &spec);
  previous_time = spec.tv_sec * 1E3 + spec.tv_nsec / 1.0E6;
  module_enter_time = previous_time;
  //previous_time = time(NULL);

  // clear histories:
  ind_hist = 0;
  for (i = 0; i < COV_WINDOW_SIZE; i++) {
    thrust_history[i] = 0;
    divergence_history[i] = 0;
  }

  // reset errors, thrust, divergence, etc.:
  previous_err = 0.0f;
  previous_cov_err = 0.0f;
  normalized_thrust = 0.0f;
  divergence = 0.0f;
  divergence_vision = 0.0f;
  divergence_vision_dt = 0.0f;
  cov_div = 0.0f;
  dt = 0.0f;
  pstate = of_landing_ctrl.pgain;
  pused = pstate;
  istate = of_landing_ctrl.igain;
  dstate = of_landing_ctrl.dgain;
  vision_message_nr = 1;
  previous_message_nr = 0;
  of_landing_ctrl.agl_lp = 0.0f;
  landing = 0;

  // exponentially decreasing gain while landing:
  elc_phase = 0;
  elc_time_start = 0;

  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(OPTICAL_FLOW_LANDING_AGL_ID, &agl_ev, vertical_ctrl_agl_cb);
  // Subscribe to the optical flow estimator:
  AbiBindMsgOPTICAL_FLOW(OPTICAL_FLOW_LANDING_OPTICAL_FLOW_ID, &optical_flow_ev, vertical_ctrl_optical_flow_cb);

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DIVERGENCE, send_divergence);
}

/**
 * Reset all variables:
 */
void reset_all_vars()
{

  int i;
  of_landing_ctrl.sum_err = 0;
  of_landing_ctrl.d_err = 0;
  stabilization_cmd[COMMAND_THRUST] = 0;
  ind_hist = 0;
  of_landing_ctrl.agl_lp = 0;
  cov_div = of_landing_ctrl.cov_set_point;
  normalized_thrust = 0.0f;
  dt = 0.0f;
  previous_err = 0.0f;
  previous_cov_err = 0.0f;
  divergence = of_landing_ctrl.divergence_setpoint;
  struct timespec spec;
  clock_gettime(CLOCK_MONOTONIC, &spec);
  previous_time = spec.tv_sec * 1E3 + spec.tv_nsec / 1.0E6;
  vision_message_nr = 1;
  previous_message_nr = 0;
  for (i = 0; i < COV_WINDOW_SIZE; i++) {
    thrust_history[i] = 0;
    divergence_history[i] = 0;
  }
  landing = 0;
  elc_phase = 0;
}

/**
 * Run the optical flow landing module
 */

void vertical_ctrl_module_run(bool in_flight)
{
  int i;
  float lp_height; // low-pass height
  float div_factor; // factor that maps divergence in pixels as received from vision to /frame

  // ensure dt >= 0
  if (dt < 0) { dt = 0.0f; }

  // get delta time, dt, to scale the divergence measurements correctly when using "simulated" vision:
  struct timespec spec;
  clock_gettime(CLOCK_MONOTONIC, &spec);
  long new_time = spec.tv_sec * 1E3 + spec.tv_nsec / 1.0E6;
  long delta_t = new_time - previous_time;
  dt += ((float)delta_t) / 1000.0f;
  if (dt > 10.0f) {
    dt = 0.0f;
    return;
  }
  previous_time = new_time;
  long module_active_time = new_time - module_enter_time;
  float module_active_time_sec = (float) module_active_time / 1000.0f;


  if (!in_flight) {

    // When not flying and in mode module:
    // Reset integrators, landing phases, etc.
    reset_all_vars();

  } else {

    /***********
     * VISION
     ***********/

    if (of_landing_ctrl.VISION_METHOD == 0) {

      // SIMULATED DIVERGENCE:

      // USE OPTITRACK HEIGHT
      of_landing_ctrl.agl = (float) gps.lla_pos.alt / 1000.0f;
      // else we get an immediate jump in divergence when switching on.
      if (of_landing_ctrl.agl_lp < 1E-5 || ind_hist == 0) {
        of_landing_ctrl.agl_lp = of_landing_ctrl.agl;
      }
      if (fabs(of_landing_ctrl.agl - of_landing_ctrl.agl_lp) > 1.0f) {
        // ignore outliers:
        of_landing_ctrl.agl = of_landing_ctrl.agl_lp;
      }
      // calculate the new low-pass height and the velocity
      lp_height = of_landing_ctrl.agl_lp * of_landing_ctrl.lp_factor + of_landing_ctrl.agl * (1.0f - of_landing_ctrl.lp_factor);

      // only calculate velocity and divergence if dt is large enough:
      if (dt > 0.0001f) {
        of_landing_ctrl.vel = (lp_height - of_landing_ctrl.agl_lp) / dt;
        of_landing_ctrl.agl_lp = lp_height;

        // calculate the fake divergence:
        if (of_landing_ctrl.agl_lp > 0.0001f) {
          divergence = of_landing_ctrl.vel / of_landing_ctrl.agl_lp;
          divergence_vision_dt = (divergence_vision / dt);
          if (fabs(divergence_vision_dt) > 1E-5) {
            div_factor = divergence / divergence_vision_dt;
          }
        } else {
          divergence = 1000.0f;
          // perform no control with this value (keeping thrust the same)
          return;
        }
        // reset dt:
        dt = 0.0f;
      }
    } else {
      // USE REAL VISION OUTPUTS:

      if (vision_message_nr != previous_message_nr && dt > 1E-5 && ind_hist > 1) {
        // TODO: this div_factor depends on the subpixel-factor (automatically adapt?)
        div_factor = -1.28f; // magic number comprising field of view etc.
        float new_divergence = (divergence_vision * div_factor) / dt;

        if (fabs(new_divergence - divergence) > 0.20) {
          if (new_divergence < divergence) { new_divergence = divergence - 0.10f; }
          else { new_divergence = divergence + 0.10f; }
        }
        // low-pass filter the divergence:
        divergence = divergence * of_landing_ctrl.lp_factor + (new_divergence * (1.0f - of_landing_ctrl.lp_factor));
        previous_message_nr = vision_message_nr;
        dt = 0.0f;
      } else {
        // after re-entering the module, the divergence should be equal to the set point:
        if (ind_hist <= 1) {
          divergence = of_landing_ctrl.divergence_setpoint;
          for (i = 0; i < COV_WINDOW_SIZE; i++) {
            thrust_history[i] = 0;
            divergence_history[i] = 0;
          }
          ind_hist++;
          dt = 0.0f;
          int32_t nominal_throttle = of_landing_ctrl.nominal_thrust * MAX_PPRZ;
          stabilization_cmd[COMMAND_THRUST] = nominal_throttle;

        }
        // else: do nothing, let dt increment
        return;
      }
    }

    /***********
    * CONTROL
    ***********/

    int32_t nominal_throttle = of_landing_ctrl.nominal_thrust * MAX_PPRZ; \

    // landing indicates whether the drone is already performing a final landing procedure (flare):
    if (!landing) {

      if(module_active_time_sec < 2.5f) {
        // First seconds, don't do anything crazy:
        int32_t thrust = nominal_throttle;
        stabilization_cmd[COMMAND_THRUST] = thrust;
        return;
      } 

      if (of_landing_ctrl.CONTROL_METHOD == 0) {
        // fixed gain control, cov_limit for landing:

        // use the divergence for control:
        float err = of_landing_ctrl.divergence_setpoint - divergence;
        int32_t thrust = nominal_throttle + of_landing_ctrl.pgain * err * MAX_PPRZ + of_landing_ctrl.igain * of_landing_ctrl.sum_err * MAX_PPRZ + of_landing_ctrl.dgain * of_landing_ctrl.d_err * MAX_PPRZ;
        // make sure the p gain is logged:
        pstate = of_landing_ctrl.pgain;
        pused = pstate;
        // bound thrust:
        Bound(thrust, 0.8 * nominal_throttle, 0.75 * MAX_PPRZ);

        // histories and cov detection:
        normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));
        thrust_history[ind_hist % COV_WINDOW_SIZE] = normalized_thrust;
        divergence_history[ind_hist % COV_WINDOW_SIZE] = divergence;
        int ind_past = (ind_hist % COV_WINDOW_SIZE) - of_landing_ctrl.delay_steps;
        while (ind_past < 0) { ind_past += COV_WINDOW_SIZE; }
        float past_divergence = divergence_history[ind_past];
        past_divergence_history[ind_hist % COV_WINDOW_SIZE] = past_divergence;
        ind_hist++;
        // determine the covariance for landing detection:
        if (of_landing_ctrl.COV_METHOD == 0) {
          cov_div = get_cov(thrust_history, divergence_history, COV_WINDOW_SIZE);
        } else {
          cov_div = get_cov(past_divergence_history, divergence_history, COV_WINDOW_SIZE);
        }

        if (ind_hist >= COV_WINDOW_SIZE && fabs(cov_div) > of_landing_ctrl.cov_limit) {
          // land by setting 90% nominal thrust:
          landing = 1;
          thrust = 0.90 * nominal_throttle;
        }
        stabilization_cmd[COMMAND_THRUST] = thrust;
        of_landing_ctrl.sum_err += err;
        of_landing_ctrl.d_err = of_landing_ctrl.lp_factor * of_landing_ctrl.d_err + (1-of_landing_ctrl.lp_factor) * (err - previous_err) * 10.0f; // 10.0f to make it similarly sized to the error
        previous_err = err;
        // printf("d_err = %f, err = %f\n", of_landing_ctrl.d_err, err);
      } else if(of_landing_ctrl.CONTROL_METHOD == 1){
        // ADAPTIVE GAIN CONTROL:

        // adapt the gains according to the error in covariance:
        float error_cov = of_landing_ctrl.cov_set_point - cov_div;

        // limit the error_cov, which could else become very large:
        if (error_cov > fabs(of_landing_ctrl.cov_set_point)) { error_cov = fabs(of_landing_ctrl.cov_set_point); }
        pstate -= (of_landing_ctrl.igain_adaptive * pstate) * error_cov;
        if (pstate < MINIMUM_GAIN) { pstate = MINIMUM_GAIN; }

        // regulate the divergence:
        float err = of_landing_ctrl.divergence_setpoint - divergence;
        pused = pstate - (of_landing_ctrl.pgain_adaptive * pstate) * error_cov;

        // make sure pused does not become too small, nor grows too fast:
        if (pused < MINIMUM_GAIN) { pused = MINIMUM_GAIN; }
        if (of_landing_ctrl.COV_METHOD == 1 && error_cov > 0.001) {
          pused = 0.5 * pused;
        }

        // set thrust:
        int32_t thrust = nominal_throttle + pused * err * MAX_PPRZ + of_landing_ctrl.igain * of_landing_ctrl.sum_err * MAX_PPRZ;

        // histories and cov detection:
        normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));
        thrust_history[ind_hist % COV_WINDOW_SIZE] = normalized_thrust;
        divergence_history[ind_hist % COV_WINDOW_SIZE] = divergence;
        int ind_past = (ind_hist % COV_WINDOW_SIZE) - of_landing_ctrl.delay_steps;
        while (ind_past < 0) { ind_past += COV_WINDOW_SIZE; }
        float past_divergence = divergence_history[ind_past];
        past_divergence_history[ind_hist % COV_WINDOW_SIZE] = 100.0f * past_divergence;
        ind_hist++;

        // only take covariance into account if there are enough samples in the histories:
        if (ind_hist >= COV_WINDOW_SIZE) {
          if (of_landing_ctrl.COV_METHOD == 0) {
            cov_div = get_cov(thrust_history, divergence_history, COV_WINDOW_SIZE);
          } else {
            cov_div = get_cov(past_divergence_history, divergence_history, COV_WINDOW_SIZE);
          }
        } else {
          cov_div = of_landing_ctrl.cov_set_point;
        }

        // TODO: could put a landing condition here based on pstate (if too low)

        // bound thrust:
        Bound(thrust, 0.8 * nominal_throttle, 0.75 * MAX_PPRZ); // was 0.6 0.9
        stabilization_cmd[COMMAND_THRUST] = thrust;
        of_landing_ctrl.sum_err += err;
      }
      else if(of_landing_ctrl.CONTROL_METHOD == 2) {

        // Exponentially decaying gain strategy:
        if(elc_phase == 0) {
          // increase the gain till you start oscillating:
          float phase_0_set_point = 0.0f;
          // increase the p-gain:
          pstate *= 1.001f; //+= 0.001;
          pused = pstate;
          istate *= 1.001f;
          dstate *= 1.001f;
          // use the divergence for control:
          float err = phase_0_set_point - divergence;
          int32_t thrust = nominal_throttle + pused * err * MAX_PPRZ + istate * of_landing_ctrl.sum_err * MAX_PPRZ + dstate * of_landing_ctrl.d_err * MAX_PPRZ;;
          // bound thrust:
          Bound(thrust, 0.25 * nominal_throttle, 0.99 * MAX_PPRZ);
          // histories and cov detection:
          normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));
          thrust_history[ind_hist % COV_WINDOW_SIZE] = normalized_thrust;
          divergence_history[ind_hist % COV_WINDOW_SIZE] = divergence;
          int ind_past = (ind_hist % COV_WINDOW_SIZE) - of_landing_ctrl.delay_steps;
          while (ind_past < 0) { ind_past += COV_WINDOW_SIZE; }
          float past_divergence = divergence_history[ind_past];
          past_divergence_history[ind_hist % COV_WINDOW_SIZE] = past_divergence;
          ind_hist++;
          // determine the covariance for landing detection:
          if (of_landing_ctrl.COV_METHOD == 0) {
            cov_div = get_cov(thrust_history, divergence_history, COV_WINDOW_SIZE);
          } else {
            cov_div = get_cov(past_divergence_history, divergence_history, COV_WINDOW_SIZE);
          }
          // printf("ELC phase 0, gain = %f, cov_div = %f\n", pstate, cov_div);
          if (ind_hist >= COV_WINDOW_SIZE && module_active_time_sec > 10.0f && fabs(cov_div - of_landing_ctrl.cov_set_point) < of_landing_ctrl.cov_limit) {
            // next phase:
            elc_phase=1;
            clock_gettime(CLOCK_MONOTONIC, &spec);
            elc_time_start = spec.tv_sec * 1E3 + spec.tv_nsec / 1E6;
            // we don't want to oscillate, so reduce the gain:
            elc_p_gain_start = of_landing_ctrl.reduction_factor_elc * pstate;
            elc_i_gain_start = of_landing_ctrl.reduction_factor_elc * istate;
            elc_d_gain_start = of_landing_ctrl.reduction_factor_elc * dstate;
          }
          stabilization_cmd[COMMAND_THRUST] = thrust;
          of_landing_ctrl.sum_err += err;
          of_landing_ctrl.d_err = of_landing_ctrl.lp_factor * of_landing_ctrl.d_err + (1-of_landing_ctrl.lp_factor) * (err - previous_err) * 10.0f; // 10.0f to make it similarly sized to the error
          previous_err = err;
        }
        else if (elc_phase == 1) {
          // land while exponentially decreasing the gain:
          clock_gettime(CLOCK_MONOTONIC, &spec);
          new_time = spec.tv_sec * 1E3 + spec.tv_nsec / 1E6;
          float t_interval = (new_time - elc_time_start) / 1000.0f;
          // printf("start = %d, now = %d, time interval = %f\n", elc_time_start, new_time, t_interval);
          // this should not happen, but just to be sure to prevent too high gain values:
          if(t_interval < 0) t_interval = 0.0f;
          // determine the P-gain, exponentially decaying: 
          pstate = elc_p_gain_start*exp(of_landing_ctrl.divergence_setpoint*t_interval);
          istate = elc_i_gain_start*exp(of_landing_ctrl.divergence_setpoint*t_interval);
          dstate = elc_d_gain_start*exp(of_landing_ctrl.divergence_setpoint*t_interval);
          // use the divergence for control:
          float err = of_landing_ctrl.divergence_setpoint - divergence;
          int32_t thrust = nominal_throttle + pstate * err * MAX_PPRZ + istate * of_landing_ctrl.sum_err * MAX_PPRZ + dstate * of_landing_ctrl.d_err * MAX_PPRZ;;
          // make sure the p gain is logged:
          pused = pstate;
          // bound thrust:
          Bound(thrust, 0.25 * nominal_throttle, 0.99 * MAX_PPRZ);

          // printf("ELC phase 1, gain = %f\n", pstate);
          // histories and cov detection:
          normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));
          thrust_history[ind_hist % COV_WINDOW_SIZE] = normalized_thrust;
          divergence_history[ind_hist % COV_WINDOW_SIZE] = divergence;
          int ind_past = (ind_hist % COV_WINDOW_SIZE) - of_landing_ctrl.delay_steps;
          while (ind_past < 0) { ind_past += COV_WINDOW_SIZE; }
          float past_divergence = divergence_history[ind_past];
          past_divergence_history[ind_hist % COV_WINDOW_SIZE] = past_divergence;
          ind_hist++;
          // determine the covariance for landing detection:
          if (of_landing_ctrl.COV_METHOD == 0) {
            cov_div = get_cov(thrust_history, divergence_history, COV_WINDOW_SIZE);
          } else {
            cov_div = get_cov(past_divergence_history, divergence_history, COV_WINDOW_SIZE);
          }

          if (ind_hist >= COV_WINDOW_SIZE && fabs(cov_div) > of_landing_ctrl.cov_limit) {
            // nothing...
          }
          stabilization_cmd[COMMAND_THRUST] = thrust;
          of_landing_ctrl.sum_err += err;
          of_landing_ctrl.d_err = of_landing_ctrl.lp_factor * of_landing_ctrl.d_err + (1-of_landing_ctrl.lp_factor) * (err - previous_err) * 10.0f; // 10.0f to make it similarly sized to the error
          previous_err = err;
          float p_land_threshold = 0.1;
          if(pstate < p_land_threshold) {
            elc_phase = 2;
          }
        }
        else {
          // land with 90% nominal thrust:
          int32_t thrust = 0.90 * nominal_throttle;
          Bound(thrust, 0.6 * nominal_throttle, 0.9 * MAX_PPRZ);
          stabilization_cmd[COMMAND_THRUST] = thrust;          
        }
      }
    }
    else {
      // land with 90% nominal thrust:
      int32_t thrust = 0.90 * nominal_throttle;
      Bound(thrust, 0.6 * nominal_throttle, 0.9 * MAX_PPRZ);
      stabilization_cmd[COMMAND_THRUST] = thrust;
    }
  }
}

/**
 * Get the mean value of an array
 * @param[out] mean The mean value
 * @param[in] *a The array
 * @param[in] n Number of elements in the array
 */
float get_mean_array(float *a, int n_elements)
{
  // determine the mean for the vector:
  float mean = 0;
  for (unsigned int i = 0; i < n_elements; i++) {
    mean += a[i];
  }
  mean /= n_elements;

  return mean;
}

/**
 * Get the covariance of two arrays
 * @param[out] cov The covariance
 * @param[in] *a The first array
 * @param[in] *b The second array
 * @param[in] n Number of elements in the arrays
 */
float get_cov(float *a, float *b, int n_elements)
{
  // Determine means for each vector:
  float mean_a = get_mean_array(a, n_elements);
  float mean_b = get_mean_array(b, n_elements);

  // Determine the covariance:
  float cov = 0;
  for (unsigned int i = 0; i < n_elements; i++) {
    cov += (a[i] - mean_a) * (b[i] - mean_b);
  }

  cov /= n_elements;

  return cov;
}



// Reading from "sensors":
static void vertical_ctrl_agl_cb(uint8_t sender_id, float distance)
{
  of_landing_ctrl.agl = distance;
}
static void vertical_ctrl_optical_flow_cb(uint8_t sender_id, uint32_t stamp, int16_t flow_x, int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, float quality, float size_divergence, float dist)
{
  divergence_vision = size_divergence;
  vision_message_nr++;
  if (vision_message_nr > 10) { vision_message_nr = 0; }
}


////////////////////////////////////////////////////////////////////
// Call our controller
void guidance_v_module_init(void)
{
  vertical_ctrl_module_init();
}

/**
 * Entering the module (user switched to module)
 */
void guidance_v_module_enter(void)
{
  int i;
  // reset integrator
  of_landing_ctrl.sum_err = 0.0f;
  of_landing_ctrl.d_err = 0.0f;
  landing = 0;
  ind_hist = 0;
  previous_err = 0.0f;
  previous_cov_err = 0.0f;
  of_landing_ctrl.agl_lp = 0.0f;
  cov_div = of_landing_ctrl.cov_set_point;
  normalized_thrust = 0.0f;
  divergence = of_landing_ctrl.divergence_setpoint;
  dt = 0.0f;
  struct timespec spec;
  clock_gettime(CLOCK_MONOTONIC, &spec);
  previous_time = spec.tv_sec* 1E3 + spec.tv_nsec / 1.0E6;
  module_enter_time = previous_time;
  vision_message_nr = 1;
  previous_message_nr = 0;
  for (i = 0; i < COV_WINDOW_SIZE; i++) {
    thrust_history[i] = 0;
    divergence_history[i] = 0;
  }
  // Exponentially decreasing gain:
  elc_phase = 0;
  elc_time_start = 0;
  pstate = of_landing_ctrl.pgain;
  pused = pstate;
  istate = of_landing_ctrl.igain;
}

void guidance_v_module_run(bool in_flight)
{
  vertical_ctrl_module_run(in_flight);
}
