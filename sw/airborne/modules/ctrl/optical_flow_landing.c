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
 * Based on the above theory, we have also developed a new strategy for landing that consists of two phases:
 * (1) while hovering, the drone determines the optimal gain by increasing the gain until oscillation
 * (2) the drone starts landing while exponentially decreasing the gain over time
 *
 * This strategy leads to smooth, high-performance constant divergence landings, as explained in the article:
 * H.W. Ho, G.C.H.E. de Croon, E. van Kampen, Q.P. Chu, and M. Mulder (submitted)
 * Adaptive Control Strategy for Constant Optical Flow Divergence Landing,
 * <https://arxiv.org/abs/1609.06767>
 */


// variables retained between module calls
float divergence;
float divergence_vision;
float divergence_vision_dt;
float normalized_thrust;
float pused;
float istate;
float dstate;
float dt;
int vision_message_nr;
int previous_message_nr;
int landing;
float previous_err;
float previous_cov_err;

// for the exponentially decreasing gain strategy:
int elc_phase;
long elc_time_start;
float elc_p_gain_start;
float elc_i_gain_start;
float elc_d_gain_start;
long count_covdiv;
float lp_cov_div;

// minimum value of the P-gain for divergence control
// adaptive control / exponential gain control will not be able to go lower
#define MINIMUM_GAIN 0.1

// SSL: we will learn unstable gains, but need stable gains for landing
// this factor represents the trade-off between stability and performance
// 1.0 = unstable, 0.0 = no performance
#define STABLE_GAIN_FACTOR 0.5

// used for automated landing:
#include "autopilot.h"
#include "subsystems/navigation/common_flight_plan.h"
#include "subsystems/datalink/telemetry.h"

// for measuring time
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

// *********************************
// include and define stuff for SSL:
// *********************************
#define RECURSIVE_LEARNING 1
#include <stdio.h>
#include "modules/computer_vision/textons.h"
float* last_texton_distribution; // used to check if a new texton distribution has been received
// TODO: last_texton_distribution's size depends on the number of textons in textons.h/c.
// it should now be set to 10 to match the number of textons on the stereoboard... this is extremely ugly.
#define n_ts 10
float texton_distribution_stereoboard[n_ts];
// #define TEXTON_DISTRIBUTION_PATH /data/video/
// On Bebop 2:
#define TEXTON_DISTRIBUTION_PATH /data/ftp/internal000
// for RLS, recursive least squares:
float** P_RLS;
// forgetting factor:
float lambda;
static FILE *distribution_logger = NULL;
static FILE *weights_file = NULL;
unsigned int n_read_samples;
// paparazzi files for doing svd etc.:
#include "math/pprz_algebra_float.h"
#include "math/pprz_matrix_decomp_float.h"
#include "math/pprz_simple_matrix.h"


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
#define OPTICAL_FLOW_LANDING_PGAIN 0.40
#endif

#ifndef OPTICAL_FLOW_LANDING_IGAIN
#define OPTICAL_FLOW_LANDING_IGAIN 0.01
#endif

#ifndef OPTICAL_FLOW_LANDING_DGAIN
#define OPTICAL_FLOW_LANDING_DGAIN 0.0
#endif

#ifndef OPTICAL_FLOW_LANDING_VISION_METHOD
#define OPTICAL_FLOW_LANDING_VISION_METHOD 1
#endif

#ifndef OPTICAL_FLOW_LANDING_CONTROL_METHOD
#define OPTICAL_FLOW_LANDING_CONTROL_METHOD 1
#endif

#ifndef OPTICAL_FLOW_LANDING_COV_METHOD
#define OPTICAL_FLOW_LANDING_COV_METHOD 0
#endif

// number of time steps used for calculating the covariance (oscillations)
#ifndef OPTICAL_FLOW_LANDING_COV_WINDOW_SIZE
#define OPTICAL_FLOW_LANDING_COV_WINDOW_SIZE 30
#endif

static abi_event agl_ev; ///< The altitude ABI event
static abi_event optical_flow_ev;

/// Callback function of the ground altitude
static void vertical_ctrl_agl_cb(uint8_t sender_id __attribute__((unused)), float distance);
// Callback function of the optical flow estimate:
static void vertical_ctrl_optical_flow_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, int16_t flow_x,
    int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, float quality, float size_divergence, float dist);

// struct containing most relevant parameters
struct OpticalFlowLanding of_landing_ctrl;

void vertical_ctrl_module_init(void);
void vertical_ctrl_module_run(bool in_flight);

// for exponential gain landing, gain increase per second during the first (hover) phase:
#define INCREASE_GAIN_PER_SECOND 0.02

/**
 * Initialize the optical flow landing module
 */
void vertical_ctrl_module_init(void)
{
  unsigned int i;

  // filling the of_landing_ctrl struct with default values:
  of_landing_ctrl.agl = 0.0f;
  of_landing_ctrl.agl_lp = 0.0f;
  of_landing_ctrl.vel = 0.0f;
  of_landing_ctrl.divergence_setpoint = 0.0f; //-0.20f; // For exponential gain landing, pick a negative value
  of_landing_ctrl.cov_set_point = -0.0075f; // for cov(uz, div), i.e., cov_method 0
  of_landing_ctrl.cov_limit =
    2.2f; // This high a value means that for a constant divergence landing no landing will be triggered
  // If you want to trigger a landing, set the limit to something like 0.025f; // limit for cov(uz,div) - used only for landing triggering
  of_landing_ctrl.lp_factor = 0.75f; // for Bebop 2  // 0.60f; // for AR drone
  of_landing_ctrl.pgain = OPTICAL_FLOW_LANDING_PGAIN;
  of_landing_ctrl.igain = OPTICAL_FLOW_LANDING_IGAIN;
  of_landing_ctrl.dgain = OPTICAL_FLOW_LANDING_DGAIN;
  of_landing_ctrl.sum_err = 0.0f;
  of_landing_ctrl.d_err = 0.0f;
  of_landing_ctrl.nominal_thrust = 0.630f; // 0.710f; //0.666f; // 0.640 with small battery
  of_landing_ctrl.VISION_METHOD = OPTICAL_FLOW_LANDING_VISION_METHOD;
  of_landing_ctrl.CONTROL_METHOD = OPTICAL_FLOW_LANDING_CONTROL_METHOD;
  of_landing_ctrl.COV_METHOD = OPTICAL_FLOW_LANDING_COV_METHOD;
  of_landing_ctrl.delay_steps = 15;
  of_landing_ctrl.window_size = OPTICAL_FLOW_LANDING_COV_WINDOW_SIZE;
  of_landing_ctrl.pgain_adaptive = 10.0;
  of_landing_ctrl.igain_adaptive = 0.25;
  of_landing_ctrl.dgain_adaptive = 0.00;
  of_landing_ctrl.reduction_factor_elc =
    0.80f; // for exponential gain landing, after detecting oscillations, the gain is multiplied with this factor
  of_landing_ctrl.lp_cov_div_factor =
    0.99; // low pass filtering cov div so that the drone is really oscillating when triggering the descent
  of_landing_ctrl.count_transition =
    300; // tuned for Bebop 2 (higher frame rate than AR drone) - number of time steps the low-passed cov div should be beyond the limit
  of_landing_ctrl.p_land_threshold =
    0.15f; // if the gain reaches this value during an exponential landing, the drone makes the final landing.
  of_landing_ctrl.learn_gains = false;
  of_landing_ctrl.stable_gain_factor = STABLE_GAIN_FACTOR;
  of_landing_ctrl.load_weights = false;
  of_landing_ctrl.close_to_edge = 0.005;
  of_landing_ctrl.use_bias = true; // true for recursive estimation
  of_landing_ctrl.snapshot = false;

  struct timespec spec;
  clock_gettime(CLOCK_MONOTONIC, &spec);
  previous_time = spec.tv_sec * 1E3 + spec.tv_nsec / 1.0E6;
  module_enter_time = previous_time;

  // clear histories:
  ind_hist = 0;
  for (i = 0; i < MAX_COV_WINDOW_SIZE; i++) {
    thrust_history[i] = 0;
    divergence_history[i] = 0;
    dt_history[i] = 0;
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

  // variables for exponentially decreasing gain while landing:
  elc_phase = 0;
  elc_time_start = 0;
  count_covdiv = 0;
  lp_cov_div = 0.0f;

  // SSL:
  // TODO: not freed!
  last_texton_distribution = (float *)calloc(n_textons,sizeof(float));
  for(i = 0; i < n_textons; i++)
  {
    last_texton_distribution[i] = 0.0f;
  }
  // TODO: not freed!
  weights = (float *)calloc(n_textons+1,sizeof(float));
  for(i = 0; i <= n_textons; i++)
  {
    weights[i] = 0.0f;
  }
  // RLS:
  // TODO: not freed!
  P_RLS = (float **)calloc((n_textons+1),sizeof(float*));
  for(i = 0; i < n_textons+1; i++)
  {
    P_RLS[i] = (float *)calloc((n_textons+1),sizeof(float));
  }
  int j;
  for(i = 0; i < n_textons+1; i++)
  {
    for(j = 0; j < n_textons+1; j++)
    {
          if(i == j)
          {
            P_RLS[i][j] = 1.0f;
          }
          else
          {
            P_RLS[i][j] = 0.0f;
          }
    }
  }
  lambda = 0.999;

  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(OPTICAL_FLOW_LANDING_AGL_ID, &agl_ev, vertical_ctrl_agl_cb);
  // Subscribe to the optical flow estimator:
  AbiBindMsgOPTICAL_FLOW(OPTICAL_FLOW_LANDING_OPTICAL_FLOW_ID, &optical_flow_ev, vertical_ctrl_optical_flow_cb);
  // register telemetry:
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
  of_landing_ctrl.agl_lp = 0;
  cov_div = 0.0f; // of_landing_ctrl.cov_set_point;
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
  ind_hist = 0;
  for (i = 0; i < MAX_COV_WINDOW_SIZE; i++) {
    thrust_history[i] = 0;
    divergence_history[i] = 0;
    dt_history[i] = 0;
  }
  landing = 0;
  elc_phase = 0;
  count_covdiv = 0;
  lp_cov_div = 0.0f;

  // SSL:
  for(i = 0; i < n_textons; i++)
  {
    last_texton_distribution[i] = 0.0f;
  }
}

/**
 * Run the optical flow landing module
 */

void vertical_ctrl_module_run(bool in_flight)
{
  int i;
  float lp_height; // low-pass height
  float div_factor; // factor that maps divergence in pixels as received from vision to 1 / frame

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
  dt_history[ind_hist % of_landing_ctrl.window_size] = dt;
  ind_hist++;

  if (!in_flight) {

    // When not flying and in mode module:
    // Reset integrators, landing phases, etc.
    // reset_all_vars(); // commented out to allow us to study the observation variables in-hand, i.e., without flying

    // SSL: only learn if not flying - due to use of resources:
    if(of_landing_ctrl.learn_gains) {
      printf("STARTING LEARNING!\n");
      // learn the weights from the file filled with training examples:
      learn_from_file();
      // reset the learn_gains variable to false:
      of_landing_ctrl.learn_gains = false;
    }
    if(of_landing_ctrl.load_weights) {
      load_weights();
      of_landing_ctrl.load_weights = false;
    }
  }

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
    lp_height = of_landing_ctrl.agl_lp * of_landing_ctrl.lp_factor + of_landing_ctrl.agl *
                (1.0f - of_landing_ctrl.lp_factor);

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

      // deal with (unlikely) fast changes in divergence:
      float max_div_dt = 0.20;
      if (fabs(new_divergence - divergence) > max_div_dt) {
        if (new_divergence < divergence) { new_divergence = divergence - max_div_dt; }
        else { new_divergence = divergence + max_div_dt; }
      }

      // low-pass filter the divergence:
      divergence = divergence * of_landing_ctrl.lp_factor + (new_divergence * (1.0f - of_landing_ctrl.lp_factor));
      previous_message_nr = vision_message_nr;
      dt = 0.0f;

    } else {
      // after re-entering the module, the divergence should be equal to the set point:
      if (ind_hist <= 1) {
        divergence = of_landing_ctrl.divergence_setpoint;
        for (i = 0; i < MAX_COV_WINDOW_SIZE; i++) {
          thrust_history[i] = 0;
          divergence_history[i] = 0;
          dt_history[i] = 0;
        }
        // TODO: is this correct? Shouldn't dt be incremented?
        dt = 0.0f;
        int32_t nominal_throttle = of_landing_ctrl.nominal_thrust * MAX_PPRZ;
        stabilization_cmd[COMMAND_THRUST] = nominal_throttle;
      }
      // else: do nothing, let dt increment
      return;
    }
  }

  if (in_flight) {

    /***********
    * CONTROL
    ***********/

    float err;
    int32_t thrust;

    // landing indicates whether the drone is already performing a final landing procedure (flare):
    if (!landing) {

      // First seconds, don't do anything crazy:
      if (module_active_time_sec < 2.5f) {
        int32_t nominal_throttle = of_landing_ctrl.nominal_thrust * MAX_PPRZ;
        thrust = nominal_throttle;
        stabilization_cmd[COMMAND_THRUST] = thrust;
        return;
      }

      if (of_landing_ctrl.CONTROL_METHOD == 0) {

        // FIXED GAIN CONTROL, cov_limit for landing:

        // make sure the p gain is logged:
        pstate = of_landing_ctrl.pgain;
        pused = pstate;
        // use the divergence for control:
        thrust = PID_divergence_control(of_landing_ctrl.divergence_setpoint, of_landing_ctrl.pgain, of_landing_ctrl.igain,
                                        of_landing_ctrl.dgain, &err);
        // keep track of histories and set the covariance
        set_cov_div(thrust);
        // update the controller errors:
        update_errors(err);
        // trigger the landing if the cov div is too high:
        if (ind_hist >= of_landing_ctrl.window_size && fabs(cov_div) > of_landing_ctrl.cov_limit) {
          final_landing_procedure();
        }
      } else if (of_landing_ctrl.CONTROL_METHOD == 1) {

        // ADAPTIVE GAIN CONTROL:
        // TODO: i-gain and d-gain are currently not adapted

        // adapt the gains according to the error in covariance:
        float error_cov = of_landing_ctrl.cov_set_point - cov_div;
        // limit the error_cov, which could else become very large:
        if (error_cov > fabs(of_landing_ctrl.cov_set_point)) { error_cov = fabs(of_landing_ctrl.cov_set_point); }
        pstate -= (of_landing_ctrl.igain_adaptive * pstate) * error_cov;
        if (pstate < MINIMUM_GAIN) { pstate = MINIMUM_GAIN; }
        pused = pstate - (of_landing_ctrl.pgain_adaptive * pstate) * error_cov;
        // make sure pused does not become too small, nor grows too fast:
        if (pused < MINIMUM_GAIN) { pused = MINIMUM_GAIN; }
        if (of_landing_ctrl.COV_METHOD == 1 && error_cov > 0.001) {
          pused = 0.5 * pused;
        }

        // use the divergence for control:
        thrust = PID_divergence_control(of_landing_ctrl.divergence_setpoint, pused, of_landing_ctrl.igain,
                                        of_landing_ctrl.dgain, &err);
        // keep track of histories and set the covariance
        if (ind_hist >= of_landing_ctrl.window_size) {
          set_cov_div(thrust);
        } else {
          cov_div = of_landing_ctrl.cov_set_point;
        }

        // SSL: if close enough, store texton inputs:
        if(ind_hist >= of_landing_ctrl.window_size && fabs(error_cov) < of_landing_ctrl.close_to_edge) {
          save_texton_distribution();
        }

        // update the controller errors:
        update_errors(err);

        // TODO: could put a landing condition here based on pstate (if too low) - for when the desired divergence is negative

      } else if (of_landing_ctrl.CONTROL_METHOD == 2) {

        // EXPONENTIAL GAIN CONTROL:

        float phase_0_set_point = 0.0f;
        if (elc_phase == 0) {
          // increase the gain till you start oscillating:

          // if not yet oscillating, increase the gains:
          if (cov_div > of_landing_ctrl.cov_set_point) {
            float time_factor;
            if (ind_hist >= 1) {
              time_factor = dt_history[(ind_hist - 1) % of_landing_ctrl.window_size];
            } else {
              time_factor = 0.0f;
            }
            pstate += time_factor * INCREASE_GAIN_PER_SECOND;
            float gain_factor = pstate / pused;
            istate *= gain_factor;
            dstate *= gain_factor;
            pused = pstate;
          }

          // use the divergence for control:
          thrust = PID_divergence_control(phase_0_set_point, pused, istate, dstate, &err);
          // keep track of histories and set the covariance
          set_cov_div(thrust);
          // update the controller errors:
          update_errors(err);

          // low pass filter cov div and remove outliers:
          if (abs(lp_cov_div - cov_div) < 0.025) { // constant tuned for cov method 0
            lp_cov_div = of_landing_ctrl.lp_cov_div_factor * lp_cov_div + (1 - of_landing_ctrl.lp_cov_div_factor) * cov_div;
          }
          // if oscillating, maintain a counter to see if it endures:
          if (lp_cov_div <= of_landing_ctrl.cov_set_point) {
            count_covdiv++;
          } else {
            count_covdiv = 0;
          }
          // if the drone has been oscillating long enough, start landing:
          if (ind_hist >= of_landing_ctrl.window_size && count_covdiv > of_landing_ctrl.count_transition) {
            // next phase:
            elc_phase = 1;
            clock_gettime(CLOCK_MONOTONIC, &spec);
            elc_time_start = spec.tv_sec * 1E3 + spec.tv_nsec / 1E6;

            // we don't want to oscillate, so reduce the gain:
            elc_p_gain_start = of_landing_ctrl.reduction_factor_elc * pstate;
            elc_i_gain_start = of_landing_ctrl.reduction_factor_elc * istate;
            elc_d_gain_start = of_landing_ctrl.reduction_factor_elc * dstate;
            count_covdiv = 0;
            of_landing_ctrl.sum_err = 0.0f;
          }

        } else if (elc_phase == 1) {
          // control divergence to 0 with the reduced gain:
          pstate = elc_p_gain_start;
          pused = pstate;
          istate = elc_i_gain_start;
          dstate = elc_d_gain_start;

          clock_gettime(CLOCK_MONOTONIC, &spec);
          new_time = spec.tv_sec * 1E3 + spec.tv_nsec / 1E6;
          float t_interval = (new_time - elc_time_start) / 1000.0f;
          // printf("start = %d, now = %d, time interval = %f\n", elc_time_start, new_time, t_interval);
          // this should not happen, but just to be sure to prevent too high gain values:
          if (t_interval < 0) { t_interval = 0.0f; }

          // use the divergence for control:
          thrust = PID_divergence_control(phase_0_set_point, pused, istate, dstate, &err);
          // keep track of histories and set the covariance
          set_cov_div(thrust);
          // update the controller errors:
          update_errors(err);

          // if we have been trying to hover stably again for 3 seconds and we move in the same way as the desired divergence, switch to landing:
          if (t_interval >= 3.0f && divergence * of_landing_ctrl.divergence_setpoint >= 0.0f) {
            // next phase:
            elc_phase = 2;
            clock_gettime(CLOCK_MONOTONIC, &spec);
            elc_time_start = spec.tv_sec * 1E3 + spec.tv_nsec / 1E6;
            count_covdiv = 0;
          }
        } else if (elc_phase == 2) {
          // land while exponentially decreasing the gain:
          clock_gettime(CLOCK_MONOTONIC, &spec);
          new_time = spec.tv_sec * 1E3 + spec.tv_nsec / 1E6;
          float t_interval = (new_time - elc_time_start) / 1000.0f;

          // this should not happen, but just to be sure to prevent too high gain values:
          if (t_interval < 0) { t_interval = 0.0f; }

          // determine the P-gain, exponentially decaying:
          float gain_scaling = exp(of_landing_ctrl.divergence_setpoint * t_interval);
          if (gain_scaling <= 1.0f) {
            pstate = elc_p_gain_start * gain_scaling;
            istate = elc_i_gain_start * gain_scaling;
            dstate = elc_d_gain_start * gain_scaling;
          }
          pused = pstate;

          // use the divergence for control:
          thrust = PID_divergence_control(of_landing_ctrl.divergence_setpoint, pused, istate, dstate, &err);
          // keep track of histories and set the covariance
          set_cov_div(thrust);
          // update the controller errors:
          update_errors(err);

          // when to make the final landing:
          if (pstate < of_landing_ctrl.p_land_threshold) {
            elc_phase = 3;
          }
        } else {
          final_landing_procedure();
        }
      }
      else  if (of_landing_ctrl.CONTROL_METHOD == 3) {

        // SSL LANDING: use learned weights for setting the gain on the way down:

        // adapt the p-gain with a low-pass filter to the gain predicted by image appearance:
        // TODO: lp_factor is now the same as used for the divergence. This may not be appropriate
        pstate = predict_gain(texton_distribution);
        float lp_factor_prediction = 0.95;
        of_landing_ctrl.pgain = lp_factor_prediction * of_landing_ctrl.pgain + (1.0f - lp_factor_prediction) * of_landing_ctrl.stable_gain_factor * pstate;
        pused = of_landing_ctrl.pgain;
        // make sure pused does not become too small, nor grows too fast:
        if (of_landing_ctrl.pgain < MINIMUM_GAIN) { of_landing_ctrl.pgain = MINIMUM_GAIN; }
        // have the i and d gain depend on the p gain:
        istate = 0.025 * of_landing_ctrl.pgain;
        dstate = 0.0f;
        printf("of_landing_ctrl.pgain = %f\n", of_landing_ctrl.pgain);

        // use the divergence for control:
        thrust = PID_divergence_control(of_landing_ctrl.divergence_setpoint, pused, istate, dstate, &err);
        // keep track of histories and set the covariance
        set_cov_div(thrust);
        // update the controller errors:
        update_errors(err);

      }
    } else {
      final_landing_procedure();
    }
  }
}

/**
 * Execute a final landing procedure
 */
void final_landing_procedure()
{
  // land with 90% nominal thrust:
  int32_t nominal_throttle = of_landing_ctrl.nominal_thrust * MAX_PPRZ;
  int32_t thrust = 0.90 * nominal_throttle;
  Bound(thrust, 0.6 * nominal_throttle, 0.9 * MAX_PPRZ);
  stabilization_cmd[COMMAND_THRUST] = thrust;
  landing = 1;
}

/**
 * Set the covariance of the divergence and the thrust / past divergence
 * @param[in] thrust: the current thrust value
 */
void set_cov_div(int32_t thrust)
{
  // histories and cov detection:
  normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));
  thrust_history[ind_hist % of_landing_ctrl.window_size] = normalized_thrust;
  divergence_history[ind_hist % of_landing_ctrl.window_size] = divergence;
  int ind_past = (ind_hist % of_landing_ctrl.window_size) - of_landing_ctrl.delay_steps;
  while (ind_past < 0) { ind_past += of_landing_ctrl.window_size; }
  float past_divergence = divergence_history[ind_past];
  past_divergence_history[ind_hist % of_landing_ctrl.window_size] = past_divergence;

  // determine the covariance for landing detection:
  // only take covariance into account if there are enough samples in the histories:
  if (of_landing_ctrl.COV_METHOD == 0) {
    cov_div = get_cov(thrust_history, divergence_history, of_landing_ctrl.window_size);
  } else {
    cov_div = get_cov(past_divergence_history, divergence_history, of_landing_ctrl.window_size);
    // printf("Time window in seconds: %f\n", get_mean_array(dt_history, of_landing_ctrl.window_size) * of_landing_ctrl.window_size);
  }

}

/**
 * Determine and set the thrust for constant divergence control
 * @param[out] thrust
 * @param[in] divergence_set_point: The desired divergence
 * @param[in] P: P-gain
 * @param[in] I: I-gain
 * @param[in] D: D-gain
 * @param[in] err*: the error of the observed divergence with respect to the set point
 */

int32_t PID_divergence_control(float divergence_setpoint, float P, float I, float D, float *err)
{
  // determine the error:
  (*err) = divergence_setpoint - divergence;

  // PID control:
  int32_t nominal_throttle = of_landing_ctrl.nominal_thrust * MAX_PPRZ;
  int32_t thrust = nominal_throttle + P * (*err) * MAX_PPRZ
                   + I * of_landing_ctrl.sum_err * MAX_PPRZ
                   + D * of_landing_ctrl.d_err * MAX_PPRZ;

  // bound thrust:
  Bound(thrust, 0.25 * nominal_throttle, 0.99 * MAX_PPRZ);

  // set the thrust:
  stabilization_cmd[COMMAND_THRUST] = thrust;
  return thrust;
}

/**
 * Updates the integral and differential errors for PID control and sets the previous error
 * @param[in] err: the error of the divergence and divergence setpoint
 */
void update_errors(float err)
{
  // maintain the controller errors:
  of_landing_ctrl.sum_err += err;
  of_landing_ctrl.d_err = of_landing_ctrl.lp_factor * of_landing_ctrl.d_err + (1 - of_landing_ctrl.lp_factor) *
                          (err - previous_err) * 10.0f; // 10.0f to make it similarly sized to the error
  previous_err = err;
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
static void vertical_ctrl_optical_flow_cb(uint8_t sender_id, uint32_t stamp, int16_t flow_x, int16_t flow_y,
    int16_t flow_der_x, int16_t flow_der_y, float quality, float size_divergence, float dist)
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
  cov_div = 0.0f; //of_landing_ctrl.cov_set_point;
  normalized_thrust = 0.0f;
  divergence = of_landing_ctrl.divergence_setpoint;
  dt = 0.0f;
  struct timespec spec;
  clock_gettime(CLOCK_MONOTONIC, &spec);
  previous_time = spec.tv_sec * 1E3 + spec.tv_nsec / 1.0E6;
  module_enter_time = previous_time;
  vision_message_nr = 1;
  previous_message_nr = 0;
  for (i = 0; i < MAX_COV_WINDOW_SIZE; i++) {
    thrust_history[i] = 0;
    divergence_history[i] = 0;
  }
  // Exponentially decreasing gain:
  elc_phase = 0;
  elc_time_start = 0;
  count_covdiv = 0;
  lp_cov_div = 0.0f;
  pstate = of_landing_ctrl.pgain;
  pused = pstate;
  istate = of_landing_ctrl.igain;

  // adaptive estimation - assumes hover condition when entering the module:
  of_landing_ctrl.nominal_thrust = (float) stabilization_cmd[COMMAND_THRUST] / MAX_PPRZ;
}

void guidance_v_module_run(bool in_flight)
{
  vertical_ctrl_module_run(in_flight);
}

// SSL:
void save_texton_distribution(void)
{
  // Since the control module runs faster than the texton vision process, we need to check that we are storing a recent vision result:
  int i, same;
  same = 1;
  for(i = 0; i < n_textons; i++)
  {
    // check if the texton distribution is the same as the previous one:
    if(texton_distribution[i] != last_texton_distribution[i])
    {
      same = 0;
    }
    // update the last texton distribution:
    last_texton_distribution[i] = texton_distribution[i];
  }

  // don't save the texton distribution if it is the same as previous time step:
  /*if(same)
  {
    printf("Same\n");
    return;
  }*/

  // TODO: if(!same)?
  // If not the same, append the target values (heights, gains) and texton values to a .dat file:
  char filename[512];
        sprintf(filename, "%s/Training_set_%05d.dat", STRINGIFY(TEXTON_DISTRIBUTION_PATH), 0);
  distribution_logger = fopen(filename, "a");
        if(distribution_logger == NULL)
        {
    perror(filename);
                //perror("Error while opening the file.\n");
        }
        else
        {
    printf("Logging at height %f, gain %f, cov_div %f\n", of_landing_ctrl.agl, pstate, cov_div);

    // save the information in a single row:
    fprintf(distribution_logger, "%f ", of_landing_ctrl.agl); // sonar measurement
    fprintf(distribution_logger, "%f ", pstate); // gain measurement
    fprintf(distribution_logger, "%f ", cov_div); // cov div measurement
    for(i = 0; i < n_textons-1; i++)
    {
      fprintf(distribution_logger, "%f ", texton_distribution[i]);
    }
    fprintf(distribution_logger, "%f\n", texton_distribution[n_textons-1]);
    fclose(distribution_logger);
  }
}

void load_texton_distribution(void)
{
  int i, j, read_result;
  char filename[512];
        sprintf(filename, "%s/Training_set_%05d.dat", STRINGIFY(TEXTON_DISTRIBUTION_PATH), 0);

        if((distribution_logger = fopen(filename, "r")))
        {
    // Load the dictionary:
    n_read_samples = 0;
    // For now we read the samples sequentially:
    for(i = 0; i < MAX_SAMPLES_LEARNING; i++)
    {
      read_result = fscanf(distribution_logger, "%f ", &sonar[n_read_samples]);
                        if(read_result == EOF) break;
      if(i % 100) printf("SONAR: %f\n", sonar[n_read_samples]);
      read_result = fscanf(distribution_logger, "%f ", &gains[n_read_samples]);
                        if(read_result == EOF) break;
      read_result = fscanf(distribution_logger, "%f ", &cov_divs_log[n_read_samples]);
                        if(read_result == EOF) break;

      text_dists[n_read_samples] = (float*) calloc(n_textons,sizeof(float));

      for(j = 0; j < n_textons-1; j++)
      {
        read_result = fscanf(distribution_logger, "%f ", &text_dists[n_read_samples][j]);
                        if(read_result == EOF) break;
      }
      read_result = fscanf(distribution_logger, "%f\n", &text_dists[n_read_samples][n_textons-1]);
                        if(read_result == EOF) break;
      n_read_samples++;
    }
                fclose(distribution_logger);

    printf("Learned samples = %d\n", n_read_samples);
  }
}

void learn_from_file(void)
{
  int i;
  float fit_error;

  printf("LOAD FILE\n");
  // first load the texton distributions:
  load_texton_distribution();

  // then learn from it:
  // TODO: uncomment & comment to learn gains instead of sonar:
  printf("FIT MODEL\n");
  if(!RECURSIVE_LEARNING)
  {
    fit_linear_model(gains, text_dists, n_textons, n_read_samples, weights, &fit_error);
    // fit_linear_model(sonar, text_dists, n_textons, n_read_samples, weights, &fit_error);
  }
  else
  {
    printf("RECURSIVE!\n");
    recursive_least_squares_batch(gains, text_dists, n_textons, n_read_samples, weights, &fit_error);
  }

  printf("SAVE WEIGHTS\n");
  // save the weights to a file:
  save_weights();

  printf("Learned! Fit error = %f\n", fit_error);

  // free learning distributions:
  for(i = 0; i < MAX_SAMPLES_LEARNING; i++)
  {
    free(text_dists[i]);
  }
}

/**
 * Recursively fit a linear model from samples to target values - batch mode, possibly for initialization.
 * @param[in] targets The target values
 * @param[in] samples The samples / feature vectors
 * @param[in] D The dimensionality of the samples
 * @param[in] count The number of samples
 * @param[out] parameters* Parameters of the linear fit
 * @param[out] fit_error* Total error of the fit // TODO: relevant for RLS?
 */
void recursive_least_squares_batch(float* targets, float** samples, uint8_t D, uint16_t count, float* params, float* fit_error)
{
  printf("START RECURSIVE!\n");
  // TODO: potentially randomizing the sequence of the samples, as not to get a bias towards the later samples
  // TODO: determine the error over the set. For now, we set the error to 0:
  (*fit_error) = 0.0f;
  // local vars for iterating, random numbers:
  int sam, d;
  uint8_t D_1 = D+1;
  float augmented_sample[D_1];
  // augmented sample is used if the bias is used:
  augmented_sample[D] = 1.0f;

  // Initialize the weights with 0.0f:
  for(d = 0; d < D_1; d++)
  {
    weights[d] = 0.0f;
  }

  // Reset the P matrix to an identity matrix:
  int i, j;
  for(i = 0; i < n_textons+1; i++)
  {
    for(j = 0; j < n_textons+1; j++)
    {
      if(i == j)
      {
        P_RLS[i][j] = 1.0f;
      }
      else
      {
        P_RLS[i][j] = 0.0f;
      }
    }
  }

  // give the samples one by one to the recursive procedure:
  for(sam = 0; sam < count; sam++)
  {
    if(!of_landing_ctrl.use_bias)
    {
      recursive_least_squares(targets[sam], samples[sam], D, params);
    }
    else
    {
      for(d = 0; d < D; d++)
      {
        augmented_sample[d] = samples[sam][d];
      }
      recursive_least_squares(targets[sam], augmented_sample, D_1, params);
    }
  }

  printf("READY!!\n");
}

static inline void float_mat_div_scalar(float **o, float **a, float scalar, int m, int n)
{
   int i, j;
   for (i = 0; i < m; i++) {
     for (j = 0; j < n; j++) {
         o[i][j] = a[i][j] / scalar;
     }
   }
}

static inline void float_mat_mul_scalar(float **o, float **a, float scalar, int m, int n)
{
   int i, j;
   for (i = 0; i < m; i++) {
     for (j = 0; j < n; j++) {
         o[i][j] = a[i][j] * scalar;
     }
   }
}


void recursive_least_squares(float target, float* sample, uint8_t length_sample, float* params)
{
  // MATLAB procedure:
  /*
    u = features(i,:);
    phi = u * P ;
          k(:, i) = phi' / (lamda + phi * u' );
    y(i)= weights(:,i)' * u';
    e(i) = targets(i) - y(i) ;
          weights(:,i+1) = weights(:,i) + k(:, i) * e(i) ;
          P = ( P - k(:, i) * phi ) / lamda;
  */

  float _u[1][length_sample];
  float _uT[length_sample][1];
  float _phi[1][length_sample];
  float _phiT[length_sample][1];
  float _k[length_sample][1];
  float _ke[length_sample][1];
  float prediction, error;
  float _u_P_uT[1][1];
  float scalar;
  float _O[length_sample][length_sample];
  int i;

  // u = features(i,:);
  for(i=0; i < length_sample; i++)
  {
    _u[0][i] = sample[i];
    _uT[i][0] = sample[i];
  }
  MAKE_MATRIX_PTR(u, _u, 1);
  MAKE_MATRIX_PTR(uT, _uT, length_sample); // transpose
  MAKE_MATRIX_PTR(phi, _phi, 1);
  MAKE_MATRIX_PTR(phiT, _phiT, length_sample);
  MAKE_MATRIX_PTR(u_P_uT, _u_P_uT, 1); // actually a scalar
  MAKE_MATRIX_PTR(k, _k, length_sample);
  MAKE_MATRIX_PTR(ke, _ke, length_sample);

  // TODO: does this go well for bias / no bias? Because P_RLS is created with (n_textons+1) x (n_textons+1)
  // I think yes, as long as P_RLS is initialized with n_textons+1 rows and columns.
  MAKE_MATRIX_PTR(P, P_RLS, length_sample);
  MAKE_MATRIX_PTR(O, _O, length_sample);
  // phi = u * P ;
  // result of multiplication goes into phi
  MAT_MUL(1, length_sample, length_sample, phi, u, P);
  // k(:, i) = phi' / (lamda + phi * u' );
  for(i = 0; i < length_sample; i++)
  {
    phiT[i][0] = phi[0][i];
  }
  // scalar:
  MAT_MUL(1, length_sample, 1, u_P_uT, phi, uT);
  scalar = u_P_uT[0][0];
  float_mat_div_scalar(k, phiT, lambda+scalar, length_sample, 1);
  // y(i)= weights(:,i)' * u';
  prediction = predict_gain(sample);
  // e(i) = targets(i) - y(i) ;
  error = target - prediction;
  // weights(:,i+1) = weights(:,i) + k(:, i) * e(i) ;
  float_mat_mul_scalar(ke, k, error, length_sample, 1);
  for(i = 0; i < length_sample; i++)
  {
    weights[i] += ke[i][0];
  }
  // P = ( P - k(:, i) * phi ) / lamda;
  MAT_MUL(length_sample, 1, length_sample, O, k, phi);
  MAT_SUB(length_sample, length_sample, P, P, O);
  float_mat_div_scalar(P, P, lambda, length_sample, length_sample);
}

/**
 * Fit a linear model from samples to target values.
 * @param[in] targets The target values
 * @param[in] samples The samples / feature vectors
 * @param[in] D The dimensionality of the samples
 * @param[in] count The number of samples
 * @param[out] parameters* Parameters of the linear fit
 * @param[out] fit_error* Total error of the fit
 */
void fit_linear_model(float* targets, float** samples, uint8_t D, uint16_t count, float* params, float* fit_error)
{

  // We will solve systems of the form A x = b,
  // where A = [nx(D+1)] matrix with entries [s1, ..., sD, 1] for each sample (1 is the bias)
  // and b = [nx1] vector with the target values.
  // x in the system are the parameters for the linear regression function.

  // local vars for iterating, random numbers:
  int sam, d;
  uint16_t n_samples = count;
  uint8_t D_1 = D+1;
  // ensure that n_samples is high enough to ensure a result for a single fit:
  n_samples = (n_samples < D_1) ? D_1 : n_samples;
  // n_samples should not be higher than count:
  n_samples = (n_samples < count) ? n_samples : count;

  // initialize matrices and vectors for the full point set problem:
  // this is used for determining inliers
  float _AA[count][D_1];
  MAKE_MATRIX_PTR(AA, _AA, count);
  float _targets_all[count][1];
  MAKE_MATRIX_PTR(targets_all, _targets_all, count);

  for (sam = 0; sam < count; sam++) {
     for(d = 0; d < D; d++) {
      AA[sam][d] = samples[sam][d];
    }
    if(of_landing_ctrl.use_bias) {
      AA[sam][D] = 1.0f;
    }
    else {
      AA[sam][D] = 0.0f;
    }
    targets_all[sam][0] = targets[sam];
  }


  // decompose A in u, w, v with singular value decomposition A = u * w * vT.
  // u replaces A as output:
  float _parameters[D_1][1];
  MAKE_MATRIX_PTR(parameters, _parameters, D_1);
  float w[n_samples], _v[D_1][D_1];
  MAKE_MATRIX_PTR(v, _v, D_1);

  pprz_svd_float(AA, w, v, count, D_1);
  pprz_svd_solve_float(parameters, AA, w, v, targets_all, count, D_1, 1);

  // used to determine the error of a set of parameters on the whole set:
  float _bb[count][1];
  MAKE_MATRIX_PTR(bb, _bb, count);
  float _C[count][1];
  MAKE_MATRIX_PTR(C, _C, count);

  // error is determined on the entire set
  // bb = AA * parameters:
  MAT_MUL(count, D_1, 1, bb, AA, parameters);
  // subtract bu_all: C = 0 in case of perfect fit:
  MAT_SUB(count, 1, C, bb, targets_all);
  *fit_error = 0;
  for (sam = 0; sam < count; sam++) {
    *fit_error += abs(C[sam][0]);
  }
  *fit_error /= count;

  for(d = 0; d < D_1; d++) {
      params[d] = parameters[d][0];
  }

}



// General TODO: what happens if n_textons changes?

float predict_gain(float* distribution)
{
  int i;
  float sum;

  /*
  // TODO: is this not slower than what our own implementation would do?

  uint8_t D_1 = n_textons+1;
  float _distr[D_1];
  float _pred[1][1];

  // make feature vector:
  MAKE_MATRIX_PTR(distr, _distr, D_1);
  for(i = 0; i < n_textons; i++) {
    distr[i] = distribution[i];
  }
  distr[n_textons] = 1.0f; // bias

  // make weight vector:
  MAKE_MATRIX_PTR(w, weights, D_1);
  // make resulting prediction:
  MAKE_MATRIX_PTR(pred, _pred, count);

  // multiply the vectors:
  MAT_MUL(1, D_1, 1, pred, distr, w);

  return pred[0][0];
  */

  sum = 0.0f;
  for(i = 0; i < n_textons; i++)
  {
    sum += weights[i] * distribution[i];
  }
  if(of_landing_ctrl.use_bias) {
    sum += weights[n_textons];
  }
  return sum;
}

void save_weights(void) {
  // save the weights to a file:
  int i;
  char filename[512];
        sprintf(filename, "%s/Weights_%05d.dat", STRINGIFY(TEXTON_DISTRIBUTION_PATH), 0);
  weights_file = fopen(filename, "w");
        if(weights_file == NULL)
        {
    perror(filename);
        }
        else
        {
    // save the information in a single row:
    for(i = 0; i <= n_textons; i++)
    {
      fprintf(weights_file, "%f ", weights[i]);
    }
    fclose(weights_file);
  }
}

void load_weights(void) {
  int i, read_result;
  char filename[512];
  sprintf(filename, "%s/Weights_%05d.dat", STRINGIFY(TEXTON_DISTRIBUTION_PATH), 0);
  weights_file = fopen(filename, "r");
        if(weights_file == NULL)
        {
    printf("No weights file!\n");
    perror(filename);
        }
        else
        {
    // load the weights, stored in a single row:
    for(i = 0; i <= n_textons; i++)
    {
      read_result = fscanf(weights_file, "%f ", &weights[i]);
                        if(read_result == EOF) break;
    }
    fclose(weights_file);
  }
}
