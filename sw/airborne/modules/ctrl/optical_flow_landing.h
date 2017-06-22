/*
 * Copyright (C) 2016
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
 * @file modules/ctrl/optical_flow_landing.c
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

#ifndef OPTICAL_FLOW_LANDING_H_
#define OPTICAL_FLOW_LANDING_H_

#include "std.h"

#define MAX_COV_WINDOW_SIZE 100

// maximum number of samples to learn from for SSL:
#define MAX_SAMPLES_LEARNING 25000

struct OpticalFlowLanding {
  float agl;                    ///< agl = height from sonar (only used when using "fake" divergence)
  float agl_lp;                 ///< low-pass version of agl
  float lp_factor;              ///< low-pass factor in [0,1], with 0 purely using the current measurement
  float vel;                    ///< vertical velocity as determined with sonar (only used when using "fake" divergence)
  float divergence_setpoint;    ///< setpoint for constant divergence approach
  float pgain;                  ///< P-gain for constant divergence control (from divergence error to thrust)
  float igain;                  ///< I-gain for constant divergence control
  float dgain;                  ///< D-gain for constant divergence control
  float sum_err;                ///< integration of the error for I-gain
  float d_err;                  ///< difference of error for the D-gain
  float nominal_thrust;         ///< nominal thrust around which the PID-control operates
  int VISION_METHOD;            ///< whether to use vision (1) or Optitrack / sonar (0)
  int CONTROL_METHOD;           ///< type of divergence control: 0 = fixed gain, 1 = adaptive gain
  float cov_set_point;          ///< for adaptive gain control, setpoint of the covariance (oscillations)
  float cov_limit;              ///< for fixed gain control, what is the cov limit triggering the landing
  float pgain_adaptive;         ///< P-gain for adaptive gain control
  float igain_adaptive;         ///< I-gain for adaptive gain control
  float dgain_adaptive;         ///< D-gain for adaptive gain control
  int COV_METHOD;               ///< method to calculate the covariance: between thrust and div (0) or div and div past (1)
  int delay_steps;              ///< number of delay steps for div past
  int window_size;              ///< number of time steps in "window" used for getting the covariance
  float reduction_factor_elc;   ///< reduction factor - after oscillation, how much to reduce the gain...
  float lp_cov_div_factor;      ///< low-pass factor for the covariance of divergence in order to trigger the second landing phase in the exponential strategy.
  int count_transition;         ///< how many time steps the drone has to be oscillating in order to transition to the hover phase with reduced gain
  float p_land_threshold;       ///< if during the exponential landing the gain reaches this value, the final landing procedure is triggered
  // TODO: volatile bool?
  volatile bool learn_gains;    ///< set to true if the robot needs to learn a mapping from texton distributions to the p-gain
  // float stable_gain_factor;     ///< this factor is multiplied with the gain estimate from SSL, in interval [0,1]. If 1, the system will be unstable, if 0, there is no control (performance).
  bool load_weights;            ///< load the weights that were learned before
  float close_to_edge;          ///< if abs(cov_div - reference cov_div) < close_to_edge, then texton distributions and gains are stored for learning
  bool use_bias;                ///< if true, a bias 1.0f will be added to the feature vector (texton distribution) - this typically leads to very large weights
  bool snapshot;                ///< if true, besides storing a texton distribution, an image will also be stored (when unstable)
  float lp_factor_prediction;   ///< low-pass value for the predicted P-value
  float ramp_duration;          ///< ramp duration in seconds for when the divergence setpoint changes
};

extern struct OpticalFlowLanding of_landing_ctrl;

// arrays containing histories for determining covariance
float thrust_history[MAX_COV_WINDOW_SIZE];
float divergence_history[MAX_COV_WINDOW_SIZE];
float past_divergence_history[MAX_COV_WINDOW_SIZE];
float dt_history[MAX_COV_WINDOW_SIZE];
unsigned long ind_hist;
// SSL:
float *text_dists[MAX_SAMPLES_LEARNING];
float sonar[MAX_SAMPLES_LEARNING];
float gains[MAX_SAMPLES_LEARNING];
float cov_divs_log[MAX_SAMPLES_LEARNING];
float *weights;

// for "communication" with file logger:
float cov_div;
float pstate;

// Without optitrack set to: GUIDANCE_H_MODE_ATTITUDE
// With optitrack set to: GUIDANCE_H_MODE_HOVER / NAV
#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_NAV

// Own guidance_v
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

// supporting functions for cov calculation:
float get_cov(float *a, float *b, int n_elements);
float get_mean_array(float *a, int n_elements);

// common functions for different landing strategies:
void set_cov_div(int32_t thrust);
int32_t PID_divergence_control(float divergence_setpoint, float P, float I, float D, float *err);
void update_errors(float error);
void final_landing_procedure(void);

// resetting all variables to be called for instance when starting up / re-entering module
void reset_all_vars(void);

// Implement own Vertical loops
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool in_flight);

// SSL functions:
void save_texton_distribution(void);
void load_texton_distribution(void);
void fit_linear_model(float* targets, float** samples, uint8_t D, uint16_t count, float* parameters, float* fit_error);
void learn_from_file(void);
float predict_gain(float* distribution);
void save_weights(void);
void load_weights(void);
void recursive_least_squares_batch(float* targets, float** samples, uint8_t D, uint16_t count, float* params, float* fit_error);
void recursive_least_squares(float target, float* sample, uint8_t length_sample, float* params);

#endif /* OPTICAL_FLOW_LANDING_H_ */
