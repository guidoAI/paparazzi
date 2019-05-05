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
 * @file modules/ctrl/optical_flow_avoidance.c
 * @brief This module implements various optical flow avoidance methods.
 *
 * A first method uses the optical flow stability theory from [1] to keep the vertical flow constant
 * with the thrust while flying forwards. The drone stops when oscillations arise.
 *
 * [1] de Croon, G.C.H.E. (2016). Monocular distance estimation with optical flow maneuvers and efference copies:
 * a stability-based strategy. Bioinspiration & biomimetics, 11(1), 016004.
 * <http://iopscience.iop.org/article/10.1088/1748-3190/11/1/016004>
 */

#ifndef OPTICAL_FLOW_AVOIDANCE_H_
#define OPTICAL_FLOW_AVOIDANCE_H_

#include "std.h"

struct OpticalFlowAvoidance {
  float lp_const;               ///< low-pass filter constant
  float flow_setpoint;        ///< setpoint for constant divergence approach
  float pgain;                  ///< P-gain for constant optical flow control (from divergence error to thrust)
  float igain;                  ///< I-gain for constant optical flow control
  float dgain;                  ///< D-gain for constant optical flow control
  float flow;                 ///< Flow estimate
  float previous_err;           ///< Previous tracking error
  float sum_err;                ///< integration of the error for I-gain
  float d_err;                  ///< difference of error for the D-gain
  float nominal_thrust;         ///< nominal thrust around which the PID-control operates
  uint32_t CONTROL_METHOD;      ///< type of divergence control: 0 = fixed gain, 1 = adaptive gain, 2 = increase gain with steps
  float cov_set_point;          ///< for adaptive gain control, setpoint of the covariance (oscillations)
  float cov_limit;              ///< for fixed gain control, what is the cov limit triggering the landing
  float pgain_adaptive;         ///< P-gain for adaptive gain control
  float igain_adaptive;         ///< I-gain for adaptive gain control
  float dgain_adaptive;         ///< D-gain for adaptive gain control
  uint32_t COV_METHOD;          ///< method to calculate the covariance: between thrust and div (0) or div and div past (1)
  uint32_t delay_steps;         ///< number of delay steps for div past
  uint32_t window_size;         ///< number of time steps in "window" used for getting the covariance
  float lp_cov_factor;        ///< low-pass factor for the covariance of flow
};

extern struct OpticalFlowAvoidance of_avoidance_ctrl;

// Without optitrack set to: GUIDANCE_H_MODE_ATTITUDE
// With optitrack set to: GUIDANCE_H_MODE_HOVER / NAV
#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_NAV

// Own guidance_v
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

// Implement own Vertical loops
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool in_flight);

// used for logging purposes:
bool oscillating;
float cov_flow;
float pred_cov_flow;
float pstate, pused;
float last_time_ofa_run;

// struct containing most relevant parameters
struct OpticalFlowAvoidance of_avoidance_ctrl; // is it not already defined above?

#endif /* OPTICAL_FLOW_AVOIDANCE_H_ */
