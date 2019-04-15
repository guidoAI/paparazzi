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
 * @file modules/ctrl/optical_flow_avoidance.h
 * @brief This module implements various optical flow avoidance methods.
 *
 * A first method uses the optical flow stability theory from [1] to keep the flow (horizontal, vertical, or flow) constant
 * while flying forwards. The drone stops when oscillations arise.
 *
 * [1] de Croon, G.C.H.E. (2016). Monocular distance estimation with optical flow maneuvers and efference copies:
 * a stability-based strategy. Bioinspiration & biomimetics, 11(1), 016004.
 * <http://iopscience.iop.org/article/10.1088/1748-3190/11/1/016004>
 *
 */

#include "optical_flow_avoidance.h"

#include "generated/airframe.h"
#include "paparazzi.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/guidance/guidance_v_adapt.h"

// used for automated landing:
#include "autopilot.h"
#include "subsystems/navigation/common_flight_plan.h"
#include "subsystems/datalink/telemetry.h"

// for measuring time
#include "mcu_periph/sys_time.h"

#include "math/pprz_stat.h"

// for moving forward:
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"

#include "stdio.h"

// Textons:
#include "modules/computer_vision/textons.h"
float *texton_distribution;
float *last_texton_distribution;
float *weights;
// On Bebop 2:
#ifdef TEXTONS_DICTIONARY_PATH
#define TEXTON_DISTRIBUTION_PATH TEXTONS_DICTIONARY_PATH
#else
#define TEXTON_DISTRIBUTION_PATH /data/ftp/internal_000
#endif
static FILE *distribution_logger = NULL;
static FILE *weights_file = NULL;

void save_texton_distribution(void);
float linear_prediction(float *distribution);
void save_weights(void);
void load_weights(void);

// variables for moving forward:
uint8_t safeToGoForwards = false;

/* Use optical flow estimates */
#ifndef OFA_OPTICAL_FLOW_ID
#define OFA_OPTICAL_FLOW_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(OFA_OPTICAL_FLOW_ID)

// Other default values:
#ifndef OFA_PGAIN
#define OFA_PGAIN 5.0
#endif

#ifndef OFA_IGAIN
#define OFA_IGAIN 0.01
#endif

#ifndef OFA_DGAIN
#define OFA_DGAIN 0.0
#endif

#ifndef OFA_CONTROL_METHOD
#define OFA_CONTROL_METHOD 0
#endif

#ifndef OFA_COV_METHOD
#define OFA_COV_METHOD 0
#endif

// number of time steps used for calculating the covariance (oscillations)
#ifndef OFA_COV_WINDOW_SIZE
#define OFA_COV_WINDOW_SIZE 30
#endif

#ifndef OFA_COV_LIMIT
#define OFA_COV_LIMIT 0.05
#endif

#ifndef OFA_COV_SETPOINT
#define OFA_COV_SETPOINT -0.03
#endif

#ifndef OFA_LP_CONST
#define OFA_LP_CONST 0.001
#endif

// Constants
// minimum value of the P-gain for optical flow control
// adaptive control / exponential gain control will not be able to go lower
#define MINIMUM_GAIN 0.1

// for exponential gain landing, gain increase per second during the first (hover) phase:
#define INCREASE_GAIN_PER_SECOND 0.02

// variables retained between module calls
float flow_vision;
float normalized_thrust;
float istate;
float dstate;
float vision_time,  prev_vision_time;
bool landing;
bool turning;
float new_heading;
float turn_degree = 110.0;
float previous_cov_err;
int32_t thrust_set;
float flow_setpoint;
float lp_cov_flow;

// incremental increase in gain:
float start_gain_increase_time;
float step_time;
float gain_increase;
int count_covflow;
int control_phase;

static abi_event optical_flow_ev;

// sending the flow message to the ground station:
// TODO: make a new message here!
static void send_flow_avoidance(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_FLOW_AVOIDANCE(trans, dev, AC_ID,
                               &(of_avoidance_ctrl.flow), &normalized_thrust,
                               &cov_flow, &pstate, &pused);
}

/// Function definitions
// Callback function of the optical flow estimate:
void flow_avoidance_ctrl_optical_flow_cb(uint8_t sender_id, uint32_t stamp, int16_t flow_x,
    int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, float quality, float size_flow, float dist);

// common functions for different optical flow control strategies:
void clear_cov_flow();
static void set_cov_flow(int32_t thrust);
static int32_t PID_flow_control(float flow_setpoint, float P, float I, float D, float dt);
static void update_errors(float error, float dt);
static uint32_t final_landing_procedure(void);

// resetting all variables to be called for instance when starting up / re-entering module
static void reset_all_vars(void);

float thrust_history[OFA_COV_WINDOW_SIZE];
float flow_history[OFA_COV_WINDOW_SIZE];
float past_flow_history[OFA_COV_WINDOW_SIZE];
uint32_t ind_hist;
uint8_t cov_array_filled;

void flow_avoidance_ctrl_module_init(void);
void flow_avoidance_ctrl_module_run(bool in_flight);
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);

// Avoidance control of lateral position with optitrack:
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
float get_heading_after_turn(float turn_degree);
uint8_t turn_to_new_heading();

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  //printf("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  struct EnuCoor_i *pos             = stateGetPositionEnu_i(); // Get your current position
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  // Calculate the sine and cosine of the heading the drone is keeping
  float sin_heading                 = sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  float cos_heading                 = cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  // Now determine where to place the waypoint you want to go to
  new_coor->x                       = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
  new_coor->y                       = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
  /*printf("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,
         POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y), POS_FLOAT_OF_BFP(pos->x), POS_FLOAT_OF_BFP(pos->y),
         DegOfRad(ANGLE_FLOAT_OF_BFP(eulerAngles->psi)));*/
  return false;
}

float get_heading_after_turn(float turn_degree) {
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  float current_angle = DegOfRad(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  float new_angle = RadOfDeg(current_angle + turn_degree);
  new_angle = fmod(new_angle, 2*M_PI);
  // printf("Old angle = %f, new angle = %f\n", current_angle, new_angle);
  return new_angle;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

uint8_t turn_to_new_heading() {
  // TODO: is once not enough?
  nav_set_heading_rad(new_heading);
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  float current_angle = ANGLE_FLOAT_OF_BFP(eulerAngles->psi);
  float diff1 = fabsf(current_angle - new_heading);
  float diff2 = fabsf(2*M_PI + current_angle - new_heading);
  if(diff1 < diff2 && diff1 < 0.01 * M_PI) {
      // turning done!
      return 1;
  }
  else if(diff2 < diff1 && diff2 < 0.01 * M_PI) {
      // turning done!
      return 1;
  }
  else {
      // keep turning
      return 0;
  }
}


/**
 * Initialize the optical flow landing module
 */
void flow_avoidance_ctrl_module_init(void)
{
  // filling the of_avoidance_ctrl struct with default values:
  of_avoidance_ctrl.flow_setpoint = 0.0f; // For exponential gain landing, pick a negative value
  of_avoidance_ctrl.cov_set_point = OFA_COV_SETPOINT;
  of_avoidance_ctrl.cov_limit = fabsf(OFA_COV_LIMIT);
  of_avoidance_ctrl.lp_const = OFA_LP_CONST;
  Bound(of_avoidance_ctrl.lp_const, 0.001f, 1.f);
  of_avoidance_ctrl.pgain = OFA_PGAIN;
  of_avoidance_ctrl.igain = OFA_IGAIN;
  of_avoidance_ctrl.dgain = OFA_DGAIN;
  of_avoidance_ctrl.flow = 0.;
  of_avoidance_ctrl.previous_err = 0.;
  of_avoidance_ctrl.sum_err = 0.0f;
  of_avoidance_ctrl.d_err = 0.0f;
  of_avoidance_ctrl.nominal_thrust = (float)guidance_v_nominal_throttle / MAX_PPRZ; // copy this value from guidance
  of_avoidance_ctrl.CONTROL_METHOD = OFA_CONTROL_METHOD;
  of_avoidance_ctrl.COV_METHOD = OFA_COV_METHOD;
  of_avoidance_ctrl.delay_steps = 15;
  of_avoidance_ctrl.window_size = OFA_COV_WINDOW_SIZE;
  of_avoidance_ctrl.pgain_adaptive = OFA_PGAIN;
  of_avoidance_ctrl.igain_adaptive = 0.03;
  of_avoidance_ctrl.dgain_adaptive = OFA_DGAIN;
  of_avoidance_ctrl.lp_cov_factor = 0.99f; // low pass filtering cov

  step_time = 10.0f;
  gain_increase = 0.25;
  count_covflow = 0;
  control_phase = 0;
  reset_all_vars();

  // Subscribe to the optical flow estimator:
  // register telemetry:
  AbiBindMsgOPTICAL_FLOW(OFA_OPTICAL_FLOW_ID, &optical_flow_ev, flow_avoidance_ctrl_optical_flow_cb);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FLOW_AVOIDANCE, send_flow_avoidance);
  // TODO: are these freed?
  texton_distribution = (float *) calloc(MAX_N_TEXTONS, sizeof(float));
  last_texton_distribution = (float *) calloc(MAX_N_TEXTONS, sizeof(float));
  // TODO: only load at init?
  weights = (float *) calloc(MAX_N_TEXTONS, sizeof(float));
  load_weights();
}

/**
 * Reset all variables:
 */
static void reset_all_vars(void)
{
  thrust_set = of_avoidance_ctrl.nominal_thrust * MAX_PPRZ;

  cov_flow = 0.;
  normalized_thrust = of_avoidance_ctrl.nominal_thrust * 100;
  previous_cov_err = 0.;
  flow_vision = 0.;
  flow_setpoint = 0;
  vision_time = get_sys_time_float();
  prev_vision_time = vision_time;
  ind_hist = 0;
  cov_array_filled = 0;
  uint32_t i;
  for (i = 0; i < OFA_COV_WINDOW_SIZE; i++) {
    thrust_history[i] = 0;
    flow_history[i] = 0;
  }
  landing = false;
  turning = false;
  new_heading = 0.0;
  lp_cov_flow = 0.0f;

  pstate = of_avoidance_ctrl.pgain;
  pused = pstate;
  istate = of_avoidance_ctrl.igain;
  dstate = of_avoidance_ctrl.dgain;

  of_avoidance_ctrl.flow = 0.;
  of_avoidance_ctrl.previous_err = 0.;
  of_avoidance_ctrl.sum_err = 0.;
  of_avoidance_ctrl.d_err = 0.;

  oscillating = false;

  // negative to indicate that it has not been set yet:
  start_gain_increase_time = -1.0;
  count_covflow = 0;
  control_phase = 0;
}

/**
 * Run the optical flow landing module
 */
void flow_avoidance_ctrl_module_run(bool in_flight)
{
  float flow_factor; // factor that maps flow in pixels as received from vision to 1 (?) / frame

  float dt = vision_time - prev_vision_time;

  // check if new measurement received
  if (dt <= 1e-5f) {
    return;
  }

  Bound(of_avoidance_ctrl.lp_const, 0.001f, 1.f);
  float lp_factor = dt / of_avoidance_ctrl.lp_const;
  Bound(lp_factor, 0.f, 1.f);

  /***********
   * VISION
   ***********/
  // TODO: this flow_factor depends on the subpixel-factor (automatically adapt?)
  // TODO: this factor is camera specific and should be implemented in the optical
  // flow calculator module not here. Additionally, the time scaling should also
  // be done in the calculator module
  flow_factor = -1.28f; // magic number comprising field of view etc.
  float new_flow = (flow_vision * flow_factor) / dt;

  // deal with (unlikely) fast changes in flow:
  static const float max_flow_dt = 0.20f;
  if (fabsf(new_flow - of_avoidance_ctrl.flow) > max_flow_dt) {
    if (new_flow < of_avoidance_ctrl.flow) { new_flow = of_avoidance_ctrl.flow - max_flow_dt; }
    else { new_flow = of_avoidance_ctrl.flow + max_flow_dt; }
  }

  // low-pass filter the flow:
  of_avoidance_ctrl.flow += (new_flow - of_avoidance_ctrl.flow) * lp_factor;
  prev_vision_time = vision_time;

  /***********
  * CONTROL
  ***********/
  // landing indicates whether the drone is already performing a final landing procedure (flare):
  if (!landing) {

    last_time_ofa_run = get_sys_time_float();

    if(!turning) {
	// set the flow set point to not loose / gain too much altitude:
	struct EnuCoor_f *pos_rob = stateGetPositionEnu_f();
	// hysteresis:
	float threshold_des;
	if(of_avoidance_ctrl.flow_setpoint == 0.0f) {
	    threshold_des = 0.25f;
	}
	else {
	    threshold_des = 0.10f;
	}
	static float desired_magnitude = 0.01f;
	if(pos_rob->z < waypoint_get_alt(WP_GOAL) - threshold_des) {
	    // apparently, the nominal thrust is not good:
	    if(of_avoidance_ctrl.flow_setpoint == 0.0f) {
		of_avoidance_ctrl.nominal_thrust += 0.005;
	    }
	    of_avoidance_ctrl.flow_setpoint = desired_magnitude;
	}
	else if(pos_rob->z > waypoint_get_alt(WP_GOAL) + threshold_des) {
	    // apparently, the nominal thrust is not good:
	    if(of_avoidance_ctrl.flow_setpoint == 0.0f) {
		of_avoidance_ctrl.nominal_thrust -= 0.005;
	    }
	    of_avoidance_ctrl.flow_setpoint = -desired_magnitude;
	}
	else {
	    of_avoidance_ctrl.flow_setpoint = 0.0f;
	}
	printf("Altitude drone = %f, waypoint = %f, flow set point = %f, nominal thrust = %f\n", pos_rob->z, waypoint_get_alt(WP_GOAL), of_avoidance_ctrl.flow_setpoint, of_avoidance_ctrl.nominal_thrust);
    }


    // If turning, we first correct the psi, before moving forward with any of the control methods:
    if(turning) {
	uint8_t done = turn_to_new_heading();
	thrust_set = of_avoidance_ctrl.nominal_thrust * MAX_PPRZ;
	set_cov_flow(thrust_set);
	if(done) turning = false; // resume avoidance control.
    }
    else if (of_avoidance_ctrl.CONTROL_METHOD == 0) {
      // FIXED GAIN CONTROL, cov_limit for landing:
      float moveDistance = 0.25;
      moveWaypointForward(WP_GOAL, moveDistance);
      nav_set_heading_towards_waypoint(WP_GOAL);

      // use the flow for control:
      thrust_set = PID_flow_control(of_avoidance_ctrl.flow_setpoint, of_avoidance_ctrl.pgain, of_avoidance_ctrl.igain,
                                    of_avoidance_ctrl.dgain, dt);

      // save the texton distribution:
      save_texton_distribution();

      // predict cov flow (not necessary here):
      float pred_cov_flow = linear_prediction(texton_distribution);
      printf("Cov div predicted: %f, measured: %f\n", pred_cov_flow, cov_flow);

      // trigger a stop if the cov div is too high:
      if (fabsf(cov_flow) > of_avoidance_ctrl.cov_limit) {

	waypoint_set_here_2d(WP_GOAL);

	// Comment the following one line for turning:
	// thrust_set = final_landing_procedure();

        // Uncomment the following two lines for turning:
        new_heading = get_heading_after_turn(turn_degree);
        clear_cov_flow();
        turning = true;
      }

    } else if (of_avoidance_ctrl.CONTROL_METHOD == 1) {
      // ADAPTIVE GAIN CONTROL:
      // TODO: i-gain and d-gain are currently not adapted

      // adapt the gains according to the error in covariance:
      float error_cov = of_avoidance_ctrl.cov_set_point - cov_flow;

      // for logging purposes
      if (fabsf(error_cov) <= 0.025) { oscillating = true; }
      else { oscillating = false; }

      // limit the error_cov, which could else become very large:
      // TODO: why should error_cov be positive? Are the fabsf correctly used here?
      if (error_cov > fabsf(of_avoidance_ctrl.cov_set_point)) { error_cov = fabsf(of_avoidance_ctrl.cov_set_point); }
      pstate -= (of_avoidance_ctrl.igain_adaptive * pstate) * error_cov;
      if (pstate < MINIMUM_GAIN) { pstate = MINIMUM_GAIN; }
      pused = pstate - (of_avoidance_ctrl.pgain_adaptive * pstate) * error_cov;
      // make sure pused does not become too small, nor grows too fast:
      if (pused < MINIMUM_GAIN) { pused = MINIMUM_GAIN; }
      /*if (of_avoidance_ctrl.COV_METHOD == 1 && error_cov > 0.001) {
        pused = 0.5 * pused;
      }*/

      // use the flow for control:
      thrust_set = PID_flow_control(of_avoidance_ctrl.flow_setpoint, pused, of_avoidance_ctrl.igain,
                                    of_avoidance_ctrl.dgain, dt);

    } else if (of_avoidance_ctrl.CONTROL_METHOD == 2) {
      if (control_phase == 0) {
        // increase the gain by steps:
        if (start_gain_increase_time < 0.0) {
          start_gain_increase_time = get_sys_time_float();
        }
        float t = get_sys_time_float();

        // using (int) as a floor function : )
        pstate = (float)((int)((t - start_gain_increase_time) / step_time)) * gain_increase;
        pused = pstate;

        thrust_set = PID_flow_control(of_avoidance_ctrl.flow_setpoint, pused, of_avoidance_ctrl.igain, of_avoidance_ctrl.dgain,
                                      dt);

        float error_cov = of_avoidance_ctrl.cov_set_point - cov_flow;
        // for logging purposes
        if (fabsf(error_cov) <= 0.01) { count_covflow++; }
        if (count_covflow > 10) { oscillating = true; }
        if (oscillating) {
          control_phase = 1;
        }
      } else {
        // we keep the same gain for now:
        thrust_set = PID_flow_control(of_avoidance_ctrl.flow_setpoint, pused, of_avoidance_ctrl.igain, of_avoidance_ctrl.dgain,
                                      dt);
        // TODO: if the drone is stable for many seconds, we should start increasing again.
      }
    } else if (of_avoidance_ctrl.CONTROL_METHOD == 3) {
	// use the predictor to stop, instead of the oscillations:
	// FIXED GAIN CONTROL, cov_limit for landing:
	float moveDistance = 0.25;
	moveWaypointForward(WP_GOAL, moveDistance);
	nav_set_heading_towards_waypoint(WP_GOAL);

	// use the flow for control:
	thrust_set = PID_flow_control(of_avoidance_ctrl.flow_setpoint, of_avoidance_ctrl.pgain, of_avoidance_ctrl.igain,
				      of_avoidance_ctrl.dgain, dt);

	// save the texton distribution:
	save_texton_distribution();
	float pred_cov_flow = linear_prediction(texton_distribution);
	printf("Cov div predicted: %f, measured: %f\n", pred_cov_flow, cov_flow);

	// trigger a stop if the PREDICTION of cov div is too high:
	if (fabsf(pred_cov_flow) > of_avoidance_ctrl.cov_limit / 2.0f) {

	    waypoint_set_here_2d(WP_GOAL);

	    // Comment the following one line for turning:
	    // thrust_set = final_landing_procedure();

	    // Uncomment the following two lines for turning:
	    new_heading = get_heading_after_turn(turn_degree);
	    clear_cov_flow();
	    turning = true;
	}
    }

    // TODO: is this the landing procedure?
    if (in_flight) {
      Bound(thrust_set, 0.25 * of_avoidance_ctrl.nominal_thrust * MAX_PPRZ, MAX_PPRZ);
      stabilization_cmd[COMMAND_THRUST] = thrust_set;
    }

  }
}

/**
 * Execute a final landing procedure
 */
uint32_t final_landing_procedure()
{
  // land with 85% nominal thrust:
  uint32_t nominal_throttle = of_avoidance_ctrl.nominal_thrust * MAX_PPRZ;
  uint32_t thrust = 0.85 * nominal_throttle;
  Bound(thrust, 0.6 * nominal_throttle, 0.9 * MAX_PPRZ);
  landing = true;

  return thrust;
}


void clear_cov_flow() {

  int32_t thrust = of_avoidance_ctrl.nominal_thrust * MAX_PPRZ;
  normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));

  for (uint16_t i = 0; i < of_avoidance_ctrl.window_size; i++) {
      thrust_history[i] = normalized_thrust;
      flow_history[i] = 0;
    }
}

/**
 * Set the covariance of the flow and the thrust / past flow
 * This funciton should only be called once per time step
 * @param[in] thrust: the current thrust value
 */
void set_cov_flow(int32_t thrust)
{
  // histories and cov detection:
  flow_history[ind_hist] = of_avoidance_ctrl.flow;

  normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));
  thrust_history[ind_hist] = normalized_thrust;

  int ind_past = ind_hist - of_avoidance_ctrl.delay_steps;
  while (ind_past < 0) { ind_past += of_avoidance_ctrl.window_size; }
  past_flow_history[ind_hist] = flow_history[ind_past];

  // determine the covariance for landing detection:
  // only take covariance into account if there are enough samples in the histories:
  if (of_avoidance_ctrl.COV_METHOD == 0 && cov_array_filled > 0) {
    // TODO: step in landing set point causes an incorrectly perceived covariance
    cov_flow = covariance_f(thrust_history, flow_history, of_avoidance_ctrl.window_size);
  } else if (of_avoidance_ctrl.COV_METHOD == 1 && cov_array_filled > 1) {
    // todo: delay steps should be invariant to the run frequency
    cov_flow = covariance_f(past_flow_history, flow_history, of_avoidance_ctrl.window_size);
  }

  if (cov_array_filled < 2 && ind_hist + 1 == of_avoidance_ctrl.window_size) {
    cov_array_filled++;
  }
  ind_hist = (ind_hist + 1) % of_avoidance_ctrl.window_size;
}

/**
 * Determine and set the thrust for constant optical flow control
 * @param[out] thrust
 * @param[in] flow_set_point: The desired flow
 * @param[in] P: P-gain
 * @param[in] I: I-gain
 * @param[in] D: D-gain
 * @param[in] dt: time difference since last update
 */
int32_t PID_flow_control(float setpoint, float P, float I, float D, float dt)
{
  // determine the error:
  float err = setpoint - of_avoidance_ctrl.flow;

  // update the controller errors:
  update_errors(err, dt);

  // PID control:
  int32_t thrust = (of_avoidance_ctrl.nominal_thrust
                    + P * err
                    + I * of_avoidance_ctrl.sum_err
                    + D * of_avoidance_ctrl.d_err) * MAX_PPRZ;

  // bound thrust:
  Bound(thrust, 0.25 * of_avoidance_ctrl.nominal_thrust * MAX_PPRZ, MAX_PPRZ);
  // Bound(thrust, 0.5 * of_avoidance_ctrl.nominal_thrust * MAX_PPRZ, 0.85 * MAX_PPRZ);
  // update covariance
  set_cov_flow(thrust);

  return thrust;
}

/**
 * Updates the integral and differential errors for PID control and sets the previous error
 * @param[in] err: the error of the flow and flow setpoint
 * @param[in] dt:  time difference since last update
 */
void update_errors(float err, float dt)
{
  float lp_factor = dt / of_avoidance_ctrl.lp_const;
  Bound(lp_factor, 0.f, 1.f);

  // maintain the controller errors:
  of_avoidance_ctrl.sum_err += err;
  of_avoidance_ctrl.d_err += (((err - of_avoidance_ctrl.previous_err) / dt) - of_avoidance_ctrl.d_err) * lp_factor;
  of_avoidance_ctrl.previous_err = err;
}

// Reading from "sensors":
void flow_avoidance_ctrl_optical_flow_cb(uint8_t sender_id UNUSED, uint32_t stamp, int16_t flow_x UNUSED,
    int16_t flow_y UNUSED, int16_t flow_der_x, int16_t flow_der_y UNUSED, float quality UNUSED, float size_flow UNUSED,
    float dist UNUSED)
{
  flow_vision = (float) flow_der_x / 10000.0f; // size_flow;
  vision_time = ((float)stamp) / 1e6;
}

////////////////////////////////////////////////////////////////////
// Call our controller
void guidance_v_module_init(void)
{
  flow_avoidance_ctrl_module_init();
}

/**
 * Entering the module (user switched to module)
 */
void guidance_v_module_enter(void)
{
  reset_all_vars();

  // adaptive estimation - assume hover condition when entering the module
  of_avoidance_ctrl.nominal_thrust = (float) stabilization_cmd[COMMAND_THRUST] / MAX_PPRZ;
  thrust_set = of_avoidance_ctrl.nominal_thrust * MAX_PPRZ;
  // TODO: REMOVE BEFORE FLIGHT:
  // of_avoidance_ctrl.nominal_thrust -= 0.01 * of_avoidance_ctrl.nominal_thrust;
}

void guidance_v_module_run(bool in_flight)
{
  flow_avoidance_ctrl_module_run(in_flight);
}

// SSL:
void save_texton_distribution(void)
{
  // Since the control module typically runs faster than the texton vision process, we need to check that we are storing a recent vision result:
  int i, same;
  same = 1;
  last_texton_distribution = get_texton_distribution();
  for (i = 0; i < n_textons; i++) {
    // check if the texton distribution is the same as the previous one:
    if (texton_distribution[i] != last_texton_distribution[i]) {
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
  }
  */

  // If not the same, append the target values (cov_div, gain, class: close or far) and texton values to a .dat file:
  char filename[512];
  sprintf(filename, "%s/Training_set_%05d.dat", STRINGIFY(TEXTON_DISTRIBUTION_PATH), 0);
  distribution_logger = fopen(filename, "a");
  if (distribution_logger == NULL) {
    perror(filename);
  } else {
    int oscillating = fabsf(cov_flow) > of_avoidance_ctrl.cov_limit;
    printf("Logging with gain %f, cov_flow %f, oscillating: %d\n", pstate, cov_flow, oscillating);

    // save the information in a single row:
    fprintf(distribution_logger, "%f ", pstate); // gain measurement
    fprintf(distribution_logger, "%f ", cov_flow); // cov flow measurement
    fprintf(distribution_logger, "%d ", oscillating); // classification
    for (i = 0; i < n_textons - 1; i++) {
      fprintf(distribution_logger, "%f ", texton_distribution[i]);
    }
    fprintf(distribution_logger, "%f\n", texton_distribution[n_textons - 1]);
    fclose(distribution_logger);
  }
}

/*
 * TODO: Do later when we can save:
 *
 *
void load_texton_distribution(void)
{
  int i, j, read_result;
  char filename[512];
  sprintf(filename, "%s/Training_set_%05d.dat", STRINGIFY(TEXTON_DISTRIBUTION_PATH), 0);

  if((distribution_logger = fopen(filename, "r")))
  {
    // Load the dictionary:
    int n_read_samples = 0;
    // For now we read the samples sequentially:
    // for(i = 0; i < MAX_SAMPLES_LEARNING; i++)
    for(i = 0; i < 30; i++)
    {
      // if(i % 100) printf("SONAR: %f\n", sonar[n_read_samples]);
      read_result = fscanf(distribution_logger, "%f ", &gains[n_read_samples]);
      if(read_result == EOF) break;
      read_result = fscanf(distribution_logger, "%f ", &cov_divs_log[n_read_samples]);
      if(read_result == EOF) break;
      read_result = fscanf(distribution_logger, "%f ", &sonar[n_read_samples]);
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

    // printf("Learned samples = %d\n", n_read_samples);
  }
}
*/

float linear_prediction(float *distribution)
{
  //printf("Start prediction!\n");
  int i;
  float sum;
  int use_bias = 1;
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
  for (i = 0; i < n_textons; i++) {
    sum += weights[i] * distribution[i];
  }
  if (use_bias) { // of_avoidance_ctrl.use_bias
    sum += weights[n_textons];
  }
  // printf("Prediction = %f\n", sum);
  return sum;
}

void save_weights(void)
{
  // save the weights to a file:
  int i;
  char filename[512];
  sprintf(filename, "%s/Weights_%05d.dat", STRINGIFY(TEXTON_DISTRIBUTION_PATH), 0);
  weights_file = fopen(filename, "w");
  if (weights_file == NULL) {
    perror(filename);
  } else {
    // save the information in a single row:
    for (i = 0; i <= n_textons; i++) {
      fprintf(weights_file, "%f ", weights[i]);
    }
    fclose(weights_file);
  }
}

void load_weights(void)
{
  int i, read_result;
  char filename[512];
  sprintf(filename, "%s/weights_%05d.dat", STRINGIFY(TEXTON_DISTRIBUTION_PATH), 0);
  weights_file = fopen(filename, "r");
  if (weights_file == NULL) {
    printf("No weights file!\n");
    perror(filename);
  } else {
    // load the weights, stored in a single row:
    for (i = 0; i <= n_textons; i++) {
      read_result = fscanf(weights_file, "%f ", &weights[i]);
      if (read_result == EOF) { break; }
    }
    fclose(weights_file);
  }
}
