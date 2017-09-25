/*
 * Copyright (C) Shuo Li
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/command_level_iros/command_level_iros.h"
 * @author Shuo Li
 * This module is the highest level
 */
#include "state.h"

#ifndef COMMAND_LEVEL_IROS_H
#define COMMAND_LEVEL_IROS_H


extern void command_run(void);  // 20HZ
extern void command_init(void);

enum states_lower_level{HOVER_AT_ORIGIN_CM,HOVER_CM,
TAKE_OFF_OPEN_LOOP_CM,TAKE_OFF_CLOSE_LOOP_CM,LAND_CM,GO_STRAIGHT_CM,PREPARE_CM,
     ZIGZAG_CM,ARC_CM,TWO_ARCS_CM,TURN_CM,STOP_TURN_CM,FAST_TAKE_OFF_CM}; 
enum states_upper_level{FIRST_PART,SECOND_PART,THIRD_PART,FOURTH_PART,FIFTH_PART};

enum maneuver{ARC_R,ARC_L,TWO_ARCS_L,TWO_ARCS_R,ZIGZAG_L,ZIGZAG_R,STOP_AND_TURN};


extern enum states_lower_level state_lower_level;
extern enum states_upper_level state_upper_level;

struct race_states
{

		bool flag_in_open_loop;             // gate = close loop, arc&zigzag are open-loop
		int gate_counter;                   // Only for the closed gates in curved section
		float current_initial_x;            // Gate-centered frame
		float current_initial_heading;

		float current_arc_radius;
		float current_arc_delta_psi;

		int current_arc_flag_right;         // 1=right, 0=left
		float current_zigzag_break_time;
		float current_zigzag_break_angle;
		float current_zigzag_desired_y;
		float current_zigzag_desired_theta;
		float current_zigzag_max_roll;
		int current_zigzag_flag_right;      // 1=right zigzag
		int current_zigzag_flag_break;      // Now we always break....

		// not used
		float current_2_arcs_radius;
		float current_2_arcs_flag_right;
		float current_2_arcs_delta_heading;

		// was PID -> now PD... -> not used
		float sum_y_error;

		float desired_x_in_first_part;      // decide when to turn: TODO
		float target_heading;
};


extern double theta_bias;
extern double phi_bias;
extern uint8_t previous_lower_level;
extern double theta_hover;
extern double phi_hover;
extern struct acceleration accel_bias;
extern struct acceleration accel_hover;
extern int arc_counter;
extern struct race_states race_state;
extern float gate_initial_position_y[];
extern float turn_point[];
extern float arc_radius[];
extern float delta_arc_angle[];
extern float gate_initial_heading[];
extern float gate_altitude[] ;
extern float open_loop_altitude[];
extern int  flag_2_arc_right[];
extern float two_arc_radius[];
extern float break_time[];
#endif

