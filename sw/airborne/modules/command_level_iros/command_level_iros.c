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
 * @file "modules/command_level_iros/command_level_iros.c"
 * @author Shuo Li
 * This module is the highest level
 */

#include <stdio.h>
#include "modules/command_level_iros/command_level_iros.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_clock.h"
#include "modules/state_autonomous_race/state_autonomous_race.h"
#include "modules/computer_vision/snake_gate_detection.h"
#include "modules/kalman_filter/kalman_filter.h"

#define PI 3.1415926

#define DESIRED_X_IN_FIRST_PART 15.0

uint8_t previous_mode;
uint8_t current_mode;
uint8_t previous_lower_level;

double theta_bias;
double phi_bias;
double theta_hover;
double phi_hover;
struct acceleration accel_bias;
struct acceleration accel_hover;
double desired_theta;
double desired_phi;

int arc_counter;
int arc_passed;
float reference_y;

void first_part_logic(void);
void second_part_logic(void);
void third_part_logic(void);
void fourth_part_logic(void);
void fifth_part_logic(void);

bool flag_init_geo;
enum states_lower_level state_lower_level ;
enum states_upper_level state_upper_level ;

// enum maneuver maneuvers[] = {ARC_L,TWO_ARCS_R,ZIGZAG_R,TWO_ARCS_L};
enum maneuver maneuvers[] = {STOP_AND_TURN,ZIGZAG_R,ZIGZAG_L,ZIGZAG_R,ARC_L};

float gate_initial_position_y[] = {3.0,2.0,2.0,4.0};
float turn_point[] = {5.0,2.5,2.5,4.5};
float gate_initial_heading[] = {00.0/PI*180, 180.0/180*3.14,180.0/180*3.14,90.0/180*3.14};

float gate_altitude[] = {-2.0,-2.3,-2.0,-1.5};
float open_loop_altitude[] = {-2.0,-2.0,-2.0,-1.5};


float break_time[] = {0.0,0.0,0.0,0.0};

/*int   flag_arc_right[] = {1,              -0,           -0,     0};*/
float arc_radius[] =     {1.5,             1.5,           1.0};
float delta_arc_angle[] = {180.0/180*3.14,135.0/180*3.14};


/*int   flag_2_arc_right[] = {0,             1,           -1,      1};*/
float two_arc_radius[] =     { 1.0,          1.5,           0.75};
float delta_2_arc_angle[] = {180.0/180*3.14,     180.0/180*3.14, 180.0/180*3.14, 180.0/180*3.14};


int   flag_zig_zag_right[] = {1,0,0};
int   flag_zig_zag_break[] = {1,1,0};
float zig_zag_desired_y[] = {6.0,6.0,0};
float zig_zag_break_time[] = {1.0,0,0};
float zig_zag_max_roll[] = {10.0/180*3.14,20.0/180*3.14,0};


float stop_and_turn_break_time = 1.0;

struct race_states race_state;

//temp fix for data set recording
int init_pos_filter = 0;

void command_init(){
    previous_mode = autopilot_mode;
    current_mode = autopilot_mode;
    init_heading = stateGetNedToBodyEulers_f()->psi;
	race_state.flag_in_open_loop = TRUE;
	race_state.gate_counter = 0;
}


void command_run() {
  
    current_mode = autopilot_mode;
    if (previous_mode != current_mode)
    {
        counter_autopilot_mode = 0;
        time_autopilot_mode = 0;
        primitive_in_use = NO_PRIMITIVE;
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		/*state_lower_level = PREPARE_CM; //PREPARE_CM;*/
		state_lower_level = FAST_TAKE_OFF_CM; //PREPARE_CM;
		// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		state_upper_level = FIRST_PART;
		/*state_upper_level = FOURTH_PART;*/
        init_heading = stateGetNedToBodyEulers_f()->psi;
		arc_passed = 0;
		arc_counter = 0;
		race_state.flag_in_open_loop = TRUE;
		race_state.gate_counter = 0;
		race_state.current_initial_x =  gate_initial_position_y[race_state.gate_counter];
		race_state.current_initial_heading= gate_initial_heading[race_state.gate_counter];
		race_state.desired_x_in_first_part = DESIRED_X_IN_FIRST_PART;
		two_arc_st.flag_in_two_arc_mode = FALSE;
		race_state.sum_y_error = 0.0;
    }
    if (autopilot_mode != AP_MODE_MODULE) {
        return;
    }


	if (race_state.flag_in_open_loop == TRUE)
	{
		race_state.current_initial_x =  gate_initial_position_y[race_state.gate_counter];
		race_state.current_initial_heading= gate_initial_heading[race_state.gate_counter] ;
	}



    if(state_upper_level  == FIRST_PART)
    {
        first_part_logic();
    }

    if(state_upper_level  == SECOND_PART)
    {
        second_part_logic();
    }

    if(state_upper_level  == THIRD_PART)
    {
        third_part_logic();
    }

    if(state_upper_level  == FOURTH_PART)
    {
        fourth_part_logic();
    }

    if(state_upper_level  == FIFTH_PART)
    {
			fifth_part_logic();
    }
    previous_mode = current_mode;
}








void first_part_logic()
{
    switch(state_lower_level)
    {
        case PREPARE_CM:
				if(prepare_before_take_off(1.0) == TRUE)
                state_lower_level = TAKE_OFF_OPEN_LOOP_CM;
            break;
			
		case TAKE_OFF_OPEN_LOOP_CM:
			if(take_off())
			{
				previous_lower_level = TAKE_OFF_OPEN_LOOP_CM;
				state_lower_level =  GO_STRAIGHT_CM;
				state_upper_level =  THIRD_PART;


				// !!--------------------------------------------------------
				race_state.gate_counter = 0;
				// !!--------------------------------------------------------
			}
			break;
		case FAST_TAKE_OFF_CM:
			if(take_off_fast())
			{
				previous_lower_level = FAST_TAKE_OFF_CM;
				state_lower_level =  GO_STRAIGHT_CM;
				state_upper_level =  SECOND_PART;
			}
			break;
		default:
			break;

    }
}


void second_part_logic()
{
	switch(state_lower_level)
	{

			case GO_STRAIGHT_CM:
					//printf("go straght !!!!!!!!!!!\n");
					if (go_through_open_gate(-5.0/180*PI,race_state.desired_x_in_first_part))
					{ 
							state_lower_level =  TURN_CM;
							race_state.flag_in_open_loop = TRUE;
					}
					break;
			case TURN_CM:
			  /*printf("The arc in first strech is executed__________________!!!!!!!!!!!!!!!!!\n");*/
					if(arc_open_loop(2.0,-5.0/180*3.14,90.0/180*PI,0,0))
					{
							race_state.gate_counter = 0; // clear gate counter since in arc_open_loop gate_counter++
							state_upper_level = THIRD_PART;
							/*state_upper_level = SECOND_PART;*/
							state_lower_level = GO_STRAIGHT_CM;
					}

			default:
					break;
	}
}

void third_part_logic()
{
	switch(state_lower_level)
	{
			case GO_STRAIGHT_CM:
					if (go_through_gate(-5.0/180*PI))
					{
							if(maneuvers[race_state.gate_counter] == ARC_L || maneuvers[race_state.gate_counter] == ARC_R ) 
							{
									state_lower_level =  ARC_CM;
									race_state.flag_in_open_loop = TRUE;
									race_state.current_arc_radius = arc_radius[race_state.gate_counter];
									race_state.current_arc_delta_psi= delta_arc_angle[race_state.gate_counter];

									if(maneuvers[race_state.gate_counter] == ARC_L)
											race_state.current_arc_flag_right = FALSE;
									else
											race_state.current_arc_flag_right = TRUE;
							}

							
							else if(maneuvers[race_state.gate_counter] == TWO_ARCS_L || maneuvers[race_state.gate_counter] == TWO_ARCS_R)
							{
									state_lower_level =  TWO_ARCS_CM;
									race_state.flag_in_open_loop = TRUE;
									race_state.current_2_arcs_radius= two_arc_radius[race_state.gate_counter];
									if(maneuvers[race_state.gate_counter] == TWO_ARCS_L)
									race_state.current_2_arcs_flag_right = FALSE;
									else
									race_state.current_2_arcs_flag_right = TRUE;
									race_state.current_2_arcs_delta_heading = delta_2_arc_angle[race_state.gate_counter];
							}
							else if(maneuvers[race_state.gate_counter] == ZIGZAG_R|| maneuvers[race_state.gate_counter] == ZIGZAG_L)
							{
									state_lower_level = ZIGZAG_CM;
									race_state.current_zigzag_break_time = zig_zag_break_time[race_state.gate_counter];
									race_state.current_zigzag_desired_y= zig_zag_desired_y[race_state.gate_counter];
									race_state.current_zigzag_max_roll= zig_zag_max_roll[race_state.gate_counter];
							}	
							else if(maneuvers[race_state.gate_counter] == STOP_AND_TURN) 
							{
									state_lower_level =  STOP_TURN_CM;
									race_state.flag_in_open_loop = TRUE;
							}
					}
					break;
			case ARC_CM:
				if(	arc_open_loop(race_state.current_arc_radius,-5.0/180*3.14,race_state.current_arc_delta_psi,race_state.current_arc_flag_right,0))
				{
							previous_mode = ARC_CM;
							race_state.flag_in_open_loop = FALSE;
							state_lower_level = GO_STRAIGHT_CM;
				}
				break;
			case ZIGZAG_CM:
				/*printf("zigzag mode\n");*/
				if(zigzag_2(race_state.current_zigzag_break_time,race_state.current_zigzag_max_roll,race_state.current_zigzag_desired_y))
				{
							previous_mode = ZIGZAG_CM;
							race_state.flag_in_open_loop = FALSE;
							state_lower_level = GO_STRAIGHT_CM;
				}
				break;
			case TWO_ARCS_CM:
				/*printf("TWO arc mode \n");*/
				if(two_arcs_open_loop(race_state.current_2_arcs_radius,-5.0/180*3.14,race_state.current_2_arcs_flag_right,race_state.current_2_arcs_delta_heading))
				{
						previous_mode = TWO_ARCS_CM;
						race_state.flag_in_open_loop = FALSE;
						state_lower_level = GO_STRAIGHT_CM;
				}
				break;
			case STOP_TURN_CM:
				if(stop_turn(stop_and_turn_break_time,90.0/180.0*PI))
				{
						previous_mode = STOP_TURN_CM;
						race_state.flag_in_open_loop = FALSE;
						state_lower_level = GO_STRAIGHT_CM;

				}
				break;
			default:
					break;
	}
}



void fourth_part_logic() {
		switch(state_lower_level)		
		{
				case GO_STRAIGHT_CM:
						if (go_through_gate(-5.0/180*PI))
						{
								state_lower_level =  ARC_CM;
								race_state.flag_in_open_loop = TRUE;
								race_state.current_arc_radius = 2.8;
								race_state.current_arc_delta_psi= 90.0/180*PI;
								race_state.current_arc_flag_right = TRUE;
						}
						break;

				case ARC_CM:
				if(	arc_open_loop(race_state.current_arc_radius,-5.0/180*3.14,race_state.current_arc_delta_psi,race_state.current_arc_flag_right,1))
				{
						state_lower_level = ARC_CM;
						state_upper_level = FIFTH_PART;
				}
				break;

				default:
				break;
		}

}


void fifth_part_logic()
{
}
