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
 * @file "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.c"
 * @author Shuo Li
 * This module is used to generate flight plan in guided mode
 */

#include <stdio.h>
#include <stdlib.h>
#include "flight_plan_in_guided_mode.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "state.h"
#include "modules/guidance_loop_velocity_autonomous_race/guidance_loop_velocity_autonomous_race.h"
#include <math.h>
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "modules/state_autonomous_race/state_autonomous_race.h"
#include "modules/computer_vision/snake_gate_detection.h"
#include "subsystems/ins.h"
#include "modules/kalman_filter/kalman_filter.h"
#include "modules/first_stretch/first_stretch.h"
#include "subsystems/imu.h"
#include "math/pprz_algebra_float.h"

// #define KP_Y 0.55//0.4 
// #define KI_Y 0.0
// #define KD_Y 0.0//0.3
// #define MAX_PHI  20.0/180*3.14

//Optitrack
// #define KP_Y 0.4 
// #define KI_Y 0.0
// #define KD_Y 0.2

#define KP_Y 0.35//40 //was 0.4
#define KI_Y -0.00
#define KD_Y 0.30  //0.3//0.2//0.04//0.10//was0.15// 0.2
#define MAX_PHI  30.0/180*3.14//was 15 then 25 deg

//most turns until now
//P 0.4
//D 0.07
//Still suspected d gain noise, add smooth or lowpass

//With low pass
//close to optitrack performance, except for unknown glitch
//P 0.4
//D 0.04


float psi0;//
float v_x_f;
float psi1;
float psi_startup;
float z0;
float x_start;
float y_start;
float heading;
float body_velocity_x;
float vx_earth;
float vy_earth;
float psi;
float velocity_body_x;
float velocity_body_y;
float velocity_earth_x;
float velocity_earth_y;
float init_heading;
float previous_error_y = 0;
float sum_y_error = 0;
double g = 9.81;

//avarage current and previous derivative term 
float prev_D_term = 0;

int primitive_in_use; // This variable is used for showing which primitive is used now;

void flight_plan_in_guided_mode_init() {
    primitive_in_use = NO_PRIMITIVE;
}

void set_altitude(float altitude)
{
		guidance_v_z_sp = /*stateGetPositionNed_i()->z +*/ POS_BFP_OF_REAL(altitude);
		guidance_v_z_ref = guidance_v_z_sp;
		gv_set_ref(guidance_v_z_sp,0,0);
}

bool prepare_before_take_off(double prepare_time)
{
    if(primitive_in_use != PREPARE_BEFORE_TAKE_OFF){
        primitive_in_use = PREPARE_BEFORE_TAKE_OFF;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
    }
	if (time_primitive > prepare_time)
	{
			return 1;
	}
	else
	{
			return 0;
	}
}


void hover()
{
    if(primitive_in_use != HOVER){
        primitive_in_use = HOVER;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
        guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
    }
}

bool go_straight(float theta,float distance,double ref_y){
    if(primitive_in_use != GO_STRAIGHT){
        primitive_in_use = GO_STRAIGHT;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        /*guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);*/
        guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
        psi0 = stateGetNedToBodyEulers_f()->psi;
        z0 = stateGetPositionNed_f()->z;
        x_start = stateGetPositionNed_f()->x;
        y_start = stateGetPositionNed_f()->y;
		sum_y_error = 0;
		states_race.attitude_control = TRUE;
		float v_x_e = stateGetSpeedNed_f()->x;
		float v_y_e = stateGetSpeedNed_f()->y;
		v_x_f = cos(psi)*v_x_e +sin(psi)*v_y_e;
		if (arc_counter == 0)
		{

				guidance_loop_set_heading(0.0);
		}
		else
		{
				guidance_loop_set_heading(3.14);
		}
    }

	float current_y;
	float sign = 1;
	int use_optitrack = 0;//else use vision
	if(ref_y > 1.5){
	 sign = -1;
	 if(use_optitrack){
	   current_y = stateGetPositionNed_f()->y;
	 }else{
	   current_y = kf_pos_y;
	 }
	}
	else{
	  if(use_optitrack){
	    current_y = stateGetPositionNed_f()->y;//x_dist;//raw vision
	  }else{
	  //current_y = ls_pos_y;
	    current_y = kf_pos_y;
	  }
	}
	float error_y = (ref_y - current_y)*sign;
	sum_y_error += error_y/100.0;
	float D_term = error_y-previous_error_y;
// 	float cuttoff = 0.01;
// 	if(D_term > cuttoff)D_term = cuttoff;
// 	if(D_term < -cuttoff)D_term = -cuttoff;
	float phi = KP_Y * error_y+ KD_Y *((D_term+prev_D_term)/2)*100.0 + KI_Y*sum_y_error;
	prev_D_term = D_term;
// 	float phi = KP_Y * error_y+ KD_Y *(error_y-previous_error_y)*20.0 + KI_Y*sum_y_error;
	if(phi > MAX_PHI)phi = MAX_PHI;
	if(phi < -MAX_PHI)phi = -MAX_PHI;
	guidance_loop_set_theta(theta);
	guidance_loop_set_phi(phi); 
	/*guidance_loop_set_heading(psi0);*/
	previous_error_y = error_y;
	//if (time_primitive > 1.0)
	
	//update body speed for use in ahrs_int_cmpl_quat.c
	float v_x_e = stateGetSpeedNed_f()->x;
	float v_y_e = stateGetSpeedNed_f()->y;
	v_x_f = cos(psi)*v_x_e +sin(psi)*v_y_e;
	
	float turn_trigger;
	if(0){//use_optitrack){
	  turn_trigger = stateGetPositionNed_f()->x;
	}else{
	  turn_trigger = kf_pos_x;
	}
	
	if(ref_y < 1.5 && turn_trigger > 3.5)   
	{
			return TRUE;
	}
	else if(ref_y > 1.5 && turn_trigger < 0)   
	{
			return TRUE;
	}
	else
	{
			return FALSE;
	}
}


struct take_off_status tf_status;

bool take_off(void)
{
		if (primitive_in_use != TAKE_OFF)
		{
				psi_startup = stateGetNedToBodyEulers_f()->psi;
				primitive_in_use = TAKE_OFF;
				counter_primitive = 0;
				time_primitive = 0;
				tf_status.flag_open_loop = FALSE;
				tf_status.flag_climb_mode = FALSE;
				tf_status.flag_hover_mode = FALSE;
				guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
				states_race.attitude_control = FALSE;
				/*guidance_loop_set_theta(0);*/
				/*guidance_loop_set_phi(0);*/
				states_race.altitude_is_achieved = 0;
				tf_status.flag_open_loop = TRUE;
				tf_status.take_off_altitude = TAKE_OFF_ALTITUDE;
				tf_status.altitude_counter = 0;
				tf_status.sum_altitude = 0.0;
				tf_status.ave_altitude = 0.0;
				guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);  // vertical module should be called!
				set_altitude(TAKE_OFF_ALTITUDE);
				tf_status.flag_ekf_initialized = FALSE;

		}

		if (tf_status.flag_open_loop == TRUE)
		{
				/*guidance_loop_set_theta(-0/57.6);*/
				/*guidance_loop_set_phi(0);*/
				guidance_loop_set_velocity(0.0,0.0);
				guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);  // vertical module should be called!
				if (time_primitive > 3.0 && tf_status.flag_ekf_initialized == FALSE)
				{
						tf_status.flag_ekf_initialized = TRUE;
						race_state.flag_in_open_loop = FALSE;
						initialize_EKF();
				}
				if (time_primitive > 6.0)
				{
						tf_status.flag_open_loop = FALSE;
						tf_status.flag_hover_mode = TRUE;
						race_state.flag_in_open_loop = FALSE;
						guidance_loop_set_velocity(0.0,0.0);
						//guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);  // vertical module should be called!
						/*printf("gate initial heading is %f\n",gate_initial_heading[race_state.gate_counter]);*/
						/*printf("gate initial y is %f\n",gate_initial_position_y[race_state.gate_counter]);*/
						return TRUE;
				}
				//printf("Take off attitude is %f\n",tf_status.take_off_altitude);
		}
		else if(tf_status.flag_hover_mode == TRUE)
		{
				//guidance_v_set_guided_z(tf_status.take_off_altitude) ;
				tf_status.sum_altitude += stateGetPositionNed_f()->z;
				//printf("Take off attitude is %f\n",tf_status.take_off_altitude);
				race_state.flag_in_open_loop = FALSE;
				tf_status.altitude_counter ++;
				tf_status.ave_altitude = tf_status.sum_altitude/tf_status.altitude_counter;

				//printf("average altitide is %f\n",tf_status.ave_altitude);
				//printf("sum altitide is %f\n",tf_status.sum_altitude );
				//printf("altitude counter is %d\n",tf_status.altitude_counter);
		}
		/*if (fabs(stateGetPositionNed_f()->z-tf_status.ave_altitude)<0.1 && tf_status.altitude_counter > 1000 )*/
		if (tf_status.altitude_counter > 1000 )
		{
				tf_status.flag_open_loop = FALSE;
				tf_status.flag_climb_mode = FALSE;
				tf_status.flag_hover_mode = FALSE;
				/*printf("exit take off !!!!!\n");*/
				return TRUE;
		}
		else
		{
				return FALSE;
		}
}

void land()
{
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
		guidance_v_set_guided_z(0.0);
        guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
}




void hold_altitude(float desired_altitude)
{
    if (primitive_in_use != HOLD_ALTITUDE)
    {
        primitive_in_use = HOLD_ALTITUDE;
        guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
        counter_primitive = 0;
        time_primitive = 0;
        states_race.altitude_is_achieved = 0;
        guidance_v_set_guided_z(TAKE_OFF_ALTITUDE);
		states_race.attitude_control = FALSE;
		/*guidance_loop_set_velocity(0.0, 0.0);*/
        return;
    }
    if (fabs(stateGetPositionNed_f()->z - desired_altitude)<0.1 && time_primitive > 2)
	{
        // psi1 = stateGetNedToBodyEulers_f()->psi;
        states_race.altitude_is_achieved = 1;
        return;
    }

}




void set_attitude(float desired_theta,float desired_phi)
{
    if(primitive_in_use != SET_ATTITUDE)
    { 
		primitive_in_use = SET_ATTITUDE;
        psi0 = stateGetNedToBodyEulers_f()->psi;
        z0 = stateGetPositionNed_f()->z;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
		states_race.attitude_control = TRUE;
    }
	guidance_loop_set_theta(desired_theta);
	guidance_v_set_guided_z(TAKE_OFF_ALTITUDE);
	guidance_loop_set_phi(desired_phi); 
	guidance_loop_set_heading(psi0);
	/*printf("It is in attitude control mode!!!!!!!!!!!\n");*/
}


void calculate_attitude_average(double * p_theta,double *p_phi,struct acceleration* p_accel)
{
}

struct arc_open_loop_status arc_status;
void drone_model(struct arc_open_loop_status * sta,double radius);




bool arc_open_loop(double radius,double desired_theta,float delta_psi,int flag_right,int flag_height)
{
    if(primitive_in_use != ARC_OPEN_LOOP)
    { 
		primitive_in_use = ARC_OPEN_LOOP;
        psi = stateGetNedToBodyEulers_f()->psi;
		psi0 = psi;
        z0 = stateGetPositionNed_f()->z;
		arc_status.flag_in_arc = TRUE;
		arc_status.x= stateGetPositionNed_f()->x;
		arc_status.y= stateGetPositionNed_f()->y;
		arc_status.z= stateGetPositionNed_f()->z;
		arc_status.phi_cmd = stateGetNedToBodyEulers_f()->phi;
		arc_status.theta_cmd = desired_theta;
        arc_status.psi_cmd= stateGetNedToBodyEulers_f()->psi;
		
		float cruise_speed = 1.8;//1.3;//2.0;
	
	
		arc_status.v_x_f = cruise_speed; 
		arc_status.v_y_f = 0; 
		arc_status.v_z_f = 0;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        /*guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);*/
        guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
		states_race.attitude_control = TRUE;
		arc_status.drag_coef_body_x_1 = -0.53;
		arc_status.drag_coef_body_y_1 = -0.47;
		arc_status.drag_coef_body_z_1 = 0;
		arc_status.drag_coef_body_x_0 = 0.0057;
		arc_status.drag_coef_body_y_0 = -0.025;
		arc_status.drag_coef_body_z_0 = 0;
		arc_status.drag_coef_body_x_2 = 0;
		arc_status.drag_coef_body_y_2 = 0;
		arc_status.drag_coef_body_z_2 = 0;
		race_state.target_heading = psi0+delta_psi;
		if(flag_right == TRUE)
				race_state.target_heading = psi0+delta_psi;
		else
				race_state.target_heading = psi0 - delta_psi;
		if (race_state.target_heading > 3.14)
				race_state.target_heading -= 2*3.14;
		else if(race_state.target_heading < -3.14)
				race_state.target_heading += 2*3.14;
    }

// calculate command needed
// transfer velocity from earth coordinate to body and body fixed coordinate
	double phi = arc_status.phi_cmd;
    double theta = arc_status.theta_cmd;
    double psi = arc_status.psi_cmd;
	// calculate body velocity
	arc_status.v_x_b = cos(theta)*arc_status.v_x_f-
			sin(theta)*arc_status.v_z_f;
	arc_status.v_y_b = sin(phi)*sin(theta)*arc_status.v_x_f+
			cos(phi)*arc_status.v_y_f+
		   sin(phi)*cos(theta)*arc_status.v_z_f;
	arc_status.v_z_b = cos(phi)*sin(theta)*arc_status.v_x_f-
		sin(phi)*arc_status.v_y_f+
		cos(phi)*cos(theta)*arc_status.v_z_f;	

    /*arc_status.v_x_b = cos(theta)*arc_status.v_x_f;*/
    /*arc_status.v_y_b = 0;*/
	/*arc_status.v_z_b = cos(phi)*sin(theta)*arc_status.v_x_f;*/

//  calculate drag in body velocity
	arc_status.drag_x_b = arc_status.drag_coef_body_x_1*arc_status.v_x_b;
	arc_status.drag_y_b = arc_status.drag_coef_body_y_1*arc_status.v_y_b;
	arc_status.drag_z_b = arc_status.drag_coef_body_z_1*arc_status.v_z_b;

//  transfer drag from body coordinate to body fixed coordinate
	arc_status.drag_x_f = cos(theta)*arc_status.drag_x_b+sin(phi)*sin(theta)*arc_status.drag_y_b
			+cos(phi)*sin(theta)*arc_status.drag_z_b;
	arc_status.drag_y_f = cos(phi)*arc_status.drag_y_b-sin(phi)*arc_status.drag_z_b;
	/*arc_status.drag_y_f = 0;*/
	arc_status.drag_z_f = -sin(theta)*arc_status.drag_x_b+sin(phi)*cos(theta)*arc_status.drag_y_b+
			cos(phi)*cos(theta)*arc_status.drag_z_b;


//  calculate command
    if (flag_right == TRUE)
	{
			double d_psi = arc_status.v_x_f/radius;
			arc_status.d_psi = arc_status.v_x_f/radius;
			arc_status.psi_cmd += d_psi/100.0;
			arc_status.theta_cmd = desired_theta;
			arc_status.phi_cmd= atan((arc_status.v_x_f*arc_status.v_x_f/radius-arc_status.drag_y_f)*cos(arc_status.theta_cmd)/
							(9.8+arc_status.drag_z_f));
	}
	else
	{
			arc_status.d_psi = arc_status.v_x_f/radius;
			arc_status.d_psi = -arc_status.d_psi;
			arc_status.psi_cmd += arc_status.d_psi/100.0;
			arc_status.theta_cmd = desired_theta;
			arc_status.phi_cmd= atan((arc_status.v_x_f*arc_status.d_psi-arc_status.drag_y_f)*cos(arc_status.theta_cmd)/
							(9.8+arc_status.drag_z_f));
	}
	/*arc_status.phi_cmd = -atan(-arc_status.v_x_f*d_psi*cos(arc_status.theta_cmd)/9.8);*/
	arc_status.thrust_cmd= (-9.8-arc_status.drag_z_f)/cos(arc_status.phi_cmd)/cos(arc_status.theta_cmd);
	// euler method to predict
	drone_model(&arc_status,radius);

	arc_status.x += arc_status.dx/100.0;
	arc_status.y += arc_status.dy/100.0;
	arc_status.z += arc_status.dz/100.0;
	arc_status.v_x_f += arc_status.dv_x_f/100.0;
	arc_status.v_y_f += arc_status.dv_y_f/100.0;
	arc_status.v_z_f += arc_status.dv_z_f/100.0;
 
	guidance_loop_set_theta(arc_status.theta_cmd);
	guidance_loop_set_phi(arc_status.phi_cmd); 
	guidance_loop_set_heading(arc_status.psi_cmd);
	
	if(flag_height == FALSE)
	{
				set_altitude(open_loop_altitude[race_state.gate_counter]);
	}
	else
	{
			double height = open_loop_altitude[race_state.gate_counter]-0.5*time_primitive;
			/*printf("desired height is %f ----------------\n",height);*/
			/*printf("current height is %f ----------------\n",stateGetPositionNed_f()->z);*/
			set_altitude(height);
	}


	if (fabs(stateGetNedToBodyEulers_f()->psi - race_state.target_heading)< 3.0/180*3.14)
	{
			arc_status.flag_in_arc = FALSE;
			if(two_arc_st.flag_first_arc == FALSE)
			{
					race_state.gate_counter++;
					race_state.flag_in_open_loop = FALSE;
					initialize_EKF();
			}
			else
			{

					/*printf("first part is finished without adding counter\n");*/
					primitive_in_use = NO_PRIMITIVE;
			}
			return 1;
	}
	else
	{
			return 0;
	}
}

void drone_model(struct arc_open_loop_status* sta,double radius)
{
		double phi = sta->phi_cmd;
		double theta = sta->theta_cmd;
		double psi = sta->psi_cmd;

		sta->dx = cos(psi)*sta->v_x_f-sin(psi)*sta->v_y_f;
		sta->dy = sin(psi)*sta->v_x_f+cos(psi)*sta->v_y_f;
		sta->dz = sta->v_z_f;
		
		double T = arc_status.thrust_cmd;
		/*double T = -9.8/cos(theta)/cos(phi);*/
		sta->dv_x_f = cos(phi)*sin(theta)*T+sta->drag_x_f+sta->v_y_f*sta->d_psi;
		/*sta->dv_x_f = cos(phi)*sin(theta)*T+sta->drag_x_f;*/
		sta->dv_y_f = -sin(phi)*T+sta->drag_y_f-sta->v_x_f*sta->d_psi; 	
		sta->dv_z_f = 9.8+cos(phi)*cos(theta)*T+sta->drag_z_f; 	
}


double sum_phi;
double sum_theta;
int trim_att_counter;
double trim_phi;
double trim_theta;
bool hover_at_origin()
{
    if(primitive_in_use != HOVER_AT_ORIGIN)
    { 
		primitive_in_use = HOVER_AT_ORIGIN;
        psi0 = stateGetNedToBodyEulers_f()->psi;
        z0 = stateGetPositionNed_f()->z;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
		guidance_h_set_guided_pos(0.0,0.0);
		guidance_h_set_guided_heading(0.0);
		guidance_v_set_guided_z(TAKE_OFF_ALTITUDE);
		sum_phi = 0;
		sum_theta = 0;
		trim_att_counter = 0;
    }
 float current_x = stateGetPositionNed_f()->x;
 float current_y = stateGetPositionNed_f()->y;
 if (time_primitive >10.0)
 {
		 sum_phi += stateGetNedToBodyEulers_f()->phi;
		 sum_theta += stateGetNedToBodyEulers_f()->theta;
		 trim_att_counter ++;
		 trim_phi = sum_phi /trim_att_counter;
		 trim_theta = sum_theta /trim_att_counter;
 }
 if (sqrt(current_x*current_x + current_y*current_y)<0.2&&time_primitive > 20)
 {
		 return 1;
 }
 else
 {
		 /*printf("Trim phi = %f,\n",trim_phi/3.14*180);*/
		 /*printf("Trim theta= %f,\n",trim_theta/3.14*180);*/
		 return 0;
 }
}


struct zigzag_open_loop_status zigzag_status;

bool zigzag_open_loop(double desired_y,double desired_theta,float max_roll,float break_angle,float break_time,int flag_zigzag_right,int flag_break)
{
    if(primitive_in_use != ZIGZAG_OPEN_LOOP)
    { 
		primitive_in_use = ZIGZAG_OPEN_LOOP;
        zigzag_status.psi0 = stateGetNedToBodyEulers_f()->psi;
        z0 = stateGetPositionNed_f()->z;
		zigzag_status.flag_in_zigzag = TRUE;
		zigzag_status.position.x= 0.0;
		zigzag_status.position.y= 0.0;
		zigzag_status.position.z= stateGetPositionNed_f()->z;
		zigzag_status.phi_cmd = stateGetNedToBodyEulers_f()->phi;
		zigzag_status.theta_cmd = desired_theta;
        zigzag_status.psi_cmd= stateGetNedToBodyEulers_f()->psi;
        counter_primitive = 0;
        time_primitive = 0;
        guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
        guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
		states_race.attitude_control = TRUE;
		zigzag_status.drag_coef.x = -0.55;
		zigzag_status.drag_coef.y = -0.56;
		zigzag_status.drag_coef.z = 0;
		zigzag_status.drag_coef_angular_rate.x = -0.08;
		zigzag_status.drag_coef_angular_rate.y = 0.1;
        zigzag_status.velocity.x = 1.8;
        zigzag_status.velocity.y = 0.0;
        zigzag_status.velocity.z = 0.0;
		zigzag_status.flag_break = TRUE;
		counter_temp3 = 0;
		time_temp3 = 0;
    }

	if (flag_break == TRUE && zigzag_status.flag_break == TRUE)
	{
			guidance_loop_set_theta(break_angle);
			guidance_loop_set_phi(0.0);
			guidance_loop_set_heading(zigzag_status.psi_cmd);
			guidance_v_set_guided_z(TAKE_OFF_ALTITUDE);
			/*printf("BBBBBBBBBBBBBBBBB\n");*/
			/*printf("break_time is %f\n",break_time);*/
			/*printf("time_temp3 is %f\n",time_temp3);*/
			if(time_temp3 > break_time)
			{
					zigzag_status.flag_break = FALSE;
			}
			return 0;
	}
	/*printf("CCCCCCCCCCCCCCCCC\n");*/
// calculate command needed
// transfer velocity from earth coordinate to body and body fixed coordinate
	// calculate body velocity
    zigzag_status.eulers.phi = stateGetNedToBodyEulers_f()->phi;
    zigzag_status.eulers.theta = stateGetNedToBodyEulers_f()->theta;
    zigzag_status.eulers.psi = stateGetNedToBodyEulers_f()->psi-zigzag_status.psi0;
    float_rmat_of_eulers_321(&zigzag_status.R_E_B,&zigzag_status.eulers); 
    float_rmat_vmult(&zigzag_status.body_velocity,&zigzag_status.R_E_B,&zigzag_status.velocity);

//  calculate drag in body velocity

	zigzag_status.drag_b.x = zigzag_status.drag_coef.x*zigzag_status.body_velocity.x;
	zigzag_status.drag_b.y = zigzag_status.drag_coef.y*zigzag_status.body_velocity.y;
	zigzag_status.drag_b.z = zigzag_status.drag_coef.z*zigzag_status.body_velocity.z;

//  transfer drag from body coordinate to earth coordinate

	float_rmat_transp_vmult(&zigzag_status.drag_e,&zigzag_status.R_E_B,&zigzag_status.drag_b);

//  predict trajectory

	struct FloatVect3 thrust_b; 
	struct FloatVect3 thrust_e; 
	struct FloatVect3 gravity;
	thrust_b.x = 0;
	thrust_b.y = 0;
	thrust_b.z = -9.8/cos(zigzag_status.eulers.phi)/cos(zigzag_status.eulers.theta);

	gravity.x = 0;
	gravity.y = 0;
	gravity.z = 9.8;

    float_rmat_transp_vmult(&thrust_e,&zigzag_status.R_E_B,&thrust_b);

	float_vect_sum(&zigzag_status.d_velocity.x,&gravity.x,&thrust_e.x,3);
	float_vect_add(&zigzag_status.d_velocity.x,&zigzag_status.drag_e.x,3);
	float_vect_copy(&zigzag_status.d_position.x,&zigzag_status.velocity.x,3);

//  intergrate position and velocity
    float_vect3_integrate_fi(&zigzag_status.position,&zigzag_status.d_position,1.0/100.0);
    float_vect3_integrate_fi(&zigzag_status.velocity,&zigzag_status.d_velocity,1.0/100.0);

	// calculte angle command
	guidance_loop_set_theta(desired_theta);
	if(flag_zigzag_right == 1)
	{
			if (zigzag_status.position.y < desired_y/2.0)
			{
					guidance_loop_set_phi(max_roll); 
					/*printf("max_roll angle is %f\n",max_roll);*/
					/*printf("predicted position y == %f\n",zigzag_status.position.y);*/
			}
			else
			{
					guidance_loop_set_phi(-max_roll); 
					/*printf("EEEEEEEEEEEEEEEEE\n");*/
					/*printf("predicted position y == %f\n",zigzag_status.position.y);*/
			}
	}
	else
	{
			if (zigzag_status.position.y > desired_y/2.0)
			{
					guidance_loop_set_phi(-max_roll); 
					/*printf("predicted position y == %f\n",zigzag_status.position.y);*/
			}
			else
			{
					guidance_loop_set_phi(max_roll); 
					/*printf("predicted position y == %f\n",zigzag_status.position.y);*/
			}
	}
	guidance_loop_set_heading(zigzag_status.psi_cmd);
	guidance_v_set_guided_z(TAKE_OFF_ALTITUDE);

	if (fabs(zigzag_status.position.y-desired_y)<0.3)
	{
			zigzag_status.flag_in_zigzag = FALSE;
			race_state.gate_counter++;
			initialize_EKF();
			return 1;
	}
	else
	{
			return 0;
	}
}

float previous_error_y ;
float previous_D_term;
bool go_through_gate(float theta)
{
		if(primitive_in_use != GO_THROUGH_GATE)
		{

				primitive_in_use = GO_THROUGH_GATE;
				psi0 = stateGetNedToBodyEulers_f()->psi;
				z0 = stateGetPositionNed_f()->z;
				counter_primitive = 0;
				time_primitive = 0;
				guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
				//guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
				race_state.flag_in_open_loop = FALSE;
				states_race.attitude_control = TRUE;
				prev_D_term = 0.0;
				previous_error_y = 0.0;
				counter_temp2 = 0;
				time_temp2 = 0;
		}

		if(break_time[race_state.gate_counter]!=0 && time_temp2 <break_time[race_state.gate_counter])
		{
				/*printf("BREAK!!!!!!!!!!!!!!!!\n");*/
				/*printf("time temp3 = %f break_time is %f\n",time_temp2,break_time[race_state.gate_counter]);*/
				guidance_loop_set_theta(5.0/180*3.14);
				guidance_loop_set_phi(0.0); 
				guidance_loop_set_heading(psi0);
				guidance_v_set_guided_z(gate_altitude[race_state.gate_counter]);
				return FALSE;
		}
       		
		float error_y = -kf_pos_y;
		
		/*printf("go through function is called\n");*/
		
		
		float D_term = error_y-previous_error_y;
		race_state.sum_y_error += error_y;
		float desired_phi = KP_Y*error_y+KD_Y*((D_term+prev_D_term)/2.0)*100+KI_Y*race_state.sum_y_error;
		/*printf("intergration item is %f\n",KI_Y*race_state.sum_y_error/3.14*180);*/
		previous_error_y = error_y;
		previous_D_term = D_term;
		if(desired_phi > MAX_PHI)
				desired_phi = MAX_PHI;
		else if (desired_phi < -MAX_PHI)
				desired_phi = -MAX_PHI;

		/*printf("desired_phi is %f\n",desired_phi/3.14*180);*/
		/*printf("error y is %f\n",error_y);*/
		guidance_loop_set_theta(theta);
		guidance_loop_set_phi(desired_phi); 
		guidance_loop_set_heading(psi0);
		/*guidance_v_set_guided_z(gate_altitude[race_state.gate_counter]);*/
		set_altitude(gate_altitude[race_state.gate_counter]);	
		if (fabs(kf_pos_x - turn_point[race_state.gate_counter])<0.05)
		{
				return TRUE;
				/*printf("exit go through mode\n");*/
		}
		else
		{
				return FALSE;
		}
}

bool go_straight_test(float time,float desired_theta)
{
		if(primitive_in_use != GO_STRAIGHT_TEST)
		{
				primitive_in_use = GO_STRAIGHT_TEST;
				psi0 = stateGetNedToBodyEulers_f()->psi;
				z0 = stateGetPositionNed_f()->z;
				counter_primitive = 0;
				time_primitive = 0;
				guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
				//guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
				race_state.flag_in_open_loop = TRUE;
		}
		guidance_loop_set_theta(desired_theta);
		guidance_loop_set_phi(0.0); 
		guidance_loop_set_heading(psi0);
		/*guidance_v_set_guided_z(-1.5);*/
		set_altitude(-1.5);	
		if(time_primitive > time)
				return TRUE;
		else
				return FALSE;
}


struct two_arc_status two_arc_st;
bool two_arcs_open_loop(float radius,float desired_theta, int flag_right,float delta_psi)
{
		if(two_arc_st.flag_in_two_arc_mode == FALSE)
		{
				two_arc_st.flag_in_two_arc_mode = TRUE;
				two_arc_st.flag_first_arc = TRUE;
				two_arc_st.flag_second_arc = FALSE;
				two_arc_st.flag_straight = FALSE;
		}
		
	
		if(two_arc_st.flag_first_arc == TRUE)
		{

				if(flag_right == 1)
				{
						/*printf("AAAAAAAAAAAAAAAA radius is %f!\n",radius);*/
					if(arc_open_loop(radius,desired_theta,delta_psi,1,0))
					{
							two_arc_st.flag_first_arc = FALSE;
							two_arc_st.flag_straight = TRUE;
							psi0 = stateGetNedToBodyEulers_f()->psi;
							counter_temp3 = 0;
							time_temp3 = 0.0;
							primitive_in_use = NO_PRIMITIVE;
							/*two_arc_st.flag_second_arc = TRUE;*/
					}
							return FALSE;
				}
				else
				{

						/*printf("BBBBBBBBBBBBBBBBBBBB\n!");*/
					if(arc_open_loop(radius,desired_theta,delta_psi,-1,0))
					{
							two_arc_st.flag_first_arc = FALSE;
							two_arc_st.flag_straight = TRUE;
							psi0 = stateGetNedToBodyEulers_f()->psi;
							counter_temp3 = 0;
							time_temp3 = 0.0;
							primitive_in_use = NO_PRIMITIVE;
					}
							return FALSE;
				}

		}

		if(two_arc_st.flag_straight == TRUE)
		{
				guidance_loop_set_theta(desired_theta);
				guidance_loop_set_phi(0.0); 
				guidance_loop_set_heading(psi0);
				/*guidance_v_set_guided_z(open_loop_altitude[race_state.gate_counter]);*/
				set_altitude(open_loop_altitude[race_state.gate_counter]);	
				/*printf("!!!!!!!!!!!!!\n");*/
				if(time_temp3 > 0.3)
				{
						two_arc_st.flag_straight = FALSE;
						two_arc_st.flag_second_arc= TRUE;
						primitive_in_use = NO_PRIMITIVE;
				}
						return FALSE;
		}

		if(two_arc_st.flag_second_arc== TRUE)
		{

				if(flag_right == 1)
				{
						/*printf("CCCCCCCCCCCCCCCCCCC  radius is %f\n!",radius);*/
					if(arc_open_loop(radius,desired_theta,delta_psi,-1,0))
					{
							two_arc_st.flag_first_arc = FALSE;
							two_arc_st.flag_second_arc = FALSE;
							two_arc_st.flag_in_two_arc_mode = FALSE;
							return TRUE;
					}
					else
							return FALSE;
							
				}
				else
				{
						/*printf("DDDDDDDDDDDDDDDDDD\n!");*/
					if(arc_open_loop(radius,desired_theta,delta_psi,1,0))
					{
							two_arc_st.flag_first_arc = FALSE;
							two_arc_st.flag_second_arc = FALSE;
							two_arc_st.flag_in_two_arc_mode = FALSE;
							initialize_EKF();
							/*printf("EEEEEEEEEEEEEEEEEE\n");*/
							return TRUE;
					}
					else
							return FALSE;
				}
		}
}


bool zigzag_2(float zig_zag_break_time,float max_roll,float distance_y)
{
		if(primitive_in_use != ZIGZAG_2)
		{
				primitive_in_use = ZIGZAG_2;
				psi0 = stateGetNedToBodyEulers_f()->psi;
				z0 = stateGetPositionNed_f()->z;
				counter_primitive = 0;
				time_primitive = 0;
				guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
				counter_temp2 = 0;
				time_temp2 = 0;
				race_state.flag_in_open_loop = FALSE;
				initialize_EKF();
		}

		if(time_temp2 < zig_zag_break_time)
		{
				guidance_loop_set_theta(10.0/180*3.14);
				guidance_loop_set_phi(0.0); 
				guidance_loop_set_heading(psi0);
				/*guidance_v_set_guided_z(gate_altitude[race_state.gate_counter]);*/
				set_altitude(gate_altitude[race_state.gate_counter]);
				return FALSE;
		}

		/*printf("zigzag_2 desired_y is %f\n",distance_y);*/
		/*printf("Kalman filter y disdance is %f\n",kf_pos_y);*/
		if(fabs(kf_pos_y)<fabs(distance_y)/2.0)
		{
				if (distance_y > 0)
				{
						guidance_loop_set_theta(0.0/180*3.14);
						guidance_loop_set_phi(max_roll); 
						guidance_loop_set_heading(psi0);
						/*guidance_v_set_guided_z(gate_altitude[race_state.gate_counter]);*/
						set_altitude(open_loop_altitude[race_state.gate_counter]);	
						return FALSE;
				}
				else
				{
						guidance_loop_set_theta(0.0/180*3.14);
						guidance_loop_set_phi(-max_roll); 
						guidance_loop_set_heading(psi0);
						/*guidance_v_set_guided_z(gate_altitude[race_state.gate_counter]);*/
						set_altitude(open_loop_altitude[race_state.gate_counter]);	
						return FALSE;
				}
		}
		else
		{
				if (distance_y > 0)
				{
						guidance_loop_set_theta(0.0/180*3.14);
						guidance_loop_set_phi(-max_roll/3.0); 
						guidance_loop_set_heading(psi0);
						set_altitude(open_loop_altitude[race_state.gate_counter]);	
						/*guidance_v_set_guided_z(open_loop_altitude[race_state.gate_counter]);*/
						if(fabs(distance_y-kf_pos_y)<0.6)
						{
								race_state.gate_counter++;
								race_state.flag_in_open_loop = FALSE;
								initialize_EKF();
								return TRUE;
						}
						else
						{
								return FALSE;
						}
				}
				else
				{
						guidance_loop_set_theta(0.0/180*3.14);
						guidance_loop_set_phi(max_roll/3.0); 
						guidance_loop_set_heading(psi0);
						/*guidance_v_set_guided_z(open_loop_altitude[race_state.gate_counter]);*/
						set_altitude(open_loop_altitude[race_state.gate_counter]);
						if(fabs(distance_y-kf_pos_y)<0.6)
						{
								race_state.gate_counter++;
								race_state.flag_in_open_loop = FALSE;
								initialize_EKF();
								return TRUE;
						}
						else
						{
								return FALSE;
						}
				}

		}

}


bool go_through_open_gate(double desired_theta, double desired_x)
{
		if(primitive_in_use != GO_THROUGH_OPEN_GATE)
		{
				primitive_in_use = GO_THROUGH_OPEN_GATE;
				psi0 = stateGetNedToBodyEulers_f()->psi;
				z0 = stateGetPositionNed_f()->z;
				counter_primitive = 0;
				time_primitive = 0;
				guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
		}
	float error_y;

	if(kf_pos_y > 0){//drone is on the right of the gate, and should stay at the right
		error_y = -(kf_pos_y-0.7);//-kf_pos_y;
	}else{//drone on the left, and should stay there
		error_y = -(kf_pos_y+0.7);
		//printf("error_y:%f\n",error_y);
	}


	float D_term = error_y-previous_error_y;
	race_state.sum_y_error += error_y;
	float desired_phi = KP_Y*error_y+KD_Y*((D_term+prev_D_term)/2.0)*100+KI_Y*race_state.sum_y_error;
	/*printf("intergration item is %f\n",KI_Y*race_state.sum_y_error/3.14*180);*/
	previous_error_y = error_y;
	previous_D_term = D_term;
	if(desired_phi > MAX_PHI)
		desired_phi = MAX_PHI;
	else if (desired_phi < -MAX_PHI)
		desired_phi = -MAX_PHI;

	/*printf("desired_phi is %f\n",desired_phi/3.14*180);*/
	/*printf("error y is %f\n",error_y);*/
	guidance_loop_set_theta(desired_theta);
	guidance_loop_set_phi(desired_phi);
	guidance_loop_set_heading(psi0);
	/*guidance_v_set_guided_z(TAKE_OFF_ALTITUDE);*/
	set_altitude(TAKE_OFF_ALTITUDE);
	
	if(/*kf_pos_x > desired_x ||*/ time_primitive > 10.5)
			return TRUE;
	else
			return FALSE;
}


bool stop_turn(float stop_turn_break_time, float delta_psi)
{
		if(primitive_in_use != STOP_TURN)
		{
				primitive_in_use = STOP_TURN;
				psi0 = stateGetNedToBodyEulers_f()->psi;
				z0 = stateGetPositionNed_f()->z;
				counter_primitive = 0;
				time_primitive = 0;
				guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
				race_state.flag_in_open_loop = TRUE;
				race_state.target_heading = psi0+delta_psi;
				if (race_state.target_heading > 3.14)
						race_state.target_heading -= 2*3.14;
				else if(race_state.target_heading < -3.14)
						race_state.target_heading += 2*3.14;
		}
		if (time_primitive < stop_turn_break_time)
		{
				guidance_loop_set_theta(10.0/180*3.14);
				guidance_loop_set_phi(0.0); 
				guidance_loop_set_heading(psi0);
				/*guidance_v_set_guided_z(gate_altitude[race_state.gate_counter]);*/
				set_altitude(gate_altitude[race_state.gate_counter]);
				return FALSE;
		}

		guidance_loop_set_theta(0.0/180*3.14);
		guidance_loop_set_phi(0.0); 
		guidance_loop_set_heading(race_state.target_heading);
		/*guidance_v_set_guided_z(gate_altitude[race_state.gate_counter]);*/
		set_altitude(gate_altitude[race_state.gate_counter]);
	if (fabs(stateGetNedToBodyEulers_f()->psi - race_state.target_heading)< 1.0/180*3.14)
	{
			race_state.gate_counter++;
			race_state.flag_in_open_loop = FALSE;
			initialize_EKF();
			return TRUE;
	}
	else
	{
			return FALSE;
	}


}



bool take_off_fast(void)
{
		if (primitive_in_use != TAKE_OFF_FAST)
		{
		    first_stretch_start();
				psi_startup = stateGetNedToBodyEulers_f()->psi;
				primitive_in_use = TAKE_OFF_FAST;
				counter_primitive = 0;
				time_primitive = 0;
				tf_status.flag_open_loop = FALSE;
				tf_status.flag_climb_mode = FALSE;
				tf_status.flag_hover_mode = FALSE;
				guidance_h_mode_changed(GUIDANCE_H_MODE_MODULE);
				states_race.attitude_control = TRUE;
				states_race.altitude_is_achieved = 0;
				tf_status.flag_open_loop = TRUE;
				tf_status.take_off_altitude = TAKE_OFF_ALTITUDE;
				tf_status.altitude_counter = 0;
				tf_status.sum_altitude = 0.0;
				tf_status.ave_altitude = 0.0;
				guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);  // vertical module should be called!
				set_altitude(TAKE_OFF_ALTITUDE);
				tf_status.flag_ekf_initialized = FALSE;
		}
				guidance_loop_set_theta(-15/57.6);
				guidance_loop_set_phi(0);
				guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);  // vertical module should be called!
				if (time_primitive > 8.5)
				{
						tf_status.flag_ekf_initialized = TRUE;
						race_state.flag_in_open_loop = FALSE;
						initialize_EKF();
						return TRUE;

				}
				else
				{
						return FALSE;
				}
}
