/*
 * Copyright (C) 2014 Hann Woei Ho
 *					  Guido de Croon
 *					  
 * Code based on the article:
 * "Optic-flow based slope estimation for autonomous landing", 
 * de Croon, G.C.H.E., and Ho, H.W., and De Wagter, C., and van Kampen, E., and Remes B., and Chu, Q.P., 
 * in the International Journal of Micro Air Vehicles, Volume 5, Number 4, pages 287 – 297, (2013)				  
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/*
 * @file paparazzi/sw/ext/ardrone2_vision/cv/opticflow/optic_flow_gdc.c
 * @brief optical-flow based hovering for Parrot AR.Drone 2.0
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "defs_and_types.h"
#include "linear_flow_fit.h"

// Is this still necessary?
#define MAX_COUNT_PT 50

#define MIN_SAMPLES_FIT 3
#define NO_FIT 0

/**
 * Analyze a linear flow field, retrieving information such as divergence, surface roughness, focus of expansion, etc.
 * @param[out] outcome If 0, there were too few vectors for a fit. If 1, the fit was successful.
 * @param[in] flow_t* vectors The optical flow vectors
 * @param[in] count The number of optical flow vectors
 * @param[in] error_threshold Error used to determine inliers / outliers. 
 * @param[in] n_iterations Number of RANSAC iterations.
 * @param[in] n_samples Number of samples used for a single fit (min. 3).
 * @param[in] im_width Image width in pixels
 * @param[in] im_height Image height in pixels
 * @param[in] slope_x* Slope of the surface in x-direction - given sufficient lateral motion.
 * @param[in] slope_y* Slope of the surface in y-direction - given sufficient lateral motion.
 * @param[in] surface_roughness* The error of the linear fit is a measure of surface roughness.
 * @param[in] focus_of_expansion_x* Image x-coordinate of the focus of expansion (contraction).
 * @param[in] focus_of_expansion_y* Image y-coordinate of the focus of expansion (contraction).
 * @param[in] relative_velocity_x* Relative velocity in x-direction, i.e., vx / z, where z is the depth in direction of the camera's principal axis.
 * @param[in] relative_velocity_y* Relative velocity in y-direction, i.e., vy / z, where z is the depth in direction of the camera's principal axis.
 * @param[in] relative_velocity_z* Relative velocity in z-direction, i.e., vz / z, where z is the depth in direction of the camera's principal axis.
 * @param[in] time_to_contact* Basically, 1 / relative_velocity_z.
 * @param[in] divergence* Basically, relative_velocity_z. Actual divergence of a 2D flow field is 2 * relative_velocity_z.
 * @param[in] fit_error* Error of the fit (same as surface roughness).
 * @param[in] n_inliers_u Number of inliers in the horizontal flow fit.
 * @param[in] n_inliers_v Number of inliers in the vertical flow fit.
 */
int analyze_linear_flow_field(struct flow_t* vectors, int count, float error_threshold, int n_iterations, int n_samples, int im_width, int im_height, float *slope_x, float *slope_y, float *surface_roughness, float *focus_of_expansion_x, float *focus_of_expansion_y, float *relative_velocity_x, float *relative_velocity_y, float *relative_velocity_z, float *time_to_contact, float* divergence, float *fit_error, int *n_inliers_u, int *n_inliers_v)
{
	// Are there enough flow vectors to perform a fit?		
	if(count < MIN_SAMPLES_FIT)
	{
		return NO_FIT;
	}

		float pu[3], pv[3], min_error_u, min_error_v;

		fitLinearFlowField(pu, pv, divergence_error, vectors, count, n_samples, &min_error_u, &min_error_v, n_iterations, error_threshold, n_inlier_minu, n_inlier_minv);

		extractInformationFromLinearFlowField(divergence, mean_tti, median_tti, d_heading, d_pitch, pu, pv, imW, imH, DIV_FILTER, FPS);

		slopeEstimation(z_x, z_y, three_dimensionality, POE_x, POE_y, *d_heading, *d_pitch, pu, pv, min_error_u, min_error_v);

}


void fitLinearFlowField(float* pu, float* pv, float* divergence_error, struct flow_t* vectors, int count, int n_samples, float* min_error_u, float* min_error_v, int n_iterations, float error_threshold, int *n_inlier_minu, int *n_inlier_minv)
{

	// ensure that n_samples is high enough to ensure a result for a single fit:
	n_samples = (n_samples < MIN_SAMPLES_FIT) ? MIN_SAMPLES_FIT : n_samples;

		//	printf("count=%d, n_sample=%d, n_iterations=%d, error_threshold=%f\n",count,n_samples,n_iterations,error_threshold);
		//	for (int i=0; i<count;i++) {
		//		printf("%d_%d, ",dx[i],dy[i]);
		//	}
		//	printf("\n");
	
		int *sample_indices;
		float **A, *bu, *bv, **AA, *bu_all, *bv_all;
		sample_indices =(int *) calloc(n_samples,sizeof(int));
		A = (float **) calloc(n_samples,sizeof(float*));// A1 is a N x 3 matrix with rows [x, y, 1]
		bu = (float *) calloc(n_samples,sizeof(float)); // bu is a N x 1 vector with elements dx (or dy)
		bv = (float *) calloc(n_samples,sizeof(float)); // bv is a N x 1 vector with elements dx (or dy)
		AA = (float **) calloc(count,sizeof(float*));   // AA contains all points with rows [x, y, 1]
		bu_all = (float *) calloc(count,sizeof(float)); // bu is a N x 1 vector with elements dx (or dy)
		bv_all = (float *) calloc(count,sizeof(float)); // bv is a N x 1 vector with elements dx (or dy)
		int si, add_si, p, i_rand, sam;
		for(sam = 0; sam < n_samples; sam++) A[sam] = (float *) calloc(3,sizeof(float));
		pu[0] = 0.0f; pu[1] = 0.0f; pu[2] = 0.0f;
		pv[0] = 0.0f; pv[1] = 0.0f; pv[2] = 0.0f;
		float * PU, * errors_pu, * PV, * errors_pv;
		int * n_inliers_pu, * n_inliers_pv;
		PU = (float *) calloc(n_iterations*3,sizeof(float));
		PV = (float *) calloc(n_iterations*3,sizeof(float));
		errors_pu = (float *) calloc(n_iterations,sizeof(float));
		errors_pv = (float *) calloc(n_iterations,sizeof(float));
		n_inliers_pu = (int *) calloc(n_iterations,sizeof(int));
		n_inliers_pv = (int *) calloc(n_iterations,sizeof(int));

		float *bb, *C;
		bb = (float *) calloc(count,sizeof(float));
		C = (float *) calloc(count,sizeof(float));

		// initialize matrices and vectors for the full point set problem:
		// this is used for determining inliers
		for(sam = 0; sam < count; sam++)
		{
			AA[sam] = (float *) calloc(3,sizeof(float));
			AA[sam][0] = (float) vectors[sam].pos.x;
			AA[sam][1] = (float) vectors[sam].pos.y;
			AA[sam][2] = 1.0f;
			bu_all[sam] = (float) vectors[sam].flow_x;
			bv_all[sam] = (float) vectors[sam].flow_y;
		}

		// ***************
		// perform RANSAC:
		// ***************
		
		int it, ii;
		for(it = 0; it < n_iterations; it++)
		{
			// select a random sample of n_sample points:
			memset(sample_indices, 0, n_samples*sizeof(int));
			i_rand = 0;

			// sampling without replacement:
			while(i_rand < n_samples)
			{
				si = rand() % count;
				add_si = 1;
				for(ii = 0; ii < i_rand; ii++)
				{
					if(sample_indices[ii] == si) add_si = 0;
				}
				if(add_si)
				{
					sample_indices[i_rand] = si;
					i_rand ++;
				}
			}

			// Setup the system:
			for(sam = 0; sam < n_samples; sam++)
			{
				A[sam][0] = (float) vectors[sample_indices[sam]].pos.x;
				A[sam][1] = (float) vectors[sample_indices[sam]].pos.y;
				A[sam][2] = 1.0f;
				bu[sam] = (float) vectors[sample_indices[sam]].flow_x;
				bv[sam] = (float) vectors[sample_indices[sam]].flow_y;
				//printf("%d,%d,%d,%d,%d\n",A[sam][0],A[sam][1],A[sam][2],bu[sam],bv[sam]);
			}

			// Solve the small system:

			// for horizontal flow:
			svdSolve(pu, A, n_samples, 3, bu);
			PU[it*3] = pu[0];
			PU[it*3+1] = pu[1];
			PU[it*3+2] = pu[2];

			// for vertical flow:
			svdSolve(pv, A, n_samples, 3, bv);
			PV[it*3] = pv[0];
			PV[it*3+1] = pv[1];
			PV[it*3+2] = pv[2];

			// count inliers and determine their error on all points:
			errors_pu[it] = 0;
			errors_pv[it] = 0;
			n_inliers_pu[it] = 0;
			n_inliers_pv[it] = 0;

			// for horizontal flow:
			MatVVMul(bb, AA, pu, 3, count);
			float scaleM;
			scaleM = -1.0;
			ScaleAdd(C, bb, scaleM, bu_all, 1, count);

			for(p = 0; p < count; p++)
			{
				if(C[p] < error_threshold)
				{
					errors_pu[it] += abs(C[p]);
					n_inliers_pu[it]++;
				}
				else
				{
					errors_pu[it] += error_threshold;
				}
			}
			// for vertical flow:
			MatVVMul(bb, AA, pv, 3, count);
			ScaleAdd(C, bb, scaleM, bv_all, 1, count);

			for(p = 0; p < count; p++)
			{
				if(C[p] < error_threshold)
				{
					errors_pv[it] += abs(C[p]);
					n_inliers_pv[it]++;
				}
				else
				{
					errors_pv[it] += error_threshold;
				}
			}
		}

		// select the parameters with lowest error:
		// for horizontal flow:
		int param;
		int min_ind = 0;
		*min_error_u = (float)errors_pu[0];
		for(it = 1; it < n_iterations; it++)
		{
			if(errors_pu[it] < *min_error_u)
			{
				*min_error_u = (float)errors_pu[it];
				min_ind = it;
			}
		}
		for(param = 0; param < 3; param++)
		{
			pu[param] = PU[min_ind*3+param];
		}
		*n_inlier_minu = n_inliers_pu[min_ind];
		
		// for vertical flow:
		min_ind = 0;
		*min_error_v = (float)errors_pv[0];

		for(it = 0; it < n_iterations; it++)
		{
			if(errors_pv[it] < *min_error_v)
			{
				*min_error_v = (float)errors_pv[it];
				min_ind = it;
			}
		}
		for(param = 0; param < 3; param++)
		{
			pv[param] = PV[min_ind*3+param];
		}		
		*n_inlier_minv = n_inliers_pv[min_ind];

		// error has to be determined on the entire set:
		MatVVMul(bb, AA, pu, 3, count);
		float scaleM;
		scaleM = -1.0;
		ScaleAdd(C, bb, scaleM, bu_all, 1, count);

		*min_error_u = 0;
		for(p = 0; p < count; p++)
		{
			*min_error_u += abs(C[p]);
		}
		MatVVMul(bb, AA, pv, 3, count);
		ScaleAdd(C, bb, scaleM, bv_all, 1, count);

		*min_error_v = 0;
		for(p = 0; p < count; p++)
		{
			*min_error_v += abs(C[p]);
		}
		*divergence_error = (*min_error_u + *min_error_v) / (2 * count);

		// delete allocated dynamic arrays
		for(sam = 0; sam < n_samples; sam++) free(A[sam]);
		for(sam = 0; sam < count; sam++) free(AA[sam]);
		free(A);
		free(PU);
		free(PV);
		free(n_inliers_pu);
		free(n_inliers_pv);
		free(errors_pu);
		free(errors_pv);
		free(bu);
		free(bv);
		free(AA);
		free(bu_all);
		free(bv_all);
		free(bb);
		free(C);
		free(sample_indices);
}

unsigned int mov_block = 15; //default: 30
float div_buf[30];
unsigned int div_point = 0;
float OFS_BUTTER_NUM_1 = 0.0004260;
float OFS_BUTTER_NUM_2 = 0.0008519;
float OFS_BUTTER_NUM_3 = 0.0004260;
float OFS_BUTTER_DEN_2 = -1.9408;
float OFS_BUTTER_DEN_3 = 0.9425;
float ofs_meas_dx_prev = 0.0;
float ofs_meas_dx_prev_prev = 0.0;
float ofs_filter_val_dx_prev = 0.0;
float ofs_filter_val_dx_prev_prev = 0.0;
float temp_divergence = 0.0;

void extractInformationFromLinearFlowField(float *divergence, float *mean_tti, float *median_tti, float *d_heading, float *d_pitch, float* pu, float* pv, int imgWidth, int imgHeight, int *DIV_FILTER, float FPS)
{
		// This method assumes a linear flow field in x- and y- direction according to the formulas:
		// u = pu[0] * x + pu[1] * y + pu[2]
		// v = pv[0] * x + pv[1] * y + pv[2]
		// where u is the horizontal flow at image coordinate (x,y)
		// and v is the vertical flow at image coordinate (x,y)
	
		// divergence:
		*divergence = pu[0] + pv[1];
		
		// minimal measurable divergence:
		float minimal_divergence = 2E-3;
		if(abs(*divergence) > minimal_divergence)
		{
			*mean_tti = 2.0f / *divergence;
			if(FPS > 1E-3) *mean_tti /= FPS;
			else *mean_tti = ((2.0f / minimal_divergence) / FPS); // TODO: precompute this
			*median_tti = *mean_tti; // TODO: remove median tti?
		}
		else
		{
			*mean_tti = ((2.0f / minimal_divergence) / FPS); // TODO: precompute this
			*median_tti = *mean_tti; // TODO: remove median tti?
		}

		// also adjust the divergence to the number of frames:
		*divergence = *divergence * FPS;

		// translation orthogonal to the camera axis:
		// flow in the center of the image:
		*d_heading = -(pu[2] + (imgWidth/2.0f) * pu[0] + (imgHeight/2.0f) * pu[1]); // TODO: why -? 
		*d_pitch = -(pv[2] + (imgWidth/2.0f) * pv[0] + (imgHeight/2.0f) * pv[1]);

		//apply a moving average
		int medianfilter = 1;
		int averagefilter = 0;
		int butterworthfilter = 0;
		int kalmanfilter = 0;
		float div_avg = 0.0f;

		if(averagefilter == 1)
		{
			*DIV_FILTER = 1;
			if (*divergence < 3.0 && *divergence > -3.0) {
				div_buf[div_point] = *divergence;
				div_point = (div_point+1) %mov_block; // index starts from 0 to mov_block
			}

			int im;
			for (im=0;im<mov_block;im++) {
				div_avg+=div_buf[im];
			}
			*divergence = div_avg/ mov_block;
		}
		else if(medianfilter == 1)
		{
			*DIV_FILTER = 2;
			//apply a median filter
			div_buf[div_point] = *divergence;
			div_point = (div_point+1) %15;
			quick_sort(div_buf,15);
			*divergence  = div_buf[8];
		}
		else if(butterworthfilter == 1)
		{
			*DIV_FILTER = 3;
			temp_divergence = *divergence;
			*divergence = OFS_BUTTER_NUM_1* (*divergence) + OFS_BUTTER_NUM_2*ofs_meas_dx_prev+ OFS_BUTTER_NUM_3*ofs_meas_dx_prev_prev- OFS_BUTTER_DEN_2*ofs_filter_val_dx_prev- OFS_BUTTER_DEN_3*ofs_filter_val_dx_prev_prev;
		    ofs_meas_dx_prev_prev = ofs_meas_dx_prev;
		    ofs_meas_dx_prev = temp_divergence;
		    ofs_filter_val_dx_prev_prev = ofs_filter_val_dx_prev;
		    ofs_filter_val_dx_prev = *divergence;
		}
		else if(kalmanfilter == 1)
		{
			*DIV_FILTER = 4;
			// TODO: implement Kalman filter
		}
}

void slopeEstimation(float *z_x, float *z_y, float *three_dimensionality, float *POE_x, float *POE_y, float d_heading, float d_pitch, float* pu, float* pv, float min_error_u, float min_error_v)
{
	float v_prop_x, v_prop_y, threshold_slope, eta;

	// extract proportional velocities / inclination from flow field:
	v_prop_x  = d_heading;
	v_prop_y = d_pitch;
	threshold_slope = 1.0;
	eta = 0.002;
	if(abs(pv[1]) < eta && abs(v_prop_y) < threshold_slope && abs(v_prop_x) >= 2* threshold_slope)
	{
		// there is not enough vertical motion, but also no forward motion:
		*z_x = pu[0] / v_prop_x;
	}
	else if(abs(v_prop_y) >= 2 * threshold_slope)
	{
		// there is sufficient vertical motion:
		*z_x = pv[0] / v_prop_y;
	}
	else
	{
		// there may be forward motion, then we can do a quadratic fit:
		*z_x = 0.0f;
	}

	*three_dimensionality = min_error_v + min_error_u;

	if(abs(pu[0]) < eta && abs(v_prop_x) < threshold_slope && abs(v_prop_y) >= 2*threshold_slope)
	{
		// there is little horizontal movement, but also no forward motion, and sufficient vertical motion:
		*z_y = pv[1] / v_prop_y;
	}
	else if(abs(v_prop_x) >= 2*threshold_slope)
	{
		// there is sufficient horizontal motion:
		*z_y = pu[1] / v_prop_x;
	}
	else
	{
		// there could be forward motion, then we can do a quadratic fit:
		*z_y = 0.0f;
	}

	// Focus of Expansion:
	// the flow planes intersect the flow=0 plane in a line
	// the FoE is the point where these 2 lines intersect (flow = (0,0))
	// x:
	float denominator = pv[0]*pu[1] - pu[0]*pv[1];
	if(abs(denominator) > 1E-5)
	{
		*POE_x = ((pu[2]*pv[1] - pv[2] * pu[1]) / denominator);
	}
	else *POE_x = 0.0f;
	// y:
	denominator = pu[1];
	if(abs(denominator) > 1E-5)
	{
		*POE_y = (-(pu[0] * *POE_x + pu[2]) / denominator);
	}
	else *POE_y = 0.0f;
}

