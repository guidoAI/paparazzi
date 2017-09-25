
// Own header
#include "modules/computer_vision/lib/vision/closed_gate_processing.h"
#include "modules/computer_vision/snake_gate_detection.h"
#include <stdio.h>
#include "modules/computer_vision/lib/vision/image.h"
#include <stdlib.h>
#include "subsystems/datalink/telemetry.h"
#include "subsystems/imu.h"
//#include "modules/computer_vision/lib/vision/gate_detection.h"
#include "modules/computer_vision/lib/vision/gate_detection_free.h"
#include "modules/computer_vision/lib/vision/gate_corner_refine.h"
#include "state.h"
#include "modules/computer_vision/opticflow/opticflow_calculator.h"
#include "modules/computer_vision/opticflow/opticflow_calculator.h"
#include "modules/state_autonomous_race/state_autonomous_race.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_clock.h"
#include "modules/flight_plan_in_guided_mode/flight_plan_in_guided_mode.h"
#include "modules/state_autonomous_race/state_autonomous_race.h"
#include "modules/computer_vision/lib/vision/qr_code_recognition.h"
#include "modules/computer_vision/lib/vision/calc_p3p.h"

#include "math/pprz_algebra.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_simple_matrix.h"

#define REFINEMENT 0
#define COLOR_FILTER 0
#define N_MISSING 3

// Gate detection settings:
int n_samples = 10000;//10000;//2000;//1000;//500;
int min_pixel_size = 55;//30;////30;//20;//40;//100;//TODO MAKE VARIABLE FOR CLIMBING TURN??///////////////////////////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!
float min_gate_quality = 0.15;//0.2;
float gate_thickness = 0;//0.05;//0.10;//
float gate_size = 34;

// //color I dont know
// uint8_t green_color[4] = {255,128,255,128}; //{0,250,0,250};
// uint8_t blue_color[4] = {0,128,0,128};//{250,250,0,250};

int y_low = 0;
int y_high = 0;
int x_low1 = 0;
int x_high1 = 0;
int x_low2 = 0;
int x_high2 = 0;
int sz = 0;
int szx1 = 0;
int szx2 = 0;

// int x = 0;
// int y = 0;


// old settings:
float princ_x = 157.0f;
float princ_y = 32.0f;
int f_fisheye = 168;
float k_fisheye = 1.150;
float k_fisheye_2 = 1.085;

// Result
#define MAX_GATES 50
struct gate_img gates_c[MAX_GATES];
struct gate_img best_gate;
struct gate_img temp_check_gate;
struct image_t img_result;
int n_gates = 0;
float best_quality = 0;
float current_quality = 0;
float best_fitness = 100000;
float psi_gate = 0;
float size_left = 0;
float size_right = 0;
//color picker
uint8_t y_center_picker  = 0;
uint8_t cb_center  = 0;
uint8_t cr_center  = 0;

//pixel distance conversion
int16_t pix_x = 0;
int16_t pix_y = 0;
int pix_sz = 0;
float hor_angle = 0;
float vert_angle = 0;
float x_dist = 0;
float y_dist = 0;
float z_dist = 0;

int last_frame_detection = 0;
int repeat_gate = 0;

// previous best gate:
struct gate_img previous_best_gate = {0};
struct gate_img last_gate;

//SAFETY AND RESET FLAGS
int uncertainty_gate = 0;
//int gate_detected = 0;
//init_pos_filter = 0;
int safe_pass_counter = 0;

float gate_quality = 0;

float fps_filter = 0;

struct timeval stop, start;

//QR code classification
int QR_class = 0;
float QR_uncertainty;

// back-side of a gate:
int back_side = 0;

//Debug messages

//free polygon points
int points_x[4];
int points_y[4];

int gates_sz = 0;

float x_center, y_center, radius;//,fitness, angle_1, angle_2, s_left, s_right;


//vectors for p3p etc
struct FloatVect3 vec_point_1, vec_point_2, vec_point_3, vec_point_4, vec_temp1, vec_temp2, vec_temp3, vec_temp4,
vec_ver_1, vec_ver_2, vec_ver_3, vec_ver_4;

struct FloatVect3 gate_vectors[4];
struct FloatVect3 gate_points[4];
struct FloatVect3 p3p_pos_solution;

//logging corner points in image plane
float gate_img_point_x_1 = 0;
float gate_img_point_y_1 = 0;
float gate_img_point_x_2 = 0;
float gate_img_point_y_2 = 0;
float gate_img_point_x_3 = 0;
float gate_img_point_y_3 = 0;
float gate_img_point_x_4 = 0;
float gate_img_point_y_4 = 0;

float local_psi = 0;

double last_detection_time = 0;

float gate_dist_x = 3.5;//distance from filter init point to gate 
float gate_size_m = 1.4;//size of gate edges in meters
float gate_center_height = -3.5;//height of gate in meters ned wrt ground


//qsort comp function for sorting 
int cmpfunc (const void * a, const void * b)
{
   return ( *(int*)a - *(int*)b );
}

int *array;
//return indexes
int cmp_i(const void *a, const void *b){
    int ia = *(int *)a;
    int ib = *(int *)b;
    return array[ia] < array[ib] ? -1 : array[ia] > array[ib];
}


void smooth_hist(int *smooth, int *raw_hist, int window){
  
  //start and end of hist padding
  for(int i = 0;i<window;i++){
    smooth[i] = 0;
  }
  
  for(int i = SNAKE_GATE_HEIGHT-window;i<SNAKE_GATE_HEIGHT;i++){
    smooth[i] = 0;
  }
  
  //avarage over 2 times window size
  for(int i = window;i<SNAKE_GATE_HEIGHT-window;i++){
    float sum = 0;
    for(int c = -window;c<window;c++){
      sum += raw_hist[i+c];
    }
    sum/=(window*2);
    smooth[i] = sum;
  }
  
}

int find_max_hist(int *hist){
  int max = 0;
  for(int i = 0;i<SNAKE_GATE_HEIGHT;i++){
    if(hist[i]>max)max = hist[i];
  }
  return max;
}

int find_hist_peeks(float *hist,int *peeks){
  int peek_count = 0;
  peeks[0] = 0;
  for(int i = 1;i<SNAKE_GATE_HEIGHT-1;i++){
    if(hist[i] > hist[i-1] && hist[i] > hist[i+1]){
      peeks[i] = hist[i];
      
    }
    else{
      peeks[i] =  0;
    }
    //peek_count++;
  }
  
  return peek_count;
}

int find_hist_peeks_flat(int *hist,int *peeks){
  int peek_count = 0;
  peeks[0] = 0;
  int last_sign = 0;
  int last_idx = 0;
  
  memset(peeks,0,SNAKE_GATE_HEIGHT*sizeof(int));//no peeks found yet
  
  for(int i = 1;i<SNAKE_GATE_HEIGHT-1;i++){
    
    int sign = hist[i] - hist[i-1];//positive up backwards difference
    
    if(last_sign > 0 && sign < 0){
      int m_idx = (i-last_idx)/2.0;
      peeks[last_idx+m_idx] = hist[last_idx+m_idx];
      peek_count++;
    }
    
    if(sign !=0){
      last_sign = sign;
      last_idx = i;
    }
    //peeks[i] = sign;
    //peek_count++;
  }
  
  return peek_count;
}

//for debugging
void print_hist(struct image_t *img,int *hist){
  
  int max_hist = find_max_hist(hist);
  int bound = max_hist/500;
  if(bound <= 0)bound = 1;
  
  for(int i = 0;i < img->h;i++){
    image_yuv422_set_color(img,img,i,hist[i]/bound);
  }
}

void print_sides(struct image_t *im, int side_1, int side_2){
    
    struct point_t from, to;
    from.x = side_1;
    from.y = 0;
    to.x = side_1;
    to.y = SNAKE_GATE_WIDTH;
    image_draw_line(im, &from, &to);
    from.x = side_2;
    from.y = 0;
    to.x = side_2;
    to.y = SNAKE_GATE_WIDTH;
    image_draw_line(im, &from, &to);
  
}

float detect_gate_sides(int *hist_raw, int *side_1, int *side_2, int* n_high_peaks){
  float high_peak_ratio = 0.5; // if a peak is higher than 50% of the highest one, it is considered a high peak
  int hist_peeks[SNAKE_GATE_HEIGHT];//max nr of peeks, for sure
  int hist_smooth[SNAKE_GATE_HEIGHT];
  smooth_hist(hist_smooth, hist_raw, 4);//was 10
  int peek_count = find_hist_peeks_flat(hist_smooth,hist_peeks);
  int max_peek = find_max_hist(hist_raw);
  
  int size = sizeof(hist_peeks)/sizeof(*hist_peeks); // TODO: is this right?
  int index[size];
  for(int i=0;i<size;i++){
        index[i] = i;
    }
  // array is used by qsort;
  array = hist_peeks;
  qsort(index, size, sizeof(*index), cmp_i);
  //printf("p1:%d p2:%d p3:%d p4:%d p5:%d p6:%d\n",hist_raw[index[0]],hist_raw[index[1]],hist_raw[index[2]],hist_raw[index[3]],hist_raw[index[4]],hist_raw[index[5]]);
  // printf("size:%d\n",size);
  //   printf("\n\ndata\tindex\n");
  //     for(int i=0;i<size;i++){
  //         printf("%d\t%d,i:%d\n", hist_peeks[index[i]], index[i],i);
  //     }
    
  // only interested if the number of high peaks is 0 - 4
  (*n_high_peaks) = 0;
  for(int i = 1; i < 5; i++){
      if(index[SNAKE_GATE_HEIGHT-i] >= (int)(0.5 * index[SNAKE_GATE_HEIGHT-1])) {
          (*n_high_peaks)++;
      }
  }

  //side 1 is left side
    if(index[SNAKE_GATE_HEIGHT-2] < index[SNAKE_GATE_HEIGHT-1]){
      *side_1 = index[SNAKE_GATE_HEIGHT-2];//313
      *side_2 = index[SNAKE_GATE_HEIGHT-1];//314
    }else{
      *side_1 = index[SNAKE_GATE_HEIGHT-1];//314
      *side_2 = index[SNAKE_GATE_HEIGHT-2];//313
    }
    
    //avarage peek height
    float peek_value = (hist_peeks[index[SNAKE_GATE_HEIGHT-2]] + hist_peeks[index[SNAKE_GATE_HEIGHT-1]])/2;
    //debug_5 = peek_value;

    // TODO: add peak ratio!!!
  
    return peek_value;

}

//##################################################################################################################
int closed_gate_processing(struct image_t *img){
  
  int filter = 0;
  int gate_graphics = 0;
  int x, y;//, y_low, y_high, x_low1, x_high1, x_low2, x_high2, sz, szx1, szx2;
  float quality;

  best_quality = 0;
  best_gate.gate_q = 0;

  n_gates = 0;
  
  //histogram for final approach gate detection
  int histogram[SNAKE_GATE_HEIGHT] = {0};
  
  for (int i = 0; i < n_samples; i++) {
    // get a random coordinate:
    x = rand() % img->h;
    y = rand() % img->w;
    
//     image_yuv422_set_color(img,img,x,y);

    if(check_color(img, x, y)){ 
      image_yuv422_set_color(img,img,x,y);;
    //check_color(img, 1, 1);
    // check if it has the right color
      
      
      
      //fill histogram
      histogram[x]++;
      
      // snake up and down:

      snake_up_and_down(img, x, y, &y_low, &y_high);
      /*snake_up_and_down_new(img, x, y, &y_low, &y_high);*/

      sz = y_high - y_low;

      y_low = y_low + (sz * gate_thickness);
      y_high = y_high - (sz * gate_thickness);

      y = (y_high + y_low) / 2;

      // if the stretch is long enough
      if (sz > min_pixel_size) {
        // snake left and right:

		 snake_left_and_right(img, x, y_low, &x_low1, &x_high1);
		 snake_left_and_right(img, x, y_high, &x_low2, &x_high2);
	/*snake_left_and_right_new(img, x, y_low, &x_low1, &x_high1);*/
        /*snake_left_and_right_new(img, x, y_high, &x_low2, &x_high2);*/


        x_low1 = x_low1 + (sz * gate_thickness);
        x_high1 = x_high1 - (sz * gate_thickness);
        x_low2 = x_low2 + (sz * gate_thickness);
        x_high2 = x_high2 - (sz * gate_thickness);

        // sizes of the left-right stretches: in y pixel coordinates
        szx1 = (x_high1 - x_low1);
        szx2 = (x_high2 - x_low2);

        // if the size is big enough:
        if (szx1 > min_pixel_size) {
          // draw four lines on the image:
          x = (x_high1 + x_low1) / 2;//was+
          // set the size to the largest line found:
          sz = (sz > szx1) ? sz : szx1;
          // create the gate:
          gates_c[n_gates].x = x;
          gates_c[n_gates].y = y;
          gates_c[n_gates].sz = sz / 2;
          // check the gate quality:
          check_gate(img, gates_c[n_gates], &quality, &gates_c[n_gates].n_sides);
          gates_c[n_gates].gate_q = quality;
          // only increment the number of gates if the quality is sufficient
          // else it will be overwritten by the next one
          if (quality > best_quality) { //min_gate_quality)
          //draw_gate(img, gates_c[n_gates]);
          best_quality = quality;
          n_gates++;
        }
      } else if (szx2 > min_pixel_size) {
          x = (x_high2 + x_low2) / 2;//was +
          // set the size to the largest line found:
          sz = (sz > szx2) ? sz : szx2;
          // create the gate:
          gates_c[n_gates].x = x;
          gates_c[n_gates].y = y;
          gates_c[n_gates].sz = sz / 2;
          // check the gate quality:
          check_gate(img, gates_c[n_gates], &quality, &gates_c[n_gates].n_sides);
          gates_c[n_gates].gate_q = quality;
          // only increment the number of gates if the quality is sufficient
          // else it will be overwritten by the next one
          if (quality > best_quality) { //min_gate_quality)
            //draw_gate(img, gates_c[n_gates]);
            best_quality = quality;
            n_gates++;
          }
        }
        if (n_gates >= MAX_GATES) {
          break;
        }

      }
      //
    }

  }
  
  //debug large closed snake gate
  ////////////////////////////////////////////////////////////////////////
  for(int i = 0;i < n_gates;i++){
    /*printf("n_gates:%d\n",n_gates);*/
    draw_gate(img, gates_c[i]);
  }
  /////////////////////////////////////////////////////////////////////////////////////////////

  // variables used for fitting:
  //float x_center, y_center, radius,
  //float fitness, angle_1, angle_2, s_left, s_right;
  //int clock_arms = 1;
  // prepare the Region of Interest (ROI), which is larger than the gate:
  float size_factor = 1.5;//2;//1.25;
  
  //init best gate
  best_gate.gate_q = 0;
  best_gate.n_sides = 0;
  
  repeat_gate = 0;

  // do an additional fit to improve the gate detection:
  if (REFINEMENT && ((best_quality > min_gate_quality && n_gates > 0)||last_frame_detection)) {// NO REFINEMENT FOR NOW !!///////////////////////////////


      int max_candidate_gates = 10;//10;

      best_fitness = 100;
      if (n_gates > 0 && n_gates < max_candidate_gates) {
        for (int gate_nr = 0; gate_nr < n_gates; gate_nr += 1) {
          int16_t ROI_size = (int16_t)(((float) gates_c[gate_nr].sz) * size_factor);
          int16_t min_x = gates_c[gate_nr].x - ROI_size;
          min_x = (min_x < 0) ? 0 : min_x;
          int16_t max_x = gates_c[gate_nr].x + ROI_size;
          max_x = (max_x < img->h) ? max_x : img->h;
          int16_t min_y = gates_c[gate_nr].y - ROI_size;
          min_y = (min_y < 0) ? 0 : min_y;
          int16_t max_y = gates_c[gate_nr].y + ROI_size;
          max_y = (max_y < img->w) ? max_y : img->w;

          //draw_gate(img, gates_c[gate_nr]);

          int gates_x = gates_c[gate_nr].x;
          int gates_y = gates_c[gate_nr].y;
          gates_sz = gates_c[gate_nr].sz;
	  
	  x_center = gates_x;
	  y_center = gates_y;
	  radius   = gates_sz;
          // detect the gate:

	  int x_center_p = x_center;
	  int y_center_p = y_center;
	  int radius_p   = radius;
	  gate_corner_ref(img, points_x, points_y, &x_center_p, &y_center_p, &radius_p,
                         (uint16_t) min_x, (uint16_t) min_y, (uint16_t) max_x, (uint16_t) max_y);

            // store the information in the gate:
            temp_check_gate.x = (int) x_center;
            temp_check_gate.y = (int) y_center;
            temp_check_gate.sz = (int) radius;
            
	    memcpy(&(temp_check_gate.x_corners[0]),points_x,sizeof(int)*4);
	    memcpy(&(temp_check_gate.y_corners[0]),points_y,sizeof(int)*4);
	    
            // also get the color fitness
            check_gate_free(img, temp_check_gate, &temp_check_gate.gate_q, &temp_check_gate.n_sides);
	    
	    if(temp_check_gate.n_sides > 3 && temp_check_gate.gate_q > best_gate.gate_q)
	    {
	   
	    //best_quality = .gate_q(first maybe zero
            // store the information in the gate:
            best_gate.x = temp_check_gate.x;
            best_gate.y = temp_check_gate.y;
            best_gate.sz = temp_check_gate.sz;
            best_gate.sz_left = temp_check_gate.sz_left;
            best_gate.sz_right = temp_check_gate.sz_right;
	    best_gate.gate_q = temp_check_gate.gate_q;
	    best_gate.n_sides = temp_check_gate.n_sides;
	    memcpy(&(best_gate.x_corners[0]),&(temp_check_gate.x_corners[0]),sizeof(int)*4);
	    memcpy(&(best_gate.y_corners[0]),&(temp_check_gate.y_corners[0]),sizeof(int)*4);
	    }
          //}

        }
        /*for (int gate_nr = 0; gate_nr < n_gates; gate_nr += 1) {
          draw_gate(img, gates_c[gate_nr]);
        }*/
      } else if (n_gates >= max_candidate_gates) {
        for (int gate_nr = n_gates - max_candidate_gates; gate_nr < n_gates; gate_nr += 1) {
          int16_t ROI_size = (int16_t)(((float) gates_c[gate_nr].sz) * size_factor);
          int16_t min_x = gates_c[gate_nr].x - ROI_size;
          min_x = (min_x < 0) ? 0 : min_x;
          int16_t max_x = gates_c[gate_nr].x + ROI_size;
          max_x = (max_x < img->h) ? max_x : img->h;
          int16_t min_y = gates_c[gate_nr].y - ROI_size;
          min_y = (min_y < 0) ? 0 : min_y;
          int16_t max_y = gates_c[gate_nr].y + ROI_size;
          max_y = (max_y < img->w) ? max_y : img->w;
    //draw_gate(img, gates_c[gate_nr]);
	  gates_sz = gates_c[gate_nr].sz;
	  x_center = gates_c[gate_nr].x;
	  y_center = gates_c[gate_nr].y;
	  radius   = gates_sz;
          // detect the gate:

	  int x_center_p = x_center;
	  int y_center_p = y_center;
	  int radius_p   = radius;
	  gate_corner_ref(img, points_x, points_y, &x_center_p, &y_center_p, &radius_p,
                         (uint16_t) min_x, (uint16_t) min_y, (uint16_t) max_x, (uint16_t) max_y);
	  
            // store the information in the gate:
            temp_check_gate.x = (int) x_center;
            temp_check_gate.y = (int) y_center;
            temp_check_gate.sz = (int) radius;
	    
	    memcpy(&(temp_check_gate.x_corners[0]),points_x,sizeof(int)*4);
	    memcpy(&(temp_check_gate.y_corners[0]),points_y,sizeof(int)*4);
	    
	    
            // also get the color fitness
            check_gate_free(img, temp_check_gate, &temp_check_gate.gate_q, &temp_check_gate.n_sides);
	    
	    if(temp_check_gate.n_sides > 3 && temp_check_gate.gate_q > best_gate.gate_q)
	    {
	    //best_fitness = fitness;
            // store the information in the gate:
            best_gate.x = temp_check_gate.x;
            best_gate.y = temp_check_gate.y;
            best_gate.sz = temp_check_gate.sz;
            best_gate.sz_left = temp_check_gate.sz_left;
            best_gate.sz_right = temp_check_gate.sz_right;
	    best_gate.gate_q = temp_check_gate.gate_q;
	    best_gate.n_sides = temp_check_gate.n_sides;
	    memcpy(&(best_gate.x_corners[0]),&(temp_check_gate.x_corners[0]),sizeof(int)*4);
	    memcpy(&(best_gate.y_corners[0]),&(temp_check_gate.y_corners[0]),sizeof(int)*4);
	    }
         // }
        }

        /*for (int gate_nr = n_gates - max_candidate_gates; gate_nr < n_gates; gate_nr += 1) {
          draw_gate(img, gates_c[gate_nr]);
        }*/
///////////////////////////////////////////////Use previous best estimate
      }
      
	 if((best_gate.gate_q == 0 && best_gate.n_sides == 0) && last_frame_detection == 1){
	
	int x_values[4];
	int y_values[4];
	
	memcpy(x_values,&(last_gate.x_corners[0]),sizeof(int)*4);
	memcpy(y_values,&(last_gate.y_corners[0]),sizeof(int)*4);
	
	//sort small to large 
	qsort(x_values, 4, sizeof(int), cmpfunc);
	qsort(y_values, 4, sizeof(int), cmpfunc);
// 	printf("in repeat_gate ########################################################\n ");
// 	printf("repeat_gate x1:%d x2:%d x3:%d x4:%d\n",last_gate.x_corners[0],last_gate.x_corners[1],last_gate.x_corners[2],last_gate.x_corners[3]);
// 	printf("repeat_gate y1:%d y2:%d y3:%d y4:%d\n",last_gate.y_corners[0],last_gate.y_corners[1],last_gate.y_corners[2],last_gate.y_corners[3]);	

	//check x size, maybe use y also later?
 	  int radius_p   = x_values[3]-x_values[0];
	  gate_corner_ref_2(img, last_gate.x_corners, last_gate.y_corners,&radius_p);
	    
            // also get the color fitness
            check_gate_free(img, last_gate, &last_gate.gate_q, &last_gate.n_sides);
	    
	    if(last_gate.n_sides > 3 && last_gate.gate_q > best_gate.gate_q)
	    {
	      repeat_gate = 1;

	    best_gate.gate_q = last_gate.gate_q;
	    best_gate.n_sides = last_gate.n_sides;
	    memcpy(&(best_gate.x_corners[0]),&(last_gate.x_corners[0]),sizeof(int)*4);
	    memcpy(&(best_gate.y_corners[0]),&(last_gate.y_corners[0]),sizeof(int)*4);
	    }
      }
   
  } else {
    //random position guesses here and then genetic algorithm
    //use random sizes and positions bounded by minimum size

  }



  // prepare for the next time:
  previous_best_gate.x = best_gate.x;
  previous_best_gate.y = best_gate.y;
  previous_best_gate.sz = best_gate.sz;
  previous_best_gate.sz_left = best_gate.sz_left;
  previous_best_gate.sz_right = best_gate.sz_right;
  previous_best_gate.gate_q = best_gate.gate_q;
  previous_best_gate.n_sides = best_gate.n_sides;
  
    //color filtered version of image for overlay and debugging
  if (COLOR_FILTER){
    int num_color = image_yuv422_colorfilt(img, img,
                      color_lum_min, color_lum_max,
                      color_cb_min, color_cb_max,
                      color_cr_min, color_cr_max
                                            );
  }

  
  //set gate point coordinate logging values to zero, in case no detection happens
  gate_img_point_x_1 = 0;
  gate_img_point_y_1 = 0;
  gate_img_point_x_2 = 0;
  gate_img_point_y_2 = 0;
  gate_img_point_x_3 = 0;
  gate_img_point_y_3 = 0;
  gate_img_point_x_4 = 0;
  gate_img_point_y_4 = 0;
  
  //draw_gate_color(img, best_gate, blue_color);
  
//change to gate based heading 
    local_psi = 0;//stateGetNedToBodyEulers_f()->psi - gate_heading;--------------------------------------------------------------

  int side_1;
  int side_2;
  int n_high_peaks;
  float hist_peek_value = detect_gate_sides(histogram,&side_1, &side_2, &n_high_peaks);
  
  //printf("side_1[%d] side_2[%d]\n",side_1,side_2);
  
  //transform to angles in image frame, ignoring tilt angle of 20 deg
  float undist_x, undist_y;
  /*float original_height_calibration = 315.0f;
  float size_factor_height = SNAKE_GATE_HEIGHT / original_height_calibration;
  float princ_x = size_factor_height * 157.0f;
  float princ_y = size_factor_height * 32.0f;
  int f_fisheye = size_factor_height * 168.0f;
  */
  
  //float psi_comp = stateGetNedToBodyEulers_f()->psi;
  undistort_fisheye_point(side_1 ,princ_y,&undist_x,&undist_y,f_fisheye,k_fisheye,princ_x,princ_y);
  float side_angle_1 = atanf(undist_x/f_fisheye)+local_psi;
  undistort_fisheye_point(side_2 ,princ_y,&undist_x,&undist_y,f_fisheye,k_fisheye,princ_x,princ_y);
  float side_angle_2 = atanf(undist_x/f_fisheye)+local_psi;
  
  float b = (tanf(side_angle_2)*tanf(side_angle_1))/((tanf(side_angle_1)-tanf(side_angle_2))*tanf(side_angle_1)); //tanf(side_angle_1)/(tanf(side_angle_2)-tanf(side_angle_1));
  y_pos_hist = 0.4+b;//was 0.5
  //float a = 1-b;
  x_pos_hist = b/tanf(-side_angle_2);//(0.5+y_pos_hist)/tanf(-side_angle_1);// tanf(side_angle_2)*b;
  
  float max_dist_h = 1.8;
  float min_dist_h = 0.3;
  float vertical_ratio =  0.04375; // * 160 = 7
  if(hist_peek_value > vertical_ratio * (float) SNAKE_GATE_WIDTH && x_pos_hist < max_dist_h && x_pos_hist > min_dist_h){
    hist_sample = 1;
    print_sides(img,side_1,side_2);
  }else{
    hist_sample = 0;
  }
  
  //print_sides(img,side_1,side_2);
  
//   debug_3 = x_pos_hist;
//   debug_4 = y_pos_hist;
  
//   debug_3 = side_angle_1*57;
//   debug_4 = side_angle_2;
  //debug_5 = local_psi*57;
  
  /////////////////////////////////////////////////////////////////////////
  //print_hist(img,histogram);
  
  
  if (best_gate.gate_q > (min_gate_quality*2) && best_gate.n_sides > 3) {//n_sides was > 2

    
    //sucessfull detection
    last_frame_detection = 1;
    vision_sample = 1;
    
    gettimeofday(&stop, 0);
    last_detection_time = (double)(stop.tv_sec + stop.tv_usec / 1000000.0);
    
    
    
    //draw_gate_color(img, best_gate, blue_color);
    if(1){//gate drawing /////////////////////////////////////////////////////////////////////////////////////
	if(repeat_gate == 0){
	  draw_gate_polygon(img,best_gate.x_corners,best_gate.y_corners,blue_color);
	}
	else if(repeat_gate == 1){
	  draw_gate_polygon(img,best_gate.x_corners,best_gate.y_corners,green_color);
	  for(int i = 0;i < 3;i++){
	    draw_cross(img,last_gate.x_corners[i],last_gate.y_corners[i],blue_color);
	  }
	}
    }
    //save for next iteration
    memcpy(&(last_gate.x_corners[0]),&(best_gate.x_corners[0]),sizeof(int)*4);
    memcpy(&(last_gate.y_corners[0]),&(best_gate.y_corners[0]),sizeof(int)*4);
    //previous best snake gate
    last_gate.x = best_gate.x;
    last_gate.y = best_gate.y;
    last_gate.sz = best_gate.sz;
    
    
    current_quality = best_quality;
    size_left = best_gate.sz_left;
    size_right = best_gate.sz_right;
    

    gate_quality = best_gate.gate_q;
        	  
	    //Small gate 1m
// 	  VECT3_ASSIGN(gate_points[0], gate_dist_x,-0.5000, -1.9000);
// 	  VECT3_ASSIGN(gate_points[1], gate_dist_x,0.5000, -1.9000);
// 	  VECT3_ASSIGN(gate_points[2], gate_dist_x,0.5000, -0.9000);
// 	  VECT3_ASSIGN(gate_points[3], gate_dist_x,-0.5000, -0.9000);
    
	  //Variable gate dist TODO test variable gate dist
	  //gate_dist_x = 3.5;//distance from filter init point to gate 
	  //gate_size_m = 1.4;//size of gate edges in meters
	  gate_center_height = -3.5;//
	  
          VECT3_ASSIGN(gate_points[0], gate_dist_x,-(gate_size_m/2), gate_center_height-(gate_size_m/2));
	  VECT3_ASSIGN(gate_points[1], gate_dist_x, (gate_size_m/2), gate_center_height-(gate_size_m/2));
	  VECT3_ASSIGN(gate_points[2], gate_dist_x, (gate_size_m/2), gate_center_height+(gate_size_m/2));
	  VECT3_ASSIGN(gate_points[3], gate_dist_x,-(gate_size_m/2), gate_center_height+(gate_size_m/2));
	  
	
	//Undistort fisheye points
	float x_gate_corners[4];
	float y_gate_corners[4];
		
	struct FloatRMat R,R_20,R_trans,Q_mat,I_mat, temp_mat, temp_mat_2;
	struct FloatEulers attitude, cam_body;
	struct FloatVect3 vec_20, p_vec,pos_vec,temp_vec,n_vec;
	
	p_vec.x = 0;
	p_vec.y = 0;
	p_vec.z = 0;
	
	MAT33_ELMT(I_mat, 0, 0) = 1;//(row,column)
	MAT33_ELMT(I_mat, 0, 1) = 0;
	MAT33_ELMT(I_mat, 0, 2) = 0;

	MAT33_ELMT(I_mat, 1, 0) = 0;//(row,column)
	MAT33_ELMT(I_mat, 1, 1) = 1;
	MAT33_ELMT(I_mat, 1, 2) = 0;

	MAT33_ELMT(I_mat, 2, 0) = 0;//(row,column)
	MAT33_ELMT(I_mat, 2, 1) = 0;
	MAT33_ELMT(I_mat, 2, 2) = 1;
	
	MAT33_ELMT(Q_mat, 0, 0) = 0;//(row,column)
	MAT33_ELMT(Q_mat, 0, 1) = 0;
	MAT33_ELMT(Q_mat, 0, 2) = 0;

	MAT33_ELMT(Q_mat, 1, 0) = 0;//(row,column)
	MAT33_ELMT(Q_mat, 1, 1) = 0;
	MAT33_ELMT(Q_mat, 1, 2) = 0;

	MAT33_ELMT(Q_mat, 2, 0) = 0;//(row,column)
	MAT33_ELMT(Q_mat, 2, 1) = 0;
	MAT33_ELMT(Q_mat, 2, 2) = 0;
	
	p_vec.x = 0;
	p_vec.y = 0;
	p_vec.z = 0;
	
	

	attitude.phi =    stateGetNedToBodyEulers_f()->phi;//positive ccw
	attitude.theta = stateGetNedToBodyEulers_f()->theta;//negative downward
	

	
	attitude.psi = local_psi;
	
	//debug_5 = attitude.psi;
	
	float_rmat_of_eulers_321(&R,&attitude);
	
	
	
	//for(int i = 0;i<4;i++)
	for(int i = 1;i<3;i++)
	{
	  float undist_x, undist_y;
	  //debug_1 = (float)best_gate.x_corners[i];// best_gate.y_corners[i]
	  //debug_2 = (float)best_gate.y_corners[i];//
	  //undist_y = 5;//(float)best_gate.y_corners[i];
	  //float princ_x = 157.0;
	  //float princ_y = 32.0;
	  undistort_fisheye_point(best_gate.x_corners[i] ,best_gate.y_corners[i],&undist_x,&undist_y,f_fisheye,k_fisheye_2,princ_x,princ_y);
	  
	  x_gate_corners[i] = undist_x + princ_x;
	  y_gate_corners[i] = undist_y + princ_y;
	  
	  if(gate_graphics)draw_cross(img,((int)x_gate_corners[i]),((int)y_gate_corners[i]),green_color);
	  //if(gate_graphics)draw_cross(img,((int)x_trail[i])+157,((int)y_trail[i])+32,green_color);
	  vec_from_point_ned(undist_x, undist_y, f_fisheye,&gate_vectors[i]);
	  
	  //Least squares stuff here:
	  vec_from_point_2(undist_x,undist_y,f_fisheye,&temp_vec);
	  //vec_from_point_2(x_trail[i],y_trail[i],f_fisheye,&temp_vec);
	  
	  //camera to body rotation
	  cam_body.phi = 0;
	  cam_body.theta = -25*(3.14/180);
	  cam_body.psi = 0;
	  float_rmat_of_eulers_321(&R_20,&cam_body);
	  MAT33_VECT3_MUL(vec_20, R_20,temp_vec);
	  
	  
	  MAT33_TRANS(R_trans,R);
	  MAT33_VECT3_MUL(n_vec, R_trans, vec_20);
	  //print_vector(n_vec);
	  
	  //to ned
	  n_vec.z = -n_vec.z;
	 
	  double vec_norm = sqrt(VECT3_NORM2(n_vec));
	  VECT3_SDIV(n_vec, n_vec, vec_norm);
	  VECT3_VECT3_TRANS_MUL(temp_mat, n_vec,n_vec);
	  MAT33_MAT33_DIFF(temp_mat,I_mat,temp_mat); 
	  MAT33_COPY(temp_mat_2,Q_mat);
	  MAT33_MAT33_SUM(Q_mat,temp_mat_2,temp_mat);
	  MAT33_VECT3_MUL(temp_vec, temp_mat, gate_points[i]);
	  VECT3_SUM(p_vec,p_vec,temp_vec);
	  
// 	  for i = 1:4
// 	  R = R + (eye(3,3)-n(:,i)*n(:,i)');
// 	  q = q + (eye(3,3)-n(:,i)*n(:,i)')*a(:,i);
// 
// 	  p = R\q;
//	  or q = Rp
//        hence p = R_inv*q
	  
	  
	}

	
	MAT33_INV(temp_mat,Q_mat);
	
	MAT33_VECT3_MUL(pos_vec, temp_mat,p_vec);
	
	ls_pos_x = pos_vec.x;
	
	//bound y to remove outliers 
	float y_threshold = 4;//2.5;
	if(pos_vec.y > y_threshold)pos_vec.y = y_threshold;
	if(pos_vec.y < -y_threshold)pos_vec.y = -y_threshold;
	
	
	ls_pos_y = pos_vec.y;//// minus for simulating open gate ! ---------------------------------------------------------------------
	//if(stateGetNedToBodyEulers_f()->psi > 1.6 || stateGetNedToBodyEulers_f()->psi < -1.6)ls_pos_y-=0.25;
	
	ls_pos_z = pos_vec.z;
 	//debug_3 = ls_pos_x;
 	//debug_4 = ls_pos_y;
// 		printf("R_mat_trans:\n");
//   		print_matrix(R_trans);
	  
	gate_img_point_x_1 = best_gate.x_corners[0];
	gate_img_point_y_1 = best_gate.y_corners[0];
	gate_img_point_x_2 = best_gate.x_corners[1];
	gate_img_point_y_2 = best_gate.y_corners[1];
	gate_img_point_x_3 = best_gate.x_corners[2];
	gate_img_point_y_3 = best_gate.y_corners[2];
	gate_img_point_x_4 = best_gate.x_corners[3];
	gate_img_point_y_4 = best_gate.y_corners[3];
	
    
  } else {

      //no detection
      vision_sample = 0;
    
      last_frame_detection = 0;
     
      current_quality = 0;
    return 0;
  }

	//better principal point?
 	//draw_cross(img,158,32,green_color);
return 1;	
}

void print_matrix(struct FloatMat33 mat)
{
  printf("mat:\n");
  printf("%f, %f, %f\n",MAT33_ELMT(mat, 0, 0),MAT33_ELMT(mat, 0, 1),MAT33_ELMT(mat, 0, 2));
  printf("%f, %f, %f\n",MAT33_ELMT(mat, 1, 0),MAT33_ELMT(mat, 1, 1),MAT33_ELMT(mat, 1, 2));
  printf("%f, %f, %f\n",MAT33_ELMT(mat, 2, 0),MAT33_ELMT(mat, 2, 1),MAT33_ELMT(mat, 2, 2));
}

void print_vector(struct FloatVect3 vec)
{
  printf("Vec: %f, %f, %f\n",vec.x,vec.y,vec.z);
}


int find_minimum(float *error)
{
    float minimum = error[0];
    int location = 0;
    
	for (int c = 1 ; c < 4 ; c++ ) 
	{
	    if ( error[c] < minimum ) 
	    {
	      minimum = error[c];
	      location = c;
	    }
	}
    return location;
}


float euclidean_distance(float x_i, float x_bp, float y_i, float y_bp)
{
  float dist = sqrt(pow((x_i - x_bp),2)+pow((y_i - y_bp),2));
  return dist;
}

void undistort_fisheye_point(int point_x, int point_y, float *undistorted_x, float *undistorted_y, int f, float k, float x_img_center, float y_img_center)
{
  /*
f = 168;
k = 1.085;
%k = 1.051;
%k = 1.118;

*/
  
  float x_mid = (float)(point_x) - princ_x;//-(float)(x_princip);
  float y_mid = (float)(point_y) - princ_y;//-(float)(y_princip);
  
  //debug_1 = x_mid;
  //debug_2 = y_mid;
  
  //to polar coordinates
  float r = sqrtf((pow(x_mid,2))+(pow(y_mid,2)));
  float theta = atan2f(y_mid,x_mid);//atanf?
  
  //debug_1 = r;
  //debug_2 = theta;
  
  //debug_1 = k;
  //debug_2 = (float)f;
  //k = 1.085;
//k = 1.051;

//k = 1.080;//last k
 //  k = 1.118;
  ///WAS 1.500 ??? why
  //k = 1.500;
  //////////
  //1.150 used in matlab
  k = k_fisheye;
  //k = 1.218;
  
  //radial distortion correction
  float R = (float)f*tan(asin(sin( atan(r/(float)f))*k));
  
                                                  // +y
                                                  // ^
                                                  // |
                                                  // |
  *undistorted_x =  R * cos(theta);//+x_princip; in (0,0)--->+x
  *undistorted_y =  R * sin(theta);//+y_princip;
  
  
}

//calculate unit vector from undistorted image point.
void vec_from_point(float point_x, float point_y, int f, struct FloatVect3 *vec)
{
  
  vec->x = (float)f;
  vec->y = (float)point_x;
  vec->z = (float)point_y;
  
  double norm = sqrt(VECT3_NORM2(*vec));
  VECT3_SDIV(*vec, *vec, norm);
  
}

void vec_from_point_ned(float point_x, float point_y, int f, struct FloatVect3 *vec)
{
  
  vec->x = (float)f;
  vec->y = (float)point_x;
  vec->z = -(float)point_y;
  
  double norm = sqrt(VECT3_NORM2(*vec));
  VECT3_SDIV(*vec, *vec, norm);
  
}

void vec_from_point_2(float point_x, float point_y, int f, struct FloatVect3 *vec)
{
  
  f = f_fisheye;
  float focus = (float)f;
  vec->x = 1.0;
  //printf("Print in point 1\n");
  vec->y = point_x/focus;
  //printf("Print in point 2\n");
  vec->z = point_y/focus;
  //printf("Print in point 3\n");
}


void back_proj_points(struct FloatVect3 *gate_point, struct FloatVect3 *cam_pos, struct FloatMat33 *R_mat, float *x_res, float *y_res)
{
  
//   point_3d = R*(gate_point-cam_pos');
// 
//     hom_coord = [point_3d(2)/point_3d(1);
//                  point_3d(3)/point_3d(1);
//                  1];
// 
//     res = intr*hom_coord;
  
  struct FloatVect3 temp1, point_3d, hom_coord, res;
  struct FloatMat33 intr, R_t;
  
  VECT3_DIFF(temp1,*gate_point,*cam_pos);
  MAT33_TRANS(R_t,*R_mat);
  MAT33_VECT3_MUL(point_3d,R_t,temp1);
//   MAT33_VECT3_MUL(point_3d,*R_mat,temp1);
  hom_coord.x = point_3d.y/point_3d.x;
  hom_coord.y = -point_3d.z/point_3d.x;
  hom_coord.z = 1;
  
//   debug_1 = hom_coord.x;
//   debug_2 = hom_coord.y;
  
  MAT33_ELMT(intr, 0, 0) = f_fisheye;//(row,column)
  MAT33_ELMT(intr, 0, 1) = 0;
  MAT33_ELMT(intr, 0, 2) = (int) princ_x;

  MAT33_ELMT(intr, 1, 0) = 0;//(row,column)
  MAT33_ELMT(intr, 1, 1) = f_fisheye;
  MAT33_ELMT(intr, 1, 2) = (int) princ_y;

  MAT33_ELMT(intr, 2, 0) = 0;//(row,column)
  MAT33_ELMT(intr, 2, 1) = 0;
  MAT33_ELMT(intr, 2, 2) = 0;
  
  MAT33_VECT3_MUL(res,intr,hom_coord);
  
//   debug_1 = res.x;
//   debug_2 = res.y;
  
  *x_res = res.x;
  *y_res = res.y;
  
  
  //Rmat
  
  //image_error()//compare with undist_x and undist_y
}

void draw_gate(struct image_t *im, struct gate_img gate)
{
  // draw four lines on the image:
  struct point_t from, to;
  if (gate.sz_left == gate.sz_right) {
    // square
    from.x = (gate.x - gate.sz);
    from.y = gate.y - gate.sz;
    to.x = (gate.x - gate.sz);
    to.y = gate.y + gate.sz;
    image_draw_line(im, &from, &to);
    from.x = (gate.x - gate.sz);
    from.y = gate.y + gate.sz;
    to.x = (gate.x + gate.sz);
    to.y = gate.y + gate.sz;
    image_draw_line(im, &from, &to);
    from.x = (gate.x + gate.sz);
    from.y = gate.y + gate.sz;
    to.x = (gate.x + gate.sz);
    to.y = gate.y - gate.sz;
    image_draw_line(im, &from, &to);
    from.x = (gate.x + gate.sz);
    from.y = gate.y - gate.sz;
    to.x = (gate.x - gate.sz);
    to.y = gate.y - gate.sz;
    image_draw_line(im, &from, &to);
  } else {
    // polygon
    from.x = (gate.x - gate.sz);
    from.y = gate.y - gate.sz_left;
    to.x = (gate.x - gate.sz);
    to.y = gate.y + gate.sz_left;
    image_draw_line(im, &from, &to);
    from.x = (gate.x - gate.sz);
    from.y = gate.y + gate.sz_left;
    to.x = (gate.x + gate.sz);
    to.y = gate.y + gate.sz_right;
    image_draw_line(im, &from, &to);
    from.x = (gate.x + gate.sz);
    from.y = gate.y + gate.sz_right;
    to.x = (gate.x + gate.sz);
    to.y = gate.y - gate.sz_right;
    image_draw_line(im, &from, &to);
    from.x = (gate.x + gate.sz);
    from.y = gate.y - gate.sz_right;
    to.x = (gate.x - gate.sz);
    to.y = gate.y - gate.sz_left;
    image_draw_line(im, &from, &to);
  }
}

void draw_gate_color(struct image_t *im, struct gate_img gate, uint8_t* color)
{
  
  // draw four lines on the image:
  struct point_t from, to;
  if (gate.sz_left == gate.sz_right) {
    // square
    from.x = (gate.x - gate.sz);
    from.y = gate.y - gate.sz;
    to.x = (gate.x - gate.sz);
    to.y = gate.y + gate.sz;
    image_draw_line_color(im, &from, &to, color);
    //draw_line_segment(im, from, to, color);
    from.x = (gate.x - gate.sz);
    from.y = gate.y + gate.sz;
    to.x = (gate.x + gate.sz);
    to.y = gate.y + gate.sz;
    image_draw_line_color(im, &from, &to, color);
    //draw_line_segment(im, from, to, color);
    from.x = (gate.x + gate.sz);
    from.y = gate.y + gate.sz;
    to.x = (gate.x + gate.sz);
    to.y = gate.y - gate.sz;
    image_draw_line_color(im, &from, &to, color);
    // draw_line_segment(im, from, to, color);
    from.x = (gate.x + gate.sz);
    from.y = gate.y - gate.sz;
    to.x = (gate.x - gate.sz);
    to.y = gate.y - gate.sz;
    image_draw_line_color(im, &from, &to, color);
    //draw_line_segment(im, from, to, color);
  } else {
    // polygon
    from.x = (gate.x - gate.sz);
    from.y = gate.y - gate.sz_left;
    to.x = (gate.x - gate.sz);
    to.y = gate.y + gate.sz_left;
    image_draw_line_color(im, &from, &to, color);
    //draw_line_segment(im, from, to, color);
    from.x = (gate.x - gate.sz);
    from.y = gate.y + gate.sz_left;
    to.x = (gate.x + gate.sz);
    to.y = gate.y + gate.sz_right;
    image_draw_line_color(im, &from, &to, color);
    //draw_line_segment(im, from, to, color);
    from.x = (gate.x + gate.sz);
    from.y = gate.y + gate.sz_right;
    to.x = (gate.x + gate.sz);
    to.y = gate.y - gate.sz_right;
    image_draw_line_color(im, &from, &to, color);
    //draw_line_segment(im, from, to, color);
    from.x = (gate.x + gate.sz);
    from.y = gate.y - gate.sz_right;
    to.x = (gate.x - gate.sz);
    to.y = gate.y - gate.sz_left;
    image_draw_line_color(im, &from, &to, color);
    //draw_line_segment(im, from, to, color);
  }
}

//AXIS system
//(0,0)   SNAKE_GATE_WIDTH
//       Y
// -|----->
//  |
//  |
//  |
//  |
//  \/
// X
// SNAKE_GATE_HEIGHT

void draw_cross(struct image_t *im,int x, int y, uint8_t* color)
{
  struct point_t from, to;
  
    //polygon
    from.x = x-10;
    from.y = y;
    to.x = x+10;
    to.y = y;
    image_draw_line_color(im, &from, &to, color);
    //draw_line_segment(im, from, to, color);
    from.x = x;
    from.y = y-10;
    to.x = x;
    to.y = y+10;
    image_draw_line_color(im, &from, &to, color);
}

void draw_gate_polygon(struct image_t *im, int *x_points, int *y_points, uint8_t* color)
{
  
  
  // draw four lines on the image:
  struct point_t from, to;
  
    //polygon
    from.x = x_points[0];
    from.y = y_points[0];
    to.x = x_points[1];
    to.y = y_points[1];
    image_draw_line_color(im, &from, &to, color);
   //draw_cross(im,x_points[0],y_points[0],green_color);
    //draw_line_segment(im, from, to, color);
    from.x = x_points[1];
    from.y = y_points[1];
    to.x = x_points[2];
    to.y = y_points[2];
    image_draw_line_color(im, &from, &to, color);
   //draw_cross(im,x_points[1],y_points[1],green_color);
    from.x = x_points[2];
    from.y = y_points[2];
    to.x = x_points[3];
    to.y = y_points[3];
    image_draw_line_color(im, &from, &to, color);
   //draw_cross(im,x_points[2],y_points[2],green_color);
    // draw_line_segment(im, from, to, color);
    from.x = x_points[3];
    from.y = y_points[3];
    to.x = x_points[0];
    to.y = y_points[0];
    image_draw_line_color(im, &from, &to, color);
    //draw_line_segment(im, from, to, color);
  
}

void check_gate_free(struct image_t *im, struct gate_img gate, float *quality, int *n_sides)
{
  int n_points, n_colored_points;
  n_points = 0;
  n_colored_points = 0;
  int np, nc;
    
  // how much of the side should be visible to count as a detected side?
  float min_ratio_side = 0.30;
  (*n_sides) = 0;

  // check the four lines of which the gate consists:
  struct point_t from, to;
  
    from.x = gate.x_corners[0];
    from.y = gate.y_corners[0];
    to.x = gate.x_corners[1];
    to.y = gate.y_corners[1];
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    n_points += np;
    n_colored_points += nc;

    from.x = gate.x_corners[1];
    from.y = gate.y_corners[1];
    to.x = gate.x_corners[2];
    to.y = gate.y_corners[2];
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    n_points += np;
    n_colored_points += nc;

    from.x = gate.x_corners[2];
    from.y = gate.y_corners[2];
    to.x = gate.x_corners[3];
    to.y = gate.y_corners[3];
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    n_points += np;
    n_colored_points += nc;

    from.x = gate.x_corners[3];
    from.y = gate.y_corners[3];
    to.x = gate.x_corners[0];
    to.y = gate.y_corners[0];
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    /*else{
        (*n_sides) = 0;
    }*/
    n_points += np;
    n_colored_points += nc;
  

  // the quality is the ratio of colored points / number of points:
  if (n_points == 0) {
    (*quality) = 0;
  } else {
    (*quality) = ((float) n_colored_points) / ((float) n_points);
  }
}

extern void check_gate(struct image_t *im, struct gate_img gate, float *quality, int *n_sides)
{
  int n_points, n_colored_points;
  n_points = 0;
  n_colored_points = 0;
  int np, nc;
  
  // how much of the side should be visible to count as a detected side?
  float min_ratio_side = 0.30;
  (*n_sides) = 0;

  // check the four lines of which the gate consists:
  struct point_t from, to;
  if (gate.sz_left == gate.sz_right) {

    from.x = gate.x - gate.sz;
    from.y = gate.y - gate.sz;
    to.x = gate.x - gate.sz;
    to.y = gate.y + gate.sz;
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    n_points += np;
    n_colored_points += nc;

    from.x = gate.x - gate.sz;
    from.y = gate.y + gate.sz;
    to.x = gate.x + gate.sz;
    to.y = gate.y + gate.sz;
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    n_points += np;
    n_colored_points += nc;

    from.x = gate.x + gate.sz;
    from.y = gate.y + gate.sz;
    to.x = gate.x + gate.sz;
    to.y = gate.y - gate.sz;
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    n_points += np;
    n_colored_points += nc;

    from.x = gate.x + gate.sz;
    from.y = gate.y - gate.sz;
    to.x = gate.x - gate.sz;
    to.y = gate.y - gate.sz;
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    /*else{
        (*n_sides) = 0;
    }*/
    n_points += np;
    n_colored_points += nc;
  } else {
    from.x = gate.x - gate.sz;
    from.y = gate.y - gate.sz_left;
    to.x = gate.x - gate.sz;
    to.y = gate.y + gate.sz_left;
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    n_points += np;
    n_colored_points += nc;

    from.x = gate.x - gate.sz;
    from.y = gate.y + gate.sz_left;
    to.x = gate.x + gate.sz;
    to.y = gate.y + gate.sz_right;
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    n_points += np;
    n_colored_points += nc;

    from.x = gate.x + gate.sz;
    from.y = gate.y + gate.sz_right;
    to.x = gate.x + gate.sz;
    to.y = gate.y - gate.sz_right;
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    n_points += np;
    n_colored_points += nc;

    from.x = gate.x + gate.sz;
    from.y = gate.y - gate.sz_right;
    to.x = gate.x - gate.sz;
    to.y = gate.y - gate.sz_left;
    check_line(im, from, to, &np, &nc);
    if ((float) nc / (float) np >= min_ratio_side) {
      (*n_sides)++;
    }
    /*else{
        (*n_sides) = 0;
    }*/
    n_points += np;
    n_colored_points += nc;
  }

  // the quality is the ratio of colored points / number of points:
  if (n_points == 0) {
    (*quality) = 0;
  } else {
    (*quality) = ((float) n_colored_points) / ((float) n_points);
  }
  
  
  //randomly sample inside 
  int n_samples_in = 20;
  float num_color_center = 0;
  float center_discard_threshold = 0.25;
  
  for (int i = 0; i < n_samples_in; i++) {
    // get a random coordinate:
    int x_in = gate.x + (rand() % gate.sz)-(0.5*gate.sz);//im->h;
    int y_in = gate.y + (rand() % gate.sz)-(0.5*gate.sz);//im->w;
    
    // image_yuv422_set_color(im,im,x_in,y_in);

    // check if it has the right color
    if (check_color(im, x_in, y_in)) {
      num_color_center ++;
    }
  }
  
  //how much center pixels colored?
  float center_factor = num_color_center/(float)n_samples_in;
  if(center_factor > center_discard_threshold) (*quality) = 0;
  
}


void check_line(struct image_t *im, struct point_t Q1, struct point_t Q2, int *n_points, int *n_colored_points)
{
  (*n_points) = 0;
  (*n_colored_points) = 0;

  float t_step = 0.05;
  int x, y;
  float t;
  // go from Q1 to Q2 in 1/t_step steps:
  for (t = 0.0f; t < 1.0f; t += t_step) {
    // determine integer coordinate on the line:
    x = (int)(t * Q1.x + (1.0f - t) * Q2.x);
    y = (int)(t * Q1.y + (1.0f - t) * Q2.y);

    // if (x >= 0 && x < im->w && y >= 0 && y < im->h) {
    if (x >= 0 && x < im->h && y >= 0 && y < im->w) {
      // augment number of checked points:
      (*n_points)++;

      if (check_color(im, x, y)) {
        // the point is of the right color:
        (*n_colored_points)++;
      }
    }
  }
}

void snake_up_and_down(struct image_t *im, int x, int y, int *y_low, int *y_high)
{
  int done = 0;
  int x_initial = x;
  (*y_low) = y;

  int missing = 0;
  // snake towards negative y (down?)
  while ((*y_low) > 0 && !done) {
    if (check_color(im, x, (*y_low) - 1)) {
      (*y_low)--;
      missing = 0;

    } else if (x+1 < im->h && check_color(im, x + 1, (*y_low) - 1)) {
      x++;
      (*y_low)--;
      missing = 0;
    } else if (x-1 >= 0 && check_color(im, x - 1, (*y_low) - 1)) {
      x--;
      (*y_low)--;
      missing = 0;
    } else {
        missing++;
        (*y_low)--;
        if(missing >= N_MISSING) {
            done = 1;
            (*y_low) += missing;
        }
    }
  }

  x = x_initial;
  (*y_high) = y;
  done = 0;
  // snake towards positive y (up?)
  // while ((*y_high) < im->h - 1 && !done) {
  missing = 0;
  while ((*y_high) < im->w - 1 && !done) {

    if (check_color(im, x, (*y_high) + 1)) {
        missing = 0;
        (*y_high)++;
    //    } else if (x < im->w - 1 && check_color(im, x + 1, (*y_high) + 1)) {
    } else if (x < im->h - 1 && check_color(im, x + 1, (*y_high) + 1)) {
        missing = 0;
        x++;
        (*y_high)++;
    } else if (x > 0 && check_color(im, x - 1, (*y_high) + 1)) {
        missing = 0;
        x--;
      (*y_high)++;
    } else {
        missing++;
        (*y_high)++;
        if(missing >= N_MISSING) {
            done = 1;
            (*y_high) -= missing;
        }
    }
  }
}

void snake_left_and_right(struct image_t *im, int x, int y, int *x_low, int *x_high)
{
  int done = 0;
  int y_initial = y;
  (*x_low) = x;

  int missing = 0;

  // snake towards negative x (left)
  while ((*x_low) > 0 && !done) {
    if (check_color(im, (*x_low) - 1, y)) {
      (*x_low)--;
      missing = 0;
    // } else if (y < im->h - 1 && check_color(im, (*x_low) - 1, y + 1)) {
    } else if (y < im->w - 1 && check_color(im, (*x_low) - 1, y + 1)) {
      y++;
      (*x_low)--;
      missing = 0;
    } else if (y > 0 && check_color(im, (*x_low) - 1, y - 1)) {
      y--;
      (*x_low)--;
      missing = 0;
    } else {
        missing++;
        (*x_low)--;
        if(missing >= N_MISSING) {
            done = 1;
            (*x_low)+=missing;
        }
    }
  }

  y = y_initial;
  (*x_high) = x;
  done = 0;
  // snake towards positive x (right)
  // while ((*x_high) < im->w - 1 && !done) {
  missing = 0;
  while ((*x_high) < im->h - 1 && !done) {

    if (check_color(im, (*x_high) + 1, y)) {
      (*x_high)++;
      missing = 0;
    // } else if (y < im->h - 1 && check_color(im, (*x_high) + 1, y++)) {
    } else if (y < im->w - 1 && check_color(im, (*x_high) + 1, y++)) {
      y++;
      missing = 0;
      (*x_high)++;
    } else if (y > 0 && check_color(im, (*x_high) + 1, y - 1)) {
      y--;
      (*x_high)++;
      missing = 0;
    } else {
        missing++;
        (*x_high)++;
        if(missing >= N_MISSING) {
            done = 1;
            (*x_high) -= missing;
        }
    }
  }
}
