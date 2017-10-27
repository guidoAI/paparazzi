/*
Copyright (c) 2017, Guido de Croon, TU Delft
All rights reserved.
*/

#include "act_fast.h"
#include "math.h"
#include "image.h"


#define MAX_AGENTS 1000;
struct agent agents[MAX_AGENTS];

/**
 * Do an ACT-FAST corner detection.
 * @param[in] *img The image to do the corner detection on
 * @param[in] threshold The threshold which we use for FAST9
 * @param[in] *num_corners reference to the amount of corners found, set by this function
 * @param[in] **ret_corners pointer to the array which contains the corners that were detected.
 * @param[in] n_agents The number of agents that will scan the image for corners
 * @param[in] n_time_steps The maximum number of time steps allowed for scanning
 * @param[in] long_step When there is not enough texture, the agent will take a long step to a next point of this length in pixels
 * @param[in] short_step When there is texture, the agent will follow the edge with this short step in pixels
 * @param[in] min_gradient The minimum gradient, in order to determine when to take a long or short step
*/
void act_fast(struct image_t *img, uint8_t fast_threshold, uint16_t *num_corners, struct point_t **ret_corners, uint16_t n_agents, uint16_t n_time_steps, float long_step, float short_step, int min_gradient, uint16_t max_corners) {

  /*
   * Procedure:
   * 1) initialize agent positions
   * 2) loop over the agents, moving and checking for corners
   */
  uint8_t fast_threshold = 30;

  // ensure that n_agents is never bigger than MAX_AGENTS
  n_agents = (n_agents < MAX_AGENTS) ? n_agents : MAX_AGENTS;

  int border = 4;

  // ***********************************
  // 1) initialize the agents' positions
  // ***********************************

  // grid sampling with a border:
  int init_border = 10;
  float GRID_ROWS = (int) ceil( sqrtf((float) max_corners) );
  float step_size_x = (img->w - 2*init_border) / (GRID_ROWS-1);
  float step_size_y = (img->h - 2*init_border) / (GRID_ROWS-1);

  int a = 0;
  for(int c = 0; c < GRID_ROWS; c++)
  {
    for(int r = 0; r < GRID_ROWS; r++)
    {
      agent ag = { (border + c * step_size_x), (border + r * step_size_y), 1};
      agents[a] = ag;
      a++;
      if(a == max_corners) break;
    }

    // only initialize a maximum of max_points agents.
    if(a == max_corners) break;
  }

  /* ********************************************************
   * 2) loop over the agents, moving and checking for corners
   * ********************************************************/

   for(int t = 0; t < n_time_steps; t++) {
       // loop over the agents
       for(int a = 0; a < max_corners; a++) {
           // only do something if the agent is active:
           if(agents[a].active) {
             // check if this position is a corner:
               uint16_t x = (uint16_t) agents[a].x;
               uint16_t y = (uint16_t) agents[a].y;
               if(fast9_detect_pixel(img, fast_threshold, x, y)) {
                   // we arrived at a corner, yeah!!!
                   agents[a].active = 0;
                   break;
               }
               else {
                   // make a step:
                   int gradient = image_gradient_pixel(struct image_t *img, struct point_t *loc, int method);
                   if(abs(gradient) >= min_gradient) {
                       // determine the angle and make a step in that direction:

                   }
                   else {
                       // make a step in the preferred direction:

                   }
               }
           }
       }
   }

}

