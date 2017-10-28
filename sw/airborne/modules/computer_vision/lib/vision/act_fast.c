/*
Copyright (c) 2017, Guido de Croon, TU Delft
All rights reserved.
*/

#include "fast_rosten.h"
#include "act_fast.h"
#include "math.h"
#include "image.h"


#define MAX_AGENTS 1000
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
void act_fast(struct image_t *img, uint8_t fast_threshold, uint16_t *num_corners, struct point_t **ret_corners, uint16_t n_agents, uint16_t n_time_steps, float long_step, float short_step, int min_gradient) {

  /*
   * Procedure:
   * 1) initialize agent positions
   * 2) loop over the agents, moving and checking for corners
   */

  // there is no need to have an adaptive threshold, as we control the number of corners by means of the number of agents
  // we only need a corner that is "strong" enough to be tracked:
  // uint8_t fast_threshold = 30;


  // method to determine the gradient:
  // 0 = simple (-1, 0, 1)
  // 1 = Sobel
  int gradient_method = 1;

  // ensure that n_agents is never bigger than MAX_AGENTS
  n_agents = (n_agents < MAX_AGENTS) ? n_agents : MAX_AGENTS;

  int border = 4;

  // ***********************************
  // 1) initialize the agents' positions
  // ***********************************

  // grid sampling with a border:
  int init_border = 10;
  float GRID_ROWS = (int) ceil( sqrtf((float) n_agents) );
  float step_size_x = (img->w - 2*init_border) / (GRID_ROWS-1);
  float step_size_y = (img->h - 2*init_border) / (GRID_ROWS-1);

  int a = 0;
  float px,py,pnorm;
  for(int c = 0; c < GRID_ROWS; c++)
  {
    for(int r = 0; r < GRID_ROWS; r++)
    {
      // px, py represent the preferred direction of the agent when there is no texture
      // here we initialize it differently for each agent:
      // TODO: don't we have a randf function in Paparazzi?
      px = ((float) (rand() % 10000)) / 10000.0f;
      py = ((float) (rand() % 10000)) / 10000.0f;
      pnorm = sqrtf(px*px+py*py);
      struct agent ag = { (border + c * step_size_x), (border + r * step_size_y), 1, px/pnorm, py/pnorm};
      agents[a] = ag;
      a++;
      if(a == n_agents) break;
    }

    // only initialize a maximum of n_agents agents.
    if(a == n_agents) break;
  }

  /* ********************************************************
   * 2) loop over the agents, moving and checking for corners
   * ********************************************************/

  // gradient
  int dx, dy;

  // loop over all time steps:
   for(int t = 0; t < n_time_steps; t++) {
       // loop over the agents
       for(a = 0; a < n_agents; a++) {
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
                 struct point_t loc = {agents[a].x, agents[a].y};
                 image_gradient_pixel(img, &loc, gradient_method, &dx, &dy);
                 int gradient = (abs(dx) + abs(dy)) / 2;
                 if(abs(gradient) >= min_gradient) {
                     // determine the angle and make a step in that direction:
                     float norm_factor = sqrtf((float) (dx*dx + dy*dy));
                     agents[a].x += (dy / norm_factor) * short_step;
                     agents[a].y += (dx / norm_factor) * short_step;
                 }
                 else {
                     // make a step in the preferred direction:
                     agents[a].x += agents[a].preferred_dir_x * long_step;
                     agents[a].y += agents[a].preferred_dir_y * long_step;
                 }
             }

             // let the agent move over the image in a toroid world:
             if(agents[a].x > img->w - border) {
                 agents[a].x = border;
             }
             else if(agents[a].x < border) {
                 agents[a].x = img->w - border;
             }
             if(agents[a].y > img->h - border) {
                  agents[a].y = border;
              }
              else if(agents[a].y < border) {
                  agents[a].y = img->h - border;
              }
           }
       }
   }

   // Transform agents to corners:
   // uint16_t *num_corners, struct point_t **ret_corners
   (*num_corners) = 0;
   for(a = 0; a < n_agents; a++) {

       // for active agents do a last check on the new position:
       if(agents[a].active) {
          // check if the last step brought the agent to a corner:
          uint16_t x = (uint16_t) agents[a].x;
          uint16_t y = (uint16_t) agents[a].y;
          if(fast9_detect_pixel(img, fast_threshold, x, y)) {
              // we arrived at a corner, yeah!!!
              agents[a].active = 0;
          }
       }

       // if inactive, the agent is a corner:
       if(!agents[a].active) {
           (*ret_corners)[(*num_corners)].x = (uint32_t) agents[a].x;
           (*ret_corners)[(*num_corners)].y = (uint32_t) agents[a].y;
           (*num_corners)++;
       }
   }
}

