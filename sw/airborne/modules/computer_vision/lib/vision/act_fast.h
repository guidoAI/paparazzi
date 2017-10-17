/*
Copyright (c) 2017, Guido de Croon, TU Delft
All rights reserved.
*/

/**
 * @file modules/computer_vision/lib/vision/act_fast.h
 * @brief Finds corners in an image by actively scanning the image. This method is inspired by the work in:
 * de Croon, G.C.H.E., and Nolfi, S. (2013, May). Act-corner: Active corner finding for optic flow determination. In Robotics and Automation (ICRA), 2013 IEEE International Conference on (pp. 4679-4684). IEEE.
 *
 * The main idea of this particular implementation, called ACT-FAST, is that actively scanning the image allows to:
 * 1. Skip uniform areas in the image. If these areas are contiguous, many non-corners will never be evaluated.
 * 2. Follow edges. Typically, following an edge will bring an agent to a corner.
 * These two simple rules lead to a significant lower number of corner evaluations, while still detecting a reasonable number of corners.
 * Each step of the agent starts by classifying the agent location as a corner or not with FAST.
 *
 * For bigger images, the computational difference between ACT-FAST and normal FAST become significant.
 */

#ifndef ACT_FAST_H
#define ACT_FAST_H

struct agent
{
        float x;
        float y;
        int active;
};

#include "std.h"
#include "lib/vision/image.h"

void act_fast(struct image_t *img, uint8_t fast_threshold, uint16_t *num_corners, struct point_t **ret_corners, uint16_t n_agents, uint16_t n_time_steps, float long_step, float short_step, int min_gradient, uint16_t max_corners);

#endif
