/*
 * Copyright (C) CDW
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
 * @file "modules/first_stretch/first_stretch.h"
 * @author CDW
 * first stretch
 */

#ifndef FIRST_STRETCH_H
#define FIRST_STRETCH_H

// Global multithreaded
extern volatile int first_stretch_add;
extern volatile float first_stretch_psi;
extern volatile float first_stretch_certainty;


extern void first_stretch_init(void);
extern void first_stretch_periodic(void);

extern void first_stretch_start(void);


#endif

