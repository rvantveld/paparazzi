/*
 * Copyright (C) 2012-2015 Group 1 AE4317 TU Delft
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

/**
 * @file modules/computer_vision/obstacle_avoidance/obstacle_avoidance.h
 * @brief Detect obstacles using opticflow and color counting
 *
 * Obstacle detection module using opticflow and color counting
 * The obstacle is detected if a certain threshold is reached
 * for either opticflow or color count
 * Result is stored in variable to be used by autopilot
 */

#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

// Include opticflow calculator and color count code
#include "opticflow_calculator.h"
#include "color_count.h"

// Module functions
extern void obstacle_avoidance_init(void);
extern void obstacle_avoidance_run(void);
extern void obstacle_avoidance_start(void);
extern void obstacle_avoidance_stop(void);

// Color count variables
extern uint8_t color_lum_min;
extern uint8_t color_lum_max;

extern uint8_t color_cb_min;
extern uint8_t color_cb_max;

extern uint8_t color_cr_min;
extern uint8_t color_cr_max;

// Variable used to communicate with flight plan (set to 0 by flight plan if detected!)
extern int obstacleDetected;

#endif /* OBSTACLE_AVOIDANCE_H */
