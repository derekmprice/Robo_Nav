/**
  ******************************************************************************
  * @file    robot_config.h
  * @brief   Configuration parameters for the robot navigation system
  ******************************************************************************
  * @attention
  *
  * Edit this file to change target coordinates and anchor locations
  * without modifying other code files.
  *
  ******************************************************************************
  */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/*                       TARGET POSITION CONFIGURATION                        */
/* -------------------------------------------------------------------------- */
/*                                                                            */
/* Change these values to set where the robot should navigate to.             */
/* Coordinates are in UWB units (175 units = 1 meter)                         */
/* -------------------------------------------------------------------------- */

// Target position coordinates (UWB units)
#define TARGET_X_POSITION    200.0f
#define TARGET_Y_POSITION    200.0f

/* -------------------------------------------------------------------------- */
/*                         ANCHOR POSITIONS CONFIGURATION                      */
/* -------------------------------------------------------------------------- */
/*                                                                            */
/* Change these values to match the physical locations of your UWB anchors.   */
/* Coordinates are in UWB units (175 units = 1 meter)                         */
/* -------------------------------------------------------------------------- */

// Anchor 0 position (UWB units)
#define ANCHOR0_X            0.0f
#define ANCHOR0_Y            0.0f
#define ANCHOR0_NAME         "ANC 0"

// Anchor 1 position (UWB units)
#define ANCHOR1_X            270.0f
#define ANCHOR1_Y            0.0f
#define ANCHOR1_NAME         "ANC 1"

// Anchor 2 position (UWB units)
#define ANCHOR2_X            270.0f
#define ANCHOR2_Y            0.0f
#define ANCHOR2_NAME         "ANC 2"

/* -------------------------------------------------------------------------- */
/*                         NAVIGATION PARAMETERS                              */
/* -------------------------------------------------------------------------- */

// Conversion factors
#define UWB_UNITS_PER_METER  175.0f
#define DRIVE_TIME_PER_METER 1805.0f

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_CONFIG_H */