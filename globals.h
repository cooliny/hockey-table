/*
 * Hockey table: Global variables used in other functions
 * globals.h
 */

#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>

 extern volatile uint8_t minutes;
 extern volatile uint8_t seconds;

 // Period tracking
 extern volatile uint8_t period;
 extern volatile uint8_t period_over;

 // Score tracking
 extern volatile uint8_t home_score;
 extern volatile uint8_t away_score;

 // Ready flags
 extern volatile int home_ready_flag;
 extern volatile int away_ready_flag;

#endif
