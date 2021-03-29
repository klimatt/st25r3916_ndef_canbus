/******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
******************************************************************************/
/*
 *      PROJECT:   ST25R391x firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file timer.c
 *
 *  \brief SW Timer implementation
 *
 *  \author Gustavo Patricio
 *
 *   This module makes use of a System Tick in millisconds and provides
 *   an abstraction for SW timers
 *
 */

/*
******************************************************************************
* INCLUDES
******************************************************************************
*/
#include "timer.h"


/*
******************************************************************************
* LOCAL DEFINES
******************************************************************************
*/

/*
******************************************************************************
* LOCAL VARIABLES
******************************************************************************
*/

static uint32_t timerStopwatchTick;

/*
******************************************************************************
* GLOBAL FUNCTIONS
******************************************************************************
*/


/*******************************************************************************/
uint32_t timerCalculateTimer( uint16_t time )
{  
  return (platformGetSysTick() + time);
}


/*******************************************************************************/
bool timerIsExpired( uint32_t timer )
{
  uint32_t uDiff;
  int32_t sDiff;
  
  uDiff = (timer - platformGetSysTick());   /* Calculate the diff between the timers */
  sDiff = uDiff;                            /* Convert the diff to a signed var      */
  /* Having done this has two side effects: 
   * 1) all differences smaller than -(2^31) ms (~25d) will become positive
   *    Signaling not expired: acceptable!
   * 2) Time roll-over case will be handled correctly: super!
   */
  
  /* Check if the given timer has expired already */
  if( sDiff < 0 )
  {
    return true;
  }
  
  return false;
}


/*******************************************************************************/
void timerDelay( uint16_t tOut )
{
  uint32_t t;
  
  /* Calculate the timer and wait blocking until is running */
  t = timerCalculateTimer( tOut );
  while( timerIsRunning(t) );
}


/*******************************************************************************/
void timerStopwatchStart( void )
{
  timerStopwatchTick = platformGetSysTick();
}


/*******************************************************************************/
uint32_t timerStopwatchMeasure( void )
{
  return (uint32_t)(platformGetSysTick() - timerStopwatchTick);
}

