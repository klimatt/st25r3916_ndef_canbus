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

/*! \file timer.h
 *
 *  \brief SW Timer implementation header file
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
#include "platform.h"

/*
******************************************************************************
* GLOBAL MACROS
******************************************************************************
*/
#define timerIsRunning(t)            (!timerIsExpired(t))

/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/
 
 /*! 
 *****************************************************************************
 * \brief  Calculate Timer
 *  
 * This method calculates when the timer will be expired given the amount
 * time in milliseconds /a tOut.
 * Once the timer has been calculated it will then be used to check when
 * it expires.
 * 
 * \see timersIsExpired
 *
 * \param[in]  time : time/duration in Milliseconds for the timer
 *
 * \return u32 : The new timer calculated based on the given time 
 *****************************************************************************
 */
uint32_t timerCalculateTimer( uint16_t time );


/*! 
 *****************************************************************************
 * \brief  Checks if a Timer is Expired
 *  
 * This method checks if a timer has already expired.
 * Based on the given timer previously calculated it checks if this timer
 * has already elapsed
 * 
 * \see timersCalculateTimer
 *
 * \param[in]  timer : the timer to check 
 *
 * \return true  : timer has already expired
 * \return false : timer is still running
 *****************************************************************************
 */
bool timerIsExpired( uint32_t timer );


 /*! 
 *****************************************************************************
 * \brief  Performs a Delay
 *  
 * This method performs a delay for the given amount of time in Milliseconds
 * 
 * \param[in]  time : time/duration in Milliseconds of the delay
 *
 *****************************************************************************
 */
void timerDelay( uint16_t time );


/*! 
 *****************************************************************************
 * \brief  Stopwatch start
 *  
 * This method initiates the stopwatch to later measure the time in ms
 * 
 *****************************************************************************
 */
void timerStopwatchStart( void );


/*! 
 *****************************************************************************
 * \brief  Stopwatch Measure
 *  
 * This method returns the elapsed time in ms since the stopwatch was initiated
 * 
 * \return The time in ms since the stopwatch was started
 *****************************************************************************
 */
uint32_t timerStopwatchMeasure( void );
