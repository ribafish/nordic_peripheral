/*
 * debug.h
 *
 *  Created on: 6 May 2018
 *      Author: gaspe
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include "nrf_log.h" //for nordic printf replacement
#include "clock.h"		// For clock


#ifndef __FILENAME__
#include <string.h>
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#endif

#if  NRF_LOG_ENABLED
#define debug_global(...) do { NRF_LOG_INFO(__VA_ARGS__ ); } while( 0 )
#define debug_error_global(...) do { LOG_INTERNAL(NRF_LOG_LEVEL_ERROR, NRF_LOG_ERROR_COLOR_CODE, __VA_ARGS__); } while( 0 )
#define debug_warning_global(...) do { LOG_INTERNAL(NRF_LOG_LEVEL_WARNING, NRF_LOG_WARNING_COLOR_CODE, __VA_ARGS__); } while( 0 )

#define debug_line_global(...) do { \
	NRF_LOG_INFO("%-25s:%4d:%8lu: ", __FILENAME__, __LINE__, clock_get_ms()); \
	NRF_LOG_INFO(__VA_ARGS__); } while ( 0 )

#define debug_errorline_global(...) do { \
	NRF_LOG_ERROR("%-25s:%4d:%8lu: ", __FILENAME__, __LINE__, clock_get_ms()); \
	NRF_LOG_ERROR(__VA_ARGS__); } while ( 0 )

#define debug_warningline_global(...) do { \
	NRF_LOG_WARNING("%-25s:%4d:%8lu: ", __FILENAME__, __LINE__, clock_get_ms()); \
	NRF_LOG_WARNING(__VA_ARGS__); } while ( 0 )

#else

#include "app_util.h"
#include "app_util_platform.h"

#define debug_global(...) 				do { } while ( 0 )
#define debug_error_global(...)			do { } while ( 0 )
#define debug_warning_global(...)		do { } while ( 0 )

#define debug_line_global(...)			do { } while ( 0 )
#define debug_errorline_global(...)		do { } while ( 0 )
#define debug_warningline_global(...)	do { } while ( 0 )
#endif



#ifndef NULL
#define NULL ((void *)0)
#endif

#endif /* DEBUG_H_ */
