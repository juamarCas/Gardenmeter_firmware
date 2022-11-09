#ifndef _UTILS_H
#define _UTILS_H

#include "stm32f3xx.h"
#include <cstdint>

/*
this uses TIMER4 for implementing the delay c:
*/



namespace utils{
	namespace delay{
		void Init();
		void ms(int ms); 
		void us(std::uint32_t us);
		
	}

	namespace math{
		long map(long value, long min_in, long max_in, long min_out, long max_out);
	}
	
}


#endif 