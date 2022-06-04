#include "Utils.h"


//TODO finishing the delay
/*
TODO: 
			-Sendig data to esp32
			-i2c sensor implementation
			-esp32 deep sleep mode
			-stm32 sleep mode
			-Test soil moisture sensors
			-putting it all together
*/
namespace utils{
	namespace delay{
		void Init(){
			 SystemCoreClockUpdate(); 
		}
		
		void ms(int ms){
			SysTick->LOAD = 8000; 
			SysTick->VAL = 0; 
			SysTick->CTRL = (1U << 0U) | (1U << 2U); // CTRL ENABLE and CLK_source internal
			for(int i = 0; i < ms; i++){
				while((SysTick->CTRL & (1U << 16U)) == 0){}
			}
			
			SysTick->CTRL = 0; 
		}
		
		void us(std::uint32_t us){
			SysTick->LOAD = 8; 
			SysTick->VAL = 0; 
			SysTick->CTRL = (1U << 0U) | (1U << 2U); // CTRL ENABLE and CLK_source internal
			for(int i = 0; i < us; i++){
				while((SysTick->CTRL & (1U << 16U)) == 0){}
			}
			
			SysTick->CTRL = 0; 
		}
	}
}