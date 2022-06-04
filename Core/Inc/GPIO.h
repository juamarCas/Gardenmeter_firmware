#ifndef _GPIO_H
#define _GPIO_H

#include <cstdint>
#include "stm32f3xx.h"


#define	INPUT  		0x0U
#define	OUTPUT  	0x1U
#define	ALTERNATE 0x2U 
#define	ANALOG    0x3U 

// #define HIGH 0x1U
// #define LOW  0x0U


namespace periph{
	
		// void set_pin(GPIO_TypeDef * gpio, std::uint32_t pin ,std::uint8_t mode); 
		// void set_pin_value(GPIO_TypeDef * gpio, std::uint32_t pin, std::int32_t state);
		// void set_afrl(GPIO_TypeDef * gpio, std::uint32_t val, std::uint32_t reg);
		// void set_afrh(GPIO_TypeDef * gpio, std::uint32_t val, std::uint32_t reg);
		class GPIO{
			public:
			enum Mode{
				Input = 0x0U, Output = 0x1U, Analog = 0x3U, Alternate = 0x2U
			};

			enum State{
				HIGH = 0x1U, LOW = 0x0U
			};

			typedef struct Config{
				GPIO_TypeDef * gpio;
				std::uint32_t pin;
				GPIO::Mode mode;
				std::uint32_t afrl_value = 0x0U;
				std::uint32_t afrh_value = 0x0U;
				std::uint32_t afrl_bit   = 0x0U;
				std::uint32_t afrh_bit   = 0x0U;
			} config;

			~GPIO(){}

			private:
				GPIO_TypeDef * m_gpio;
				std::uint32_t m_pin;
				std::uint32_t m_afrl;
				std::uint32_t m_afrh;

			public:
			/**
			 * @brief Construct a new GPIO object which you can set the gpio configuration
			 * 
			 * @param _configuration  structure with the gpio configuration
			 */
				GPIO(const config& _configuration);
			/**
			 * @brief toggles a gpio state
			 * 
			 */
				void Toggle();
			/**
			 * @brief Sets a gpio state (los or high)
			 * 
			 * @param state enumarator defining whic state wants the user
			 */
				void SetGPIO(GPIO::State state);
				
		};
		
}

#endif