#ifndef _GPIO_H
#define _GPIO_H

#include <cstdint>
#include "stm32f3xx.h"

namespace periph{
	
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
				std::uint32_t afrl_value          = 0x0U;
				std::uint32_t afrh_value          = 0x0U;
				std::uint32_t afrl_bit_position   = 0x0U;
				std::uint32_t afrh_bit_position   = 0x0U;
			} config;

			~GPIO(){}

			private:
				GPIO_TypeDef * m_gpio;
				std::uint32_t m_pin;
				std::uint32_t m_afrl;
				std::uint32_t m_afrh;
				std::uint32_t m_afrl_bit_position;
				std::uint32_t m_afrh_bit_position;

				GPIO::Mode m_mode;


				void ApplyConfiguration();

			public:
			/**
			 * @brief Construct a new GPIO object which you can set the gpio configuration
			 * 
			 * @param _configuration  structure with the gpio configuration
			 */
				GPIO(const config& _configuration);
				GPIO(){}
			/**
			 * @brief toggles a gpio state
			 * 
			 */
				void Toggle();
			/**
			 * @brief Sets a gpio state (low or high)
			 * 
			 * @param state enumarator defining whic state wants the user
			 */
				void SetGPIO(GPIO::State state);
			/**
			 * @brief Updates gpio configuration
			 * @param _config config structure representing the GPIO
			*/
				void SetConfig(GPIO::config& _config);

			/**
			 * @brief Sets a gpio without needing to create a class for it
			 * 
			 * @param _config configuration structure
			 */
				static void SetGPIO(const config& _config);

				
				
		};
		
}

#endif