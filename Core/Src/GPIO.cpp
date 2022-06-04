#include "GPIO.h"

namespace periph{
	
		// void set_pin(GPIO_TypeDef * gpio, std::uint32_t pin ,std::uint8_t m){
		// 	gpio->MODER &= ~((1U << pin * 2) | (1U << (pin * 2 + 1))); 
		// 	if(m == INPUT) return; 		
		// 	gpio->MODER |= (m << (pin * 2)); 								
		//  }
			
		// 	void set_pin_value(GPIO_TypeDef * gpio, std::uint32_t pin, std::int32_t state){	
		// 		if(state == LOW) gpio->ODR &= ~(1U << pin);
		// 		else gpio->ODR |= (1U << pin); 		
		// 	}
	
		// 	void set_afrl(GPIO_TypeDef * gpio, std::uint32_t val, std::uint32_t reg){
		// 		gpio->AFR[0] |= (val << reg);  
		// 	}
		
		// 	void set_afrh(GPIO_TypeDef * gpio, std::uint32_t val, std::uint32_t reg){
		// 		gpio->AFR[1] |= (val << reg);  
		// 	}
	GPIO::GPIO(const config& _config){
		m_gpio = _config.gpio;
		m_pin  = _config.pin;
		m_afrl = _config.afrl_value;
		m_afrh = _config.afrh_value;

		//set mode
		m_gpio->MODER &= ~((1U << _config.pin * 2) | (1U << _config.pin * 2));
		if(_config.mode != GPIO::Mode::Input) {
			m_gpio->MODER |= ((std::uint32_t)_config.mode << (_config.pin * 2));
		}

		if(m_afrh == 0x0U && m_afrl == 0x0U) return;

		m_gpio->AFR[0] |= (m_afrl << _config.afrl_bit);
		m_gpio->AFR[1] |= (m_afrh << _config.afrh_bit);

	}

	void GPIO::Toggle(){
		m_gpio->ODR ^= (1U << m_pin);
	}
		
	
}
	