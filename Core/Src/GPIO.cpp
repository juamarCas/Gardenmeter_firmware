#include "GPIO.h"

namespace periph{
	GPIO::GPIO(const config& _config){
		m_gpio     = _config.gpio;
		m_pin      = _config.pin;
		m_afrl     = _config.afrl_value;
		m_afrh     = _config.afrh_value;
		m_mode     = _config.mode;
		m_afrh_bit = _config.afrh_bit;
		m_afrl_bit = _config.afrl_bit;
		ApplyConfiguration();

	}

	void GPIO::SetConfig(GPIO::config& _config){
		m_gpio     = _config.gpio;
		m_pin      = _config.pin;
		m_afrl     = _config.afrl_value;
		m_afrh     = _config.afrh_value;
		m_mode     = _config.mode;
		m_afrh_bit = _config.afrh_bit;
		m_afrl_bit = _config.afrl_bit;
		ApplyConfiguration();
		
	}

	void GPIO::ApplyConfiguration(){
		//set mode
		m_gpio->MODER &= ~((1U << m_pin * 2) | (1U << m_pin * 2));
		if(m_mode != GPIO::Mode::Input) {
			m_gpio->MODER |= ((std::uint32_t)m_mode << (m_pin * 2));
		}

		if(m_afrh == 0x0U && m_afrl == 0x0U) return;

		m_gpio->AFR[0] |= (m_afrl << m_afrl_bit);
		m_gpio->AFR[1] |= (m_afrh << m_afrh_bit);
	}

	void GPIO::Toggle(){
		m_gpio->ODR ^= (1U << m_pin);
	}

	void GPIO::SetGPIO(const config& _config){
		_config.gpio->MODER &= ~((1U << _config.pin * 2) | (1U << _config.pin * 2));
		if(_config.mode != GPIO::Mode::Input) {
			_config.gpio->MODER |= ((std::uint32_t)_config.mode << (_config.pin * 2));
		}

		if(_config.afrh_value == 0x0U && _config.afrl_value == 0x0U) return;

		_config.gpio->AFR[0] |= (_config.afrl_value << _config.afrl_bit);
		_config.gpio->AFR[1] |= (_config.afrh_value << _config.afrh_bit);
	}
		
	
}
	