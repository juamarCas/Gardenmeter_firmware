/*
 * SHTC3.cpp
 *
 *  Created on: Sep 4, 2021
 *      Author: yiond
 */

#include "SHTC3.h"

SHTC3::SHTC3(I2C_HandleTypeDef * i2c, std::uint8_t addr) {
	m_addr = addr;
	m_i2c = i2c;
}

bool SHTC3::begin(){
	/*wake up device*/
	wake_up();
	/*read device*/

	if(Read_ID() != 0x0807){
		return false;
	}

	return true;
}

bool SHTC3::Read_sensor(float& temp, float& hum, std::uint16_t MEAS_CMD){
	std::uint8_t data[6] = {0};
	Write_reg((std::uint8_t *)&MEAS_CMD, 2);
	utils::delay::ms(14);
	//Read_reg(std::uint8_t * data, std::uint16_t mem_addr ,std::uint8_t size_t, std::uint8_t size_r);
	Read_reg(data, 6);



	std::int32_t t_temp = (std::int32_t)((std::uint32_t)data[0] << 8 | data[1]);
	std::uint32_t t_hum = ((std::uint32_t)data[3] << 8 | data[4]);

	if(!CheckCRC(t_temp, data[2])){
		return false;
	}

	if(!CheckCRC(t_hum, data[5])){
		return false;
	}
	/*
	 * hum = (stemp * 175.0f) / 65535.0f - 45.0f;
	 * temp = (shum * 100.0f) / 65535.0f;
	 */

	temp = (float)(17500 * (t_temp)/65535) - 4500;
	temp /= 100;
	hum = (float)(100 * t_hum / 65535);

	return true;

}


std::uint16_t SHTC3::Read_ID(void){
	std::uint8_t data[2] = {0};
	if(!Read_reg(data, SHTC3_ID_CMD, 2, 2)){
		return 0;
	}
	std::uint16_t res = data[0] << 8 | data[1];
	std::uint16_t id  = res & SHTC3_PRODUCT_MASK;

	return id;

}

bool SHTC3::CheckCRC(std::uint16_t val, std::uint8_t expected){
	std::uint8_t data[2] = {val >> 8, val & 0xFF};
	std::uint8_t crc = 0xFF;
	std::uint8_t poly = 0x31;


	for (uint8_t indi = 0; indi < 2; indi++) {
		crc ^= data[indi];
		for (uint8_t indj = 0; indj < 8; indj++) {
			if (crc & 0x80) {
				crc = (uint8_t)((crc << 1) ^ poly);
			} else {
				crc <<= 1;
			}
		}
	}

	if (expected ^ crc)	{
    return false;
	}
  return true;
}

void SHTC3::wake_up(){
	Write_reg16(SHTC3_WAKE_UP_CMD);
	utils::delay::us(250);
}

void SHTC3::sleep(){
	Write_reg16(SHTC3_SLEEP_CMD);
}

bool SHTC3::Write_reg16(std::uint16_t data){
	return (HAL_I2C_Master_Transmit(m_i2c, (m_addr << 1), (std::uint8_t *)&data , 2, HAL_MAX_DELAY) == HAL_OK);
}

bool SHTC3::Write_reg(std::uint8_t * data, std::uint8_t size){
	return (HAL_I2C_Master_Transmit(m_i2c, (m_addr << 1), data , size, HAL_MAX_DELAY) == HAL_OK);
}

bool SHTC3::Read_reg(std::uint8_t * data, std::uint16_t mem_addr ,std::uint8_t size_t, std::uint8_t size_r){
	HAL_I2C_Master_Transmit(m_i2c, (m_addr << 1), (std::uint8_t *)&mem_addr , size_t, HAL_MAX_DELAY);
	return (HAL_I2C_Master_Receive(m_i2c, (m_addr << 1), data, size_r, HAL_MAX_DELAY) == HAL_OK);
}

bool SHTC3::Read_reg(std::uint8_t * data, std::uint8_t size_r){
	return (HAL_I2C_Master_Receive(m_i2c, (m_addr << 1), data, size_r, HAL_MAX_DELAY) == HAL_OK);
}

