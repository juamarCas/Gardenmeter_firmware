/*
 * SHTC3.h
 *
 *  Created on: Sep 4, 2021
 *      Author: yiond
 *
 *      all this information is taken from the datasheet
 *      got some inspiration with this lib: https://github.com/belyalov/stm32-hal-libraries/blob/master/shtc3.c
 *      currently does not support clock sccretch
 */

#ifndef SRC_SMARTGARDEN_SHTC3_H_
#define SRC_SMARTGARDEN_SHTC3_H_
#include <cstdint>
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_i2c.h"
#include "Utils.h"


#define SHTC3_ID_MASK      0xF7C0U
#define SHTC3_PRODUCT_MASK 0X083FU;

/*addresses are inverted because of the way the function is made to send the addr
* and the fact this architecture is little endian*/
/*ex -> C8EF is id_addr and i'm sendig EFC8 that is the actual addr
 */
#define SHTC3_ID_CMD                 0xC8EFU
#define SHTC3_WAKE_UP_CMD            0x1735U
#define SHTC3_SLEEP_CMD              0x98B0U
#define SHTC3_NORMAL_MEASUREMENT_CMD 0xA27CU

class SHTC3 {
public:

private:
	I2C_HandleTypeDef * m_i2c;
	std::uint8_t m_addr;

	bool Write_reg16(std::uint16_t data);
	bool Write_reg(std::uint8_t * data, std::uint8_t size);
	bool Read_reg(std::uint8_t * data, std::uint16_t mem_addr ,std::uint8_t size_t, std::uint8_t size_r);
	bool Read_reg(std::uint8_t * data, std::uint8_t size_r);
	bool Read_reg16(std::uint8_t * data, std::uint16_t  mem_addr);
	bool CheckCRC(std::uint16_t val, std::uint8_t expected);

public:
	/*
	 * read the specific code for the sensor, not entirely, so the return should be: 0x0407
	 */
	std::uint16_t Read_ID(void);
	bool begin();
	void sleep();
	void wake_up();
	bool Read_sensor(float& temp, float& hum, std::uint16_t MEAS_CMD);
	//void Read_sensor(std::uint8_t &temp, std::uint8_t &hum);
	SHTC3(I2C_HandleTypeDef * i2c, std::uint8_t addr = 0x70);
};

#endif /* SRC_SMARTGARDEN_SHTC3_H_ */