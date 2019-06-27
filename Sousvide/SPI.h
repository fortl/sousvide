/*
 * SPI.h
 *
 * Created: 10.06.2019 21:16:05
 *  Author: Саша
 */ 


#ifndef SPI_H_
#define SPI_H_


/*==========================================*/
/* C++ compatible */

#ifdef __cplusplus
extern "C" {
	#endif

	/*==========================================*/


	uint8_t SPI_ReadByte(uint8_t data);
	void SPI_WriteByte(uint8_t data);
	void SPI_Init(void);

	/*==========================================*/
	/* C++ compatible */

	#ifdef __cplusplus
}
#endif









#endif /* SPI_H_ */