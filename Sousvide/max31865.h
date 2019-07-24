/*
 * max31865.h
 *
 * Created: 10.06.2019 20:57:39
 *  Author: Саша
 */ 


#ifndef MAX31865_H_
#define MAX31865_H_

/*==========================================*/
/* C++ compatible */

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================*/


double max31865_temperature(void);
uint16_t readRTD (void);
void max31865_setup(void);
double max31865_plus_temperature(void);
uint8_t max31865_readFault(void);

/*==========================================*/
/* C++ compatible */

#ifdef __cplusplus
}
#endif








#endif /* MAX31865_H_ */