/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
 * AeroBeam 3DOF Quadrocopter Code
 * Developer: Onur Yildirim
 * 
 * 31/10/2017 Tuesday 21:30pm
 * 
 * 
 */

/*  Changelog:
 *  
 *  ------------------------------------------------------------------
 *  Code Version: 0.1 31-Oct-2017
 *  - Initial code build
 */
 
#ifndef __AEROBEAM_H
#define __AEROBEAM_H

#include <stdint.h>
#include <stdbool.h>

extern uint8_t CDCReceiveNewData;
extern uint32_t CDCReceiveLength;
extern uint8_t usart3_rx_data[2];


uint8_t AB_setup(void);
uint8_t AB_loop_while(void);
uint8_t AB_timer_interrupt(void);
uint8_t AB_loop_1000Hz(void);
uint8_t AB_loop_100Hz(void);
uint8_t AB_loop_50Hz(void);
uint8_t AB_loop_20Hz(void);
uint8_t AB_loop_10Hz(void);
uint8_t AB_loop_4Hz(void);
uint8_t AB_loop_1Hz(void);
uint8_t AB_animation(void);
uint8_t AB_pwm_update(uint16_t motr1, 
											uint16_t motr2, 
											uint16_t motr3, 
											uint16_t motr4);
uint8_t AB_i2c_read(uint8_t device_address_7bit, uint8_t reg_address, uint8_t *read_buffer, uint8_t read_lentgh);
uint8_t AB_i2c_write(uint8_t device_address_7bit, uint8_t reg_address, uint8_t *write_buffer, uint8_t write_lentgh);
uint8_t AB_bno055_init(void);
uint8_t AB_bno055_read(float *x, float *y, float *z);
uint8_t calculateLRC(uint8_t *data, uint8_t data_size);
uint8_t AB_ros_transmit(void);
uint8_t AB_ros_receive(void);
uint8_t AB_usart3_interrupt(void);
uint8_t AB_bno055_status_check(void);
uint8_t AB_CDC_receive_check(void);
uint8_t AB_USART3_receive_check(void);
uint8_t AB_calculate_pwm(void);
float AB_limit(float val, float lim);


/*  <<<<<<<<<<<<<< Additional PFP >>>>>>>>>>>>>>> */
void Tprint( void);
void Receive_Check( void);
bool Stop( uint8_t stime);
bool is_Ros_Disconnected( void);

#endif /* __AEROBEAM_H */

