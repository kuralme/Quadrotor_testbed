/**
  ******************************************************************************
  * File Name          : aerobeam.c
  * Description        : Custom Functions
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
	    

/* 
 * PIN Definitions:
 * PB0 :  Green LED (LD1)			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
 * PB7 :  Blue  LED (LD2)			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
 * PB14:  Red   LED (LD3)			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
 * 
 * PE0 :  Buzzer								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);
 * PC13:  Blue User Button			HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)==GPIO_PIN_SET
 * 
 * USB CDC Transmit template:
 *	   cdc_tr_len=sprintf((char *)cdc_tr_buff,"EXAMPLE TEXT: %d \n",EXAMPLE_VARIABLE1);
  	   CDC_Transmit_FS(cdc_tr_buff,cdc_tr_len);
 * 
 * USART3 Transmit template (over ST-LINK USB):  //230400 baud
 *		 cdc_tr_len=sprintf((char *)cdc_tr_buff,"EXAMPLE TEXT: %d \n",EXAMPLE_VARIABLE1);
       HAL_UART_Transmit(&huart3,cdc_tr_buff,cdc_tr_len,100);
 *		
 *	
 */


#include "aerobeam.h"
#include "stm32f7xx_hal.h"
#include "usbd_cdc_if.h"
#include "tim.h"
#include "i2c.h"
#include "usart.h"
#include "BNO055.h"

// UART variables
uint8_t CDCReceiveNewData=0;
uint32_t CDCReceiveLength=0;
uint8_t cdc_tr_buff[128];
uint8_t cdc_tr_len = 0;
uint8_t usart3_rx_data[2];

// Motor PWM variables (Value Between: 0 - 1000)
uint16_t motor1=0;
uint16_t motor2=0;
uint16_t motor3=0;
uint16_t motor4=0;

// Sensor reading variables
float euler_x, euler_y, euler_z;
//float quat_w, quat_x, quat_y, quat_z;

// Tprint, Recieve_check and is_Ros_Disconnected variables
uint8_t stop_count=0, usart_count=0;
uint16_t ros_count=0;
bool emergency;

// Ros communication variables
float heartbeat=0, heartbeat_prev;
float gain=0;
uint16_t command =0; //1100: safe, 1500: armed, 1000:emergency
uint16_t ros_rcv_err_count=0;
uint8_t motors_armed = 0;
uint8_t ros_tx_size = 28;
uint8_t ros_tx_buf[33];
uint8_t ros_tx_data[28];


//************ PID Coefficients *********************************/
float roll_KP  = 12,			roll_KI  = 2.8,		  roll_KD  = 2.3;
float pitch_KP = 8,		    pitch_KI = 2.0,		  pitch_KD = 2.0;
float yaw_KP   = 25,			yaw_KI   = 0.8,		  yaw_KD   = 0.1;
/****************************************************************/

//************ Other PID Variables ******************************/
float roll_P  = 0,			roll_I  = 0,			roll_D  = 0;
float pitch_P = 0,		  pitch_I = 0,		  pitch_D = 0;
float yaw_P   = 0,			yaw_I   = 0,			yaw_D   = 0;
float roll_I_limit = 150;
float pitch_I_limit = 150;
float yaw_I_limit = 200;
float roll_error = 0,     pitch_error = 0,     yaw_error = 0;
float roll_error_old = 0, pitch_error_old = 0, yaw_error_old = 0;
float roll_error_sum = 0, pitch_error_sum = 0, yaw_error_sum = 0;
float yaw_err_init = 0;
float yaw_jump_prevention_limit = 300;

// Referance values
uint16_t roll_ref =0;
uint16_t pitch_ref=0;
uint16_t yaw_ref  =0;
uint16_t thr_ref  =400; // motor initial torque
float yaw_ref_sum =0;
float yaw_init =0;
float roll_motor=0, pitch_motor=0, yaw_motor=0;
float dt=0.01;

float board_roll_offset= -5;  // -5
float board_pitch_offset= 4;  // 4
float euler_z_sum, euler_z_old;

/********************************************************************************/
/*!
*   @brief   PID Calculation and PWM output
*/
/********************************************************************************/
uint8_t AB_calculate_pwm(void)
{
 /******************** Error calculation ***************************/
	euler_x += board_roll_offset;
	euler_y += board_pitch_offset;
	
  roll_error = ((roll_ref)-1500)/20.0 - euler_x;
	pitch_error = (1500 - (pitch_ref))/20.0 - euler_y;
	
   
	// Yaw angle control:  referance scale-> +- 180°
  if(euler_z > 180) euler_z -= 360;
	if ( (euler_z - euler_z_old)> -5 && (euler_z - euler_z_old)< 5 ) euler_z_sum += euler_z - euler_z_old;
	euler_z_old = euler_z;
	
	if(yaw_ref == 0 || motors_armed == 0){
		yaw_ref_sum = euler_z;
		euler_z_sum = 0;
	  yaw_error = 0;
	}
	else{
		yaw_ref_sum += (float)((yaw_ref)-1500)/1110.0F;
	  yaw_error =  yaw_ref_sum - euler_z;
		if(yaw_error > 180) yaw_error -= 360;
    else if(yaw_error < -180) yaw_error += 360;
	}
	

	
 /******************** PID for roll angle **************************/
  roll_P = roll_KP*roll_error;
	roll_I = roll_KI*roll_error_sum*dt;
	roll_D = (roll_KD*(euler_x-roll_error_old))/dt;
	roll_error_sum += roll_error;
	roll_error_sum = AB_limit(roll_error_sum,roll_I_limit);
	roll_error_old = euler_x;
	roll_motor = (roll_P + roll_I - roll_D);
	

 /******************** PID for pitch angle *************************/
  pitch_P = pitch_KP*pitch_error;
	pitch_I = pitch_KI*pitch_error_sum*dt;
	pitch_D = (pitch_KD*(euler_y-pitch_error_old))/dt;
	pitch_error_sum += pitch_error;
	pitch_error_sum = AB_limit(pitch_error_sum,pitch_I_limit);
	pitch_error_old = euler_y;
	pitch_motor = (pitch_P + pitch_I - pitch_D);

  
 /******************** PID for yaw angle ***************************/
// Yaw P is limited in case of sharp angle changes (a.k.a. jump prevention)
	yaw_P = AB_limit((yaw_KP*yaw_error),yaw_jump_prevention_limit);
	yaw_I = yaw_KI*yaw_error_sum*dt;
	yaw_D = (yaw_KD*(euler_z-yaw_error_old))/dt;
	yaw_error_sum += yaw_error;
	yaw_error_sum = AB_limit(yaw_error_sum,yaw_I_limit);
	yaw_error_old = euler_z;
	yaw_motor = (yaw_P + yaw_I - yaw_D);
	
	
 /*** PID values is added/subtracted to motor referance torque *****/
	motor1 = thr_ref - yaw_motor + roll_motor;
	motor3 = thr_ref - yaw_motor - roll_motor;
	
	motor2 = thr_ref + yaw_motor + pitch_motor;
	motor4 = thr_ref + yaw_motor - pitch_motor;
	
 /**** Motor torque limitations *******/
  // North motor
  if(motor1 < 50) motor1 = 50;
	if(motor1 > 800) motor1 = 800;
	
  // East motor
  if(motor2 < 50) motor2 = 50;
	if(motor2 > 800) motor2 = 800;
		
  // South motor
  if(motor3 < 50) motor3 = 50;
	if(motor3 > 800) motor3 = 800;
		
	// West motor
  if(motor4 < 50) motor4 = 50;
	if(motor4 > 800) motor4 = 800;
	
  if(emergency){
	motors_armed=0;
	command = 1000;
  emergency=Stop(15); // Stopping motors for <n> seconds - Stop(n)
	}
  
  /**** Final pwm values applied to the motors *******/
	if (motors_armed==1) AB_pwm_update(motor1,motor2,motor3,motor4);
	else AB_pwm_update(0,0,0,0);
	 
	return 0;
}

uint8_t AB_setup(void)
{
	AB_animation();
  AB_pwm_update(0,0,0,0);
	
	BNO055_Start();	//BNO055 initialization
	
	Transmit((char *)cdc_tr_buff, "\r\nReading Euler angles:\r\n\r\n");
	HAL_Delay(500);
	
	return 0;
}


uint8_t AB_loop_while(void)
{
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)==GPIO_PIN_SET)
	{
		if (motors_armed==1)
		{
			motors_armed=0;
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
			while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)==GPIO_PIN_SET);
		}
	}
	if (command == 1500  && motors_armed == 0 )
	{
	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
		HAL_Delay(2000);
		motors_armed=1;
	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);
	}
  else if (command < 1050)
	{
	  motors_armed=0;
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
	}	
  return 0;
}


uint8_t AB_loop_1Hz(void)
{
  return 0;
}

uint8_t AB_loop_4Hz(void)
{	
  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);	
	return 0;
}

uint8_t AB_loop_10Hz(void)
{ 	
	Receive_Check();  // Terminal(Doclight): check data sent from terminal
	if (emergency==false) Tprint();  // Terminal: print method call
	return 0;
}

uint8_t AB_loop_20Hz(void)
{
	AB_ros_transmit();
	if(emergency==false) emergency = is_Ros_Disconnected();
	return 0;
}

uint8_t AB_loop_50Hz(void)
{
	
	return 0;
}


uint8_t AB_loop_100Hz(void)
{
	BNO055_readEuler(&euler_x, &euler_y, &euler_z);	 // Read euler angles from the sensor
	AB_calculate_pwm(); // Implement PID control to the motors	 
	return 0;
}

uint8_t AB_loop_1000Hz(void)
{
	AB_CDC_receive_check();
	return 0;
}




uint16_t loop_counter=0;

uint8_t AB_timer_interrupt(void)
{
	loop_counter++;
	AB_loop_1000Hz();
	if (loop_counter%10  == 0 ) AB_loop_100Hz();
	if (loop_counter%20  == 0 ) AB_loop_50Hz();
	if (loop_counter%50  == 0 ) AB_loop_20Hz();
	if (loop_counter%100 == 0 ) AB_loop_10Hz();
	if (loop_counter%250 == 0 ) AB_loop_4Hz();
	if (loop_counter>=1000) 
	{
		AB_loop_1Hz();
		loop_counter=0;
	}
	return 0;
}

uint8_t AB_animation(void)
{
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET); HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET); HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET); HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);  HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);  HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);  HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);HAL_Delay(10);
	return 0;
}

uint8_t AB_pwm_update(uint16_t motr1, 
											uint16_t motr2, 
											uint16_t motr3, 
											uint16_t motr4)
{
  TIM4->CCR1 = motr1+1000;
	TIM4->CCR2 = motr2+1000;
	TIM4->CCR3 = motr3+1000;
	TIM4->CCR4 = motr4+1000;
  return 0;
}


uint8_t AB_i2c_read(uint8_t device_address_7bit, uint8_t reg_address, uint8_t *read_buffer, uint8_t read_lentgh)
{
  HAL_I2C_Master_Transmit(&hi2c1,device_address_7bit<<1,&reg_address,1,100);
	HAL_I2C_Master_Receive(&hi2c1,device_address_7bit<<1,read_buffer,read_lentgh,100);
	return 0;
}

uint8_t AB_i2c_write(uint8_t device_address_7bit, uint8_t reg_address, uint8_t *write_buffer, uint8_t write_lentgh)
{
	uint8_t i2c_temp[64]={0};
	i2c_temp[0]=reg_address;
	for(uint8_t t=1;t<=write_lentgh;t++)
	{
	  i2c_temp[t]=write_buffer[t-1];
	}
  HAL_I2C_Master_Transmit(&hi2c1,device_address_7bit<<1,i2c_temp,write_lentgh+1,100);
	return 0;
}


uint8_t AB_CDC_receive_check(void)
{	
  if(CDCReceiveNewData==1)
	{
		CDCReceiveNewData=0;
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
		AB_ros_receive();
		//yaw_KD = gain;
		//pitch_KP = gain;
		//cdc_tr_len=sprintf((char *)cdc_tr_buff,"H:%5f C:%4d R:%4d P:%4d T:%4d Y:%4d  G:%2.2f E:%3d  \r\n",heartbeat,command,roll_ref,pitch_ref,thr_ref,yaw_ref,gain,ros_rcv_err_count);
	  //HAL_UART_Transmit(&huart3,cdc_tr_buff,cdc_tr_len,100);
	}
	return 0;
}



uint8_t AB_usart3_interrupt(void)
{	
	UserRxBufferFS[usart_count]= usart3_rx_data[0];
	usart_count++;
	return 0;
}


float AB_limit(float val, float lim)
{
  if(val > lim) val = lim;
	if(val < -lim) val = -lim;
	return val;
}


uint8_t calculateLRC(uint8_t *data, uint8_t data_size)
{
    uint8_t LRC = 0x00;
    for (uint8_t t = 0; t < data_size; ++t) LRC ^= data[t];
    return LRC;
}



uint8_t AB_ros_transmit(void)
{
	float m1,m2,m3,m4;
	m1=motor1; m2=motor2;
	m3=motor3; m4=motor4;
	memcpy(&ros_tx_data    ,&euler_x,4);
	memcpy(&ros_tx_data[4] ,&euler_y,4);
	memcpy(&ros_tx_data[8] ,&euler_z,4);
	memcpy(&ros_tx_data[12],&m1     ,4);
	memcpy(&ros_tx_data[16],&m2     ,4);
	memcpy(&ros_tx_data[20],&m3     ,4);
	memcpy(&ros_tx_data[24],&m4     ,4);
	uint8_t r = 0;
  ros_tx_buf[r++] = 0xA5;
  ros_tx_buf[r++] = ros_tx_size;
  for(uint8_t j = 0; j < ros_tx_size; ++j) ros_tx_buf[r++] = ros_tx_data[j];
  ros_tx_buf[r++] = calculateLRC(ros_tx_data, ros_tx_size);
  ros_tx_buf[r++] = 0x5A;
	ros_tx_buf[r++] = 0x0A;
	CDC_Transmit_FS(ros_tx_buf,ros_tx_size+5);
	return 0;
}

/* 0xA5,0x18,0x__,0x__,0x__,0x__,0x__,0x__,0x__,0x__,0x__,0x__,0x__,0x__,0x__,0x__,0x__,0x__,
 * strt|len |heartbeat (4 bytes)| command (4 bytes) | roll    (4 bytes) | pitch   (4 bytes) |
 * 0x__,0x__,0x__,0x__ ,0x__,0x__,0x__,0x__,0x00,0x5A,0x0A
 * throttle  (4 bytes) | yaw     (4 bytes) |LRC | end| lf
 */

uint8_t AB_ros_receive(void)
{
	uint8_t ros_rx_data[28]={0}, s=0;
	float roll_t,pitch_t, yaw_t, thr_t,comm_t;
	if(UserRxBufferFS[0]==0xA5 && UserRxBufferFS[31]==0x5A)
	{
	  for (s=0;s<28;s++) ros_rx_data[s] = UserRxBufferFS[s+2];
		if (UserRxBufferFS[30]==calculateLRC(ros_rx_data, UserRxBufferFS[1]))
		{
		  memcpy(&heartbeat ,&ros_rx_data     ,4);
			memcpy(&comm_t    ,&ros_rx_data[4]  ,4);
			memcpy(&roll_t    ,&ros_rx_data[8]  ,4);
			memcpy(&pitch_t   ,&ros_rx_data[12] ,4);
			memcpy(&thr_t     ,&ros_rx_data[16] ,4);
			memcpy(&yaw_t     ,&ros_rx_data[20] ,4);
			memcpy(&gain      ,&ros_rx_data[24] ,4);
			command = comm_t;  roll_ref =roll_t;
			pitch_ref = pitch_t; yaw_ref  =yaw_t;
			thr_ref   = thr_t-1000;
		}
		else ros_rcv_err_count++;
	}
	else ros_rcv_err_count++;
	return 0;
}



/********************************************************************************/
/*!
*   @brief   Print function for terminals (For ex. Doclight)
*/
/********************************************************************************/
void Tprint()
{	
  if( emergency )
  {
	 /* <<<<<<<<<<<<<<<<<<<<<<<<<  Emergency stop print >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
	 Transmit((char *)cdc_tr_buff, "\r\n<<< Emergency Mode >>>\r\nStopping motors for %d sec...\r\n", stop_count);
	}	
	/* <<<<<<<<<<<<<<<<<<<<<<<<<  Print out euler angles  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
	Transmit((char *)cdc_tr_buff, "RPY= %3.2f    %3.2f    %3.2f\r\n", euler_x, euler_y, euler_z);
}



/********************************************************************************/
/*!
*   @brief   Data check received from terminal
*/
/********************************************************************************/
void Receive_Check()
{
	//  If received "stop" -> Emergency = true
	if( UserRxBufferFS[0]=='s'
	 && UserRxBufferFS[1]=='t'
	 && UserRxBufferFS[2]=='o'
	 && UserRxBufferFS[3]=='p')
	 {
	  emergency=true;
	 }
	// Buffer cleaning
	for(int i=0;i<10;i++){
		UserRxBufferFS[i]='\0';
		usart3_rx_data[i]='\0';
	}
	usart_count=0;
}




/********************************************************************************/
/*!
*   @brief   Emergency function to stop motors
*/
/********************************************************************************/
bool Stop( uint8_t stime)
{
	if(stop_count==0) stop_count = stime+1;
	stop_count--;
	
	// Stops motors for <stop_count> seconds
	AB_pwm_update(0,0,0,0);
	HAL_Delay(1000);
	Tprint();
	
	if(stop_count==0)
	{
	 motors_armed=1;
	 command = 1000;
	 return false;
	}
	else
	 return true;
}


/***********************************************************************************/
/*!
*   @brief   Ros communication check by checking heartbeat (a.k.a. ros time counter)
*/
/***********************************************************************************/
bool is_Ros_Disconnected( void)
{
	ros_count++;
	
	// Heartbeat check every 2 sec
  if( ros_count%40 == 0){
		// If heartbeat didnt "beat" -> Emergency = true
		if( command != 0 && heartbeat == heartbeat_prev ) return true;			
	}
  heartbeat_prev = heartbeat;
	return false;
}
	
