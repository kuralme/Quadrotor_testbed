/**
 *******************************************************************************
 * File Name          : BNO055.c
 * Description        : BNO055 sensor API
 *
 *******************************************************************************
 *
 * MIT License
 * 
 * Copyright (C) 04/12/2017  Mustafa Ege Kural
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *******************************************************************************
 */
 
#include "stm32f7xx_hal.h"
#include "stdarg.h"
#include "usbd_cdc_if.h"
#include "i2c.h"
#include "usart.h"
#include "BNO055.h"
#include "eeprom.h"


/*********************************************************************************************/
/*!
    @Warning   Choose communication method
        
		Uncommend/commend one of the macros given below:
				
    "#define VCP" to use USB_VCP communication
    or
    "#define UARTx" to use USART communication  (x for uart instance pre-initialized)				
*/
/*********************************************************************************************/

//#define VCP
#define UART3

#ifdef UART1
UART_HandleTypeDef *huart = &huart1;
#elif defined UART2
UART_HandleTypeDef *huart = &huart2;
#elif defined UART3
UART_HandleTypeDef *huart = &huart3;
#endif

bool success, newCalib;
uint8_t bnoID[1], tx_buff[128];

BNO055_t bno;

/* Optional struct definitions:
Euler_t euler;
Quaternion_t quat;
*/

/*****************************************************************************/
/**
    @brief  Sets the device address into a struct variable
		        for further use
*/
/*****************************************************************************/
void BNO055( uint8_t addr )
{
	bno.sensorID = 0;
	bno.mode = 0;
	bno.address = addr;
}


/**************************************************************************/
/**
    @brief  BNO055 start up
*/
/**************************************************************************/
void BNO055_Start()
{
  // Assign device i2c address
	BNO055( BNO055_ADDRESS);
	
	// Start the BNO
	success = BNO055_Initialize();
	
	// If initialization successful
	if(success){
	  // Check calibration and calibrate the sensor if neccesary
	   newCalib = BNO055_Calibrate();
	  
    // Store the data into EEPROM if freshly calibrated	
    if(newCalib)
	     BNO055_storeCalibration();
	
	   /*  Optionals
	   BNO055_getSensorOffsets( calibData);
	   BNO055_getTemp( temp);
	   BNO055_setExtCrystalUse( ex);
     BNO055_getSystemStatus(*system_status,*self_test_result, *system_error);
	   */
		
	   // Switch to NDOF fusion mode to prepare sensor readings
	   BNO055_PrepareToRead();
    }
	else
	Transmit((char *)tx_buff,"Check your connections...\r\n");
	
}



/*******************************************************************************/
/**
    @brief  Sets up the device with default scales and resolutions given below:

   Gyro FSO          +- 2000 °/s  ,  Resolution = 16       LSB/(°/s)
   Accelerator FSO   +- 4g        ,  Resolution = 14       LSB/(°/s)
   Magnetometer                   ,  Resolution = 13,13,15 LSB/(°/s)
*/
/*******************************************************************************/
bool BNO055_Initialize()
{
	Transmit((char *)tx_buff, "\r\nInitializing BNO..\r\n" );
	
	// Read chip id (should give 0xA0)
  //BNO055_CHIP_ID_ADDR  0x00
  HAL_I2C_Mem_Read(&hi2c1, bno.address, BNO055_CHIP_ID_ADDR, 1, &bno.sensorID , 1, 100);
  HAL_Delay(200);
  if( bno.sensorID!= BNO055_ID){
		Transmit((char *)tx_buff, "BNO connection failed \r\n" );
	  return false;
	}
	Transmit((char *)tx_buff, "BNO connection successful\r\n" );
	
  // Device reset
  //BNO055_SYS_TRIGGER_ADDR   0x3F
  tx_buff[0] = 0x20; // RST_SYS  0010 0000
  HAL_I2C_Mem_Write(&hi2c1, bno.address, BNO055_SYS_TRIGGER_ADDR, 1, tx_buff, 2, 100);
  HAL_Delay(700);
	
	// Switch mode to config mode just in case
	BNO055_setMode( OPERATION_MODE_CONFIG );
	
  // Device normal power mode (optional)
  //BNO055_PWR_MODE_ADDR  0x3E
  tx_buff[0] = POWER_MODE_NORMAL; // 0x00
  HAL_I2C_Mem_Write(&hi2c1, bno.address, BNO055_PWR_MODE_ADDR, 1, tx_buff, 2, 100);
  HAL_Delay(50);
  
  // Page selection (optional)
  //BNO055_PAGE_ID_ADDR  0x07
  tx_buff[0] = 0; //Page 0
  HAL_I2C_Mem_Write(&hi2c1, bno.address, BNO055_PAGE_ID_ADDR, 1, tx_buff, 2, 100);
  HAL_Delay(50);
		
  // Clock configuration
  //BNO055_SYS_TRIGGER_ADDR  0x3F
  tx_buff[0] = 0;  //Use external oscillator
  HAL_I2C_Mem_Write(&hi2c1, bno.address, BNO055_SYS_TRIGGER_ADDR, 1, tx_buff, 2, 100);
  HAL_Delay(50);

  /* <<<<<<<<<<<<<<<<<<<<<< Extra Configurations >>>>>>>>>>>>>>>>>>>>>>>>>>*/
  /* Set the output units */
  /*
  uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
		
  HAL_I2C_Mem_Write(&hi2c1, bno.address, BNO055_UNIT_SEL_ADDR, 1, &unitsel, 2, 100);
  HAL_Delay(200);	
  */

  /* Configure axis mapping (see datasheet section 3.4) */
  /*
	tx_buff[0] = REMAP_CONFIG_P2;  // P0-P7, Default is P1
  HAL_I2C_Mem_Write(&hi2c1, bno.address, BNO055_AXIS_MAP_CONFIG_ADDR, 1, tx_buff, 2, 100);
  HAL_Delay(200);
  HAL_I2C_Mem_Write(&hi2c1, bno.address, BNO055_AXIS_MAP_SIGN_ADDR, 1, tx_buff, 2, 100);
  HAL_Delay(200);
  */
  return true;
}

/**************************************************************************/
/**
    @brief  Puts the chip in the specified operating mode
*/
/**************************************************************************/
void BNO055_setMode(uint8_t mod)
{
  bno.mode = mod;
	HAL_I2C_Mem_Write(&hi2c1, bno.address, BNO055_OPR_MODE_ADDR, 1, &bno.mode, 2, 100);
  HAL_Delay(100);
}

/**************************************************************************/
/**
    @brief  Use the external 32.768KHz crystal
*/
/**************************************************************************/
void BNO055_setExtCrystalUse(bool usextal)
{
  uint8_t modeback = bno.mode;

  /* Switch to config mode (just in case since this is the default) */
  BNO055_setMode(OPERATION_MODE_CONFIG);
  HAL_Delay(25);
	
	tx_buff[0] = 0; //Page 0
  HAL_I2C_Mem_Write(&hi2c1, bno.address, BNO055_PAGE_ID_ADDR, 1, tx_buff, 2, 100);
	HAL_Delay(50);
	
  if (usextal) {
		tx_buff[0] = 0x80;
    HAL_I2C_Mem_Write(&hi2c1, bno.address, BNO055_SYS_TRIGGER_ADDR, 1, tx_buff, 2, 100);
		HAL_Delay(50);
  } 
	else {
	  tx_buff[0] = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, bno.address, BNO055_SYS_TRIGGER_ADDR, 1, tx_buff, 2, 100);
  }
	
  HAL_Delay(10);
  /* Set the requested operating mode (see section 3.3) */
  BNO055_setMode(modeback);
  HAL_Delay(20);
}


/**************************************************************************/
/**
    @brief  Gets the latest system status info
*/
/**************************************************************************/
void BNO055_getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error)
{
  tx_buff[0] = 0; //Page 0
  HAL_I2C_Mem_Write(&hi2c1, bno.address, BNO055_PAGE_ID_ADDR, 1, tx_buff, 2, 100);
	HAL_Delay(50);

  /* System Status (see section 4.3.58)
     ---------------------------------
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms */

  if (system_status != 0)
    HAL_I2C_Mem_Read(&hi2c1, bno.address, BNO055_SYS_STAT_ADDR, 1, system_status, 1, 100);

  /* Self Test Results (see section )
     --------------------------------
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good! */

  if (self_test_result != 0)
    HAL_I2C_Mem_Read(&hi2c1, bno.address, BNO055_SELFTEST_RESULT_ADDR, 1, self_test_result, 1, 100);

  /* System Error (see section 4.3.59)
     ---------------------------------
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error */

  if (system_error != 0)
    HAL_I2C_Mem_Read(&hi2c1, bno.address, BNO055_SYS_ERR_ADDR, 1, system_error, 1, 100);
	
  Transmit((char *)tx_buff, "System info:\r\n -System Status: %d\r\n -Self Test Results: %d\r\n -System Error: %d\r\n", *system_status, *self_test_result, *system_error);
}


/**********************************************************************************/
/**
    @brief  Checks if calibration data exists in pre-defined sector of EEPROM.
            If data found, the data will be reloaded to BNO055 offset registers.
            Else, new calibration will be required
*/
/**********************************************************************************/
bool BNO055_getCalibration()
{	
	uint16_t eeData[11];
	uint8_t cData[22];
	uint16_t eeAddress = 0;
	bnoID[0] = 0;
	
  // bnoID, EEPROM is read by 32bit ( 0x000000XX )
  EE_Read(eeAddress, (uint32_t*)bnoID);
	
	 //just in case
	if(bno.address != BNO055_ADDRESS || bno.sensorID != BNO055_ID){
	bno.address  = BNO055_ADDRESS;
	bno.sensorID = BNO055_ID;
  }
	
	if ( *bnoID != bno.sensorID){
	 Transmit((char *)tx_buff,"No calibration data exists in EEPROM\r\n");		
	 HAL_Delay(1000);
	 return false;
  }
	
	Transmit((char *)tx_buff, "Calibration data found in EEPROM!\r\n");	
  HAL_Delay(500);
		
	// EEPROM is read by 32 bit ( 3 x ( 4x 8bit value))
	EE_Reads(eeAddress+1, 6, (uint32_t*)cData);
		
  for(int i=0;i<11;i++)
		eeData[i] = (int16_t)(((uint16_t)cData[2*i+1]<<8) | cData[2*i]);
	
	HAL_Delay(20);
	
  Transmit((char *)tx_buff, "-----------------------------------------------\r\nCalibration data:\r\n\r\nAccel_radius: %d\tMag_radius: %d\r\n", eeData[9], eeData[10]);
	HAL_Delay(200);
	
	Transmit((char *)tx_buff, "Ax: %d\tMx: %d\tGx: %d\r\nAy: %d\tMy: %d\tGy: %d\r\nAz: %d\tMz: %d\tGz: %d\r\n"
		, eeData[0], eeData[1], eeData[2], eeData[3], eeData[4], eeData[5], eeData[6], eeData[7], eeData[8]);
	HAL_Delay(200);
	
	Transmit((char *)tx_buff, "-----------------------------------------------\r\n");
	HAL_Delay(1000);
	
	Transmit((char *)tx_buff, "Restoring calibration data to BNO055...\r\n");
  
	BNO055_setSensorOffsets( cData);
	return true;
}




/**************************************************************************/
/**
    @brief  Calibrate the BNO055
*/
/**************************************************************************/
bool BNO055_Calibrate()
{	
	uint8_t gyro, accel, mag, calStats, result;
	result = BNO055_getCalibration();	
	BNO055_setMode( OPERATION_MODE_NDOF );
	
	/* <<<<<<<<<<<<<<<<<<<<<<<<<  If: calibration found in EEPROM  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
  if ( result){
		Transmit((char *)tx_buff, "Checking calibration data...\r\n");
    HAL_Delay(500);
		
		HAL_I2C_Mem_Read(&hi2c1, bno.address, BNO055_CALIB_STAT_ADDR, 1, &calStats, 1, 100);
	  HAL_Delay(50);
		
		if( (calStats & 0x03) < 3 ){
			
		 Transmit((char *)tx_buff, "Magnetometer not ready. Move the device randomly to calibrate...\r\n");
		 HAL_Delay(1000);
			
			while ( BNO055_isFullyCalibrated()==false )
	      {
			  HAL_I2C_Mem_Read(&hi2c1, bno.address, BNO055_CALIB_STAT_ADDR, 1, &calStats, 1, 1000);
        mag = calStats & 0x03;
					
        Transmit((char *)tx_buff, "Mag %d\r\n", mag);			  
			  HAL_Delay(400);
			  }
		 }
		 Transmit((char *)tx_buff, "Done!\r\n");
     HAL_Delay(500);
     Transmit((char *)tx_buff, "-----------------------------------------------\r\n");
     HAL_Delay(500);
		 		
		 return false;
	 }
	 
	/* <<<<<<<<<<<<<<<<<<<<<<<<<<<<  Else: Calibrate  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/ 
	 
	 Transmit((char *)tx_buff, "Please calibrate sensor...\r\n");
	 HAL_Delay(2000);	 
	 Transmit((char *)tx_buff, "---------------------------------------------------------------------------------\r\n");
	 HAL_Delay(200);	 
   Transmit((char *)tx_buff, " -Place the device 6 different stable position for few secs to calibrate accelerometer\r\n");
	 HAL_Delay(200);	 
	 Transmit((char *)tx_buff, " -Place the device a single stable position to calibrate gyroscope\r\n");
	 HAL_Delay(200);	 
	 Transmit((char *)tx_buff, " -Move the device randomly to calibrate magnetometer\r\n--------------------------------------------------------");
	 HAL_Delay(200);
	 Transmit((char *)tx_buff, "-----------------------\r\n>> 3 = fully calibrated\r\n>> 0 = not calibrated\r\n\r\n");
	 HAL_Delay(200);
	 
	 // These variables are used as flags not to print calibrated ones
	 uint8_t gcheck = 1, acheck = 1, mcheck = 1;
	 uint8_t accel_prev, mag_prev;
		 
		while ( BNO055_isFullyCalibrated() == false )
	  {
		 HAL_I2C_Mem_Read(&hi2c1, bno.address, BNO055_CALIB_STAT_ADDR, 1, &calStats, 1, 100);
		 HAL_Delay(200);
				 
      gyro = (calStats >> 4) & 0x03;
       if( gyro != 0 && gcheck )
				{ if( gyro == 3){gcheck=0;
					  Transmit((char *)tx_buff, "Gyro done!\r\n");}
				  else
					  Transmit((char *)tx_buff, "Gyro %d \r\n", gyro);
				 
         HAL_Delay(100);
				}				 
       accel = (calStats >> 2) & 0x03;
				if( accel != 0 && accel != accel_prev && acheck )
				{ if( accel == 3){acheck=0;
						Transmit((char *)tx_buff, "Accel done!\r\n");}
				  else
					  Transmit((char *)tx_buff, "Accel %d \r\n", accel);
				 
				 HAL_Delay(100);
				 accel_prev = accel;
				}
       mag = calStats & 0x03;
				if( mag != 0 && mag != mag_prev && mcheck)
			  { if( mag == 3){mcheck=0;
 					  Transmit((char *)tx_buff, "Magno done!\r\n");}
				  else 
						Transmit((char *)tx_buff, "Mag %d \r\n", mag);
					
				HAL_Delay(100);
				mag_prev = mag;
			 }
				 
		  HAL_Delay(200);
		}
	Transmit((char *)tx_buff, "Fully calibrated!\r\n");		
	return true;
}





/**************************************************************************/
/**
    @brief  Reads the sensor's offset registers into a byte array
*/
/**************************************************************************/
bool BNO055_getSensorOffsets( uint8_t* data)
{
    if ( BNO055_isFullyCalibrated())
    {
      BNO055_setMode(OPERATION_MODE_CONFIG);
			
      // Read new calibration data
		  HAL_I2C_Mem_Read(&hi2c1, bno.address, ACCEL_OFFSET_X_LSB_ADDR, 1, data, 22, 1000);	  		
		  HAL_Delay(200);
      return true;
    }
  return false;
}


/**********************************************************************************/
/**
    @brief  Reads calibration data from BNO055 and stores it into the defined 
            pre-defined sector of EEPROM.
*/
/**********************************************************************************/
bool BNO055_storeCalibration()
{
	uint16_t newData[11], eeAddress = 0;
	uint8_t cData[22];
	bool check;
	
	BNO055_getSensorOffsets( cData);
	
	Transmit((char *)tx_buff, "Storing new calibration data to EEPROM...\r\n");
	
  // Store first byte as chip id of BNO 055
	bnoID[0] = bno.sensorID;
	check = EE_Write(eeAddress, (uint32_t)bnoID[0]);
	HAL_Delay(100);
	
	// EEPROM is written by 32 bits ( 6 x ( 4x 8bit value))
	check = EE_Writes(eeAddress+1, 6, (uint32_t*)cData);
	HAL_Delay(100);
	
	if(check == false){
	  Transmit((char *)tx_buff, "The data store into EEPROM failed!\r\n");		
	  HAL_Delay(1000);
		return false;
	}
			
	for(int i=0;i<11;i++) 
	  newData[i] = (int16_t)(((uint16_t)cData[2*i+1]<<8) | cData[2*i]);
	
  Transmit((char *)tx_buff, "The data is stored into EEPROM!\r\n");
	HAL_Delay(500);
	Transmit((char *)tx_buff, "-----------------------------------------------\r\nOffset calibration results:\r\n\r\nAccel_radius: %d\tMag_radius: %d\r\n", newData[9], newData[10]);
	HAL_Delay(200);
	Transmit((char *)tx_buff, "Ax: %d\tMx: %d\tGx: %d\r\nAy: %d\tMy: %d\tGy: %d\r\nAz: %d\tMz: %d\tGz: %d\r\n"
			, newData[0], newData[1], newData[2], newData[3], newData[4], newData[5], newData[6], newData[7], newData[8]);
	HAL_Delay(200);
	Transmit((char *)tx_buff, "-----------------------------------------------\r\n");
	
	return true;
}



/********************************************************************************/
/**
    @brief  Writes an array of calibration values into the sensor's offset registers
*/
/********************************************************************************/
void BNO055_setSensorOffsets( uint8_t* data)
{
	if(bno.mode != OPERATION_MODE_CONFIG)
	 BNO055_setMode( OPERATION_MODE_CONFIG );
	
	HAL_I2C_Mem_Write(&hi2c1, bno.address, ACCEL_OFFSET_X_LSB_ADDR, 1, data, 23, 1000);
	HAL_Delay(200);
}


/******************************************************************************/
/**
    @brief  Check if the calibration fully completed
*/
/******************************************************************************/
bool BNO055_isFullyCalibrated(void)
{
	uint8_t calStats;
	HAL_I2C_Mem_Read(&hi2c1, bno.address, BNO055_CALIB_STAT_ADDR, 1, &calStats, 1, 100);
	HAL_Delay(200);
	
  if (calStats != 255)
    return false;  // not fully calibrated yet
  else
    return true;   // fully calibrated
}


/************************************************************************************/
/**
    @brief  Reading preparation 

    @note   The reading mode is set to ndof by default. Please refer to 
            datasheet section 3.3 for other modes
*/
/************************************************************************************/
void  BNO055_PrepareToRead()
{
	if(bno.mode != OPERATION_MODE_NDOF)
	  BNO055_setMode(OPERATION_MODE_NDOF);	
	HAL_Delay(1000);
}

/**************************************************************************/
/**
    @brief  Gets the temperature in degrees celsius
*/
/**************************************************************************/
void BNO055_getTemp( float *t)
{
	uint8_t temp;
	HAL_I2C_Mem_Read(&hi2c1, bno.address, BNO055_TEMP_ADDR, 1, &temp, 1, 100);
	*t = (float)temp;
}


/**************************************************************************/
/**
    @brief  Gets Euler angle readings from the specified source

    @Note   Additionally, the data can be stored the values in a struct
*/
/**************************************************************************/
void BNO055_readEuler( float *x, float *y , float *z)
{
	uint8_t Buffer[6];
	int16_t Buffer16[3];
	
	//Read euler angles
	//BNO055_EULER_H_LSB_ADDR  0x1A
	HAL_I2C_Mem_Read(&hi2c1, bno.address, BNO055_EULER_H_LSB_ADDR, 1, Buffer, 6, 1000);
	for(int i=0;i<3;i++)
     Buffer16[i] = (int16_t)(((uint16_t)Buffer[2*i+1]<<8) | Buffer[2*i]);
	
  // Store the angles into the given variables
	/* 1 degree = 16 LSB */
  *z = (float)Buffer16[0]/16.0F;
  *x = (float)Buffer16[1]/16.0F;
  *y = (float)Buffer16[2]/16.0F;
	
	// Store the angles into a struct additionally
  //  euler.euler_roll  = *x;
  //	euler.euler_pitch = *y;
  //	euler.euler_yaw   = *z;
}


/**************************************************************************/
/**
    @brief  Gets Quaternion reading from the specified source

    @Note   Additionally, the data can be stored the values in a struct
*/
/**************************************************************************/
void BNO055_readQuaternion( float *qw, float *qx, float *qy, float *qz)
{
	uint8_t Buffer[8];
	uint16_t Buffer16[4];
  const double scale = (1.0 / (1<<14));
	
	//Read quaternions
	//BNO055_QUATERNION_DATA_W_LSB_ADDR  0x20
	HAL_I2C_Mem_Read(&hi2c1, bno.address, BNO055_QUATERNION_DATA_W_LSB_ADDR, I2C_MEMADD_SIZE_8BIT, Buffer, 8, 1000);
	for(int i=0;i<3;i++)
    Buffer16[i] = (int16_t)(((uint16_t)Buffer[2*i+1]<<8) | Buffer[2*i]);
	
	// Store the quaternions into the given variables
  *qw = (float)Buffer16[0]*scale;
  *qx = (float)Buffer16[1]*scale;
  *qy = (float)Buffer16[2]*scale;
	*qz = (float)Buffer16[3]*scale;
	
	
	// Store the quaternions into a struct additionally
  //  quat.quaternion_w = *qw;
  //	quat.quaternion_x = *qx;
  //	quat.quaternion_y = *qy;
  //	quat.quaternion_z = *qz;
}




/**************************************************************************/
/**
    @brief  Terminal printing method using HAL Libraries
*/
/**************************************************************************/
void Transmit( char *s, const char *format, ...)
{
	va_list arg;
	va_start (arg, format);
	vsprintf (s, format, arg);
	va_end (arg);
	
  #if defined UART1 || defined UART2 || defined UART3
	  HAL_UART_Transmit( huart, (uint8_t*)s, strlen(s), 100);
  #else
    CDC_Transmit_FS((uint8_t*)s, strlen(s));
  #endif
}
