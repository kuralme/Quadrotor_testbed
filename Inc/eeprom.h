/**
 *******************************************************************************
 * File Name          : eeprom.h
 * Description        : EEPROM header file
 *
 *******************************************************************************
 *
 * MIT License
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

#ifndef __EEPROM_H
#define __EEPROM_H

#include <stdbool.h>
#include "stm32f7xx_hal.h"


/***********************************************************/
/*!
    @brief  Sector definition:
	          Uncomment/comment one of the sectors below
*/
/***********************************************************/
#define SECTOR_8  //(default)
//#define SECTOR_9
//#define SECTOR_10
//#define SECTOR_11

//################################################################################################################
bool	EE_Format(void);
bool 	EE_Read(uint16_t VirtualAddress, uint32_t* Data);
bool 	EE_Write(uint16_t VirtualAddress, uint32_t Data);
bool	EE_Reads(uint16_t StartVirtualAddress,uint16_t HowMuchToRead,uint32_t* Data);
bool 	EE_Writes(uint16_t StartVirtualAddress,uint16_t HowMuchToWrite,uint32_t* Data);
//################################################################################################################

#endif
