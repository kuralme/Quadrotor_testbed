/**
 *******************************************************************************
 * File Name          : eeprom.c
 * Description        : EEPROM operations source file
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
#include "eeprom.h"

/************************************************************************************/
/*!
    @Note   Flash sector size is adjusted to 64k instead of 256kbytes due
		        to the variable bit size limit
*/
/************************************************************************************/
#define		_EEPROM_FLASH_SECTOR_SIZE		65536

/************************************************************************************/
/*!
    @brief  Define one of the 4 sectors below in the header file to choose which 
		        sector of the flash memory for write, read or erase operations to be held. 
						
		@Note   In this API, flash memory operations only held for single bank 
		        option. Please refer to the referance manual for dual bank operation
            and further information about AXIM interface.						
*/
/************************************************************************************/

#if defined SECTOR_8
/* Base address of the Flash sector 8 */    
#define _EEPROM_FLASH_SECTOR_ADDRESS    ((uint32_t)0x08100000) /* Base @ of SECTOR 8, 256 Kbytes */
#endif

#if defined SECTOR_9
/* Base address of the Flash sector 9 */    
#define _EEPROM_FLASH_SECTOR_ADDRESS    ((uint32_t)0x08140000) /* Base @ of SECTOR 9, 256 Kbytes */
#endif

#if defined SECTOR_10
/* Base address of the Flash sector 10 */    
#define _EEPROM_FLASH_SECTOR_ADDRESS    ((uint32_t)0x08180000) /* Base @ of SECTOR 10, 256 Kbytes */
#endif

#if defined SECTOR_11
/* Base address of the Flash sector 11 */    
#define _EEPROM_FLASH_SECTOR_ADDRESS    ((uint32_t)0x081C0000) /* Base @ of SECTOR 11, 256 Kbytes */
#endif


uint32_t	EEPROMSectorBackup[_EEPROM_FLASH_SECTOR_SIZE/4];

//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
bool	EE_Format(void)
{	
	uint32_t	error;		
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef	flashErase;
	flashErase.TypeErase = FLASH_TYPEERASE_SECTORS;
	
	#if defined SECTOR_8
	flashErase.Sector = FLASH_SECTOR_8;
	#endif
	
	#if defined SECTOR_9
	flashErase.Sector = FLASH_SECTOR_9;
	#endif
	
	#if defined SECTOR_10
	flashErase.Sector = FLASH_SECTOR_10;
	#endif
	
	#if defined SECTOR_11
	flashErase.Sector = FLASH_SECTOR_11;
	#endif
	
	flashErase.NbSectors = 1;
	flashErase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	
	if(HAL_FLASHEx_Erase(&flashErase,&error)==HAL_OK)
	{
		HAL_FLASH_Lock();
		if(error != 0xFFFFFFFF)
			return false;
		else
			return true;	
	}
	HAL_FLASH_Lock();
	return false;
}
//##########################################################################################################
bool EE_Read(uint16_t VirtualAddress, uint32_t* Data)
{
	if(VirtualAddress >=	(_EEPROM_FLASH_SECTOR_SIZE/4))
		return false;
	*Data =  (*(__IO uint32_t*)((VirtualAddress*4)+_EEPROM_FLASH_SECTOR_ADDRESS));
	return true;
}
//##########################################################################################################
bool EE_Write(uint16_t VirtualAddress, uint32_t Data)
{
	if(VirtualAddress >=	(_EEPROM_FLASH_SECTOR_SIZE/4))
		return false;

	if((*(__IO uint32_t*)((VirtualAddress*4)+_EEPROM_FLASH_SECTOR_ADDRESS)) != 0xFFFFFFFF)
	{
		
		if( EE_Reads(0,(_EEPROM_FLASH_SECTOR_SIZE/4),EEPROMSectorBackup)==false)
		{
			HAL_FLASH_Lock();
			return false;
		}
		EEPROMSectorBackup[VirtualAddress]=Data;
		EE_Format();
		
		HAL_FLASH_Unlock();
		for(uint16_t	i=0 ; i<_EEPROM_FLASH_SECTOR_SIZE/4 ; i++)
		{
      if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(i*4)+_EEPROM_FLASH_SECTOR_ADDRESS,(uint64_t)EEPROMSectorBackup[i])!=HAL_OK)
			{
				HAL_FLASH_Lock();
				return false;
			}			
		}
	}	
	HAL_FLASH_Unlock();
	if(Data!=0xFFFFFFFF)
	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(VirtualAddress*4)+_EEPROM_FLASH_SECTOR_ADDRESS,(uint64_t)Data)==HAL_OK)
		{
			HAL_FLASH_Lock();
			return true;
		}
		else
		{
			HAL_FLASH_Lock();
			return false;
		}
	}
	HAL_FLASH_Lock();
	return true;

}
//##########################################################################################################
bool EE_Reads(uint16_t StartVirtualAddress,uint16_t HowMuchToRead,uint32_t* Data)
{
	if((StartVirtualAddress+HowMuchToRead) >	(_EEPROM_FLASH_SECTOR_SIZE/4))
		return false;
	for(uint16_t	i=StartVirtualAddress ; i<HowMuchToRead+StartVirtualAddress ; i++)
	{
		Data[i-StartVirtualAddress] =  (*(__IO uint32_t*)((i*4)+_EEPROM_FLASH_SECTOR_ADDRESS));
	}
	return true;
}
//##########################################################################################################
bool 	EE_Writes(uint16_t StartVirtualAddress,uint16_t HowMuchToWrite,uint32_t* Data)
{
	if((StartVirtualAddress+HowMuchToWrite) >	(_EEPROM_FLASH_SECTOR_SIZE/4))
		return false;
	if( EE_Reads(0,(_EEPROM_FLASH_SECTOR_SIZE/4),EEPROMSectorBackup)==false)
		return false;
	for(uint16_t	i=StartVirtualAddress ; i<HowMuchToWrite+StartVirtualAddress ; i++)
	{				
		EEPROMSectorBackup[i]=Data[i-StartVirtualAddress];
	}	
	if(EE_Format()==false)
		return false;
	HAL_FLASH_Unlock();
	for(uint16_t	i=0 ; i<(_EEPROM_FLASH_SECTOR_SIZE/4); i++)
	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(i*4)+_EEPROM_FLASH_SECTOR_ADDRESS,(uint64_t)EEPROMSectorBackup[i])!=HAL_OK)
		{
			HAL_FLASH_Lock();
			return false;
		}
	}
	HAL_FLASH_Lock();
	return true;
}
//##########################################################################################################
