/**
*   This file is part of IcedCoffeeOS
*   (https://github.com/rromanotero/IcedCoffeeOS).
*
*   Copyright (c) 2020 Rafael Roman Otero.
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*
**/

#include "hal.h"
#include "sd.h"

/*
*  HAL Storage Init
*/
uint32_t hal_storage_init( void ){
	if( sd_init() == SD_OK )
	    return HAL_SUCCESS;
     else
        return HAL_FAILED;
}

/*
*   Read a block from the Storage
*
* params
*      buffer - buffer to read data to
*      lba - Logical Block Addressing (matches sector number, when sector size is 512)
*      block_num - num of blocks to read
*
*  returns num of block read
*/
uint32_t hal_storage_read_block( uint8_t* buffer, uint32_t lba,  uint32_t num_of_blocks ){
    return sd_readblock(lba, buffer, num_of_blocks);
}
