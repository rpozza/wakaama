/*
 * Copyright [2016] [Riccardo Pozza]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author:
 * Riccardo Pozza <r.pozza@surrey.ac.uk>
 */

#ifndef N25Q_DRIVER
#define N25Q_DRIVER

#include "mbed.h"
#include <string>

/*
 * MEMORY TABLE (64MBIT memory):
 * 128 Sectors of 64KB each 	=> 8192KB (8MB)   0x00000000 -> 0x007FFFFF
 * 16 Sub-Sectors of 4KB each 	=> 64KB		  	  0x00000000 -> 0x00000FFF
 * 16 Pages of 256B each 		=> 4096B 		  0x00000000 -> 0x000000FF
 *
 * Protected Area Sizes:
 * (see datasheet)
 */

#define N25Q_VERBOSE				0

// SPI constants
#define SPI_BITS					8
#define SPI_MODE_CPHA1_CPOL1		3
#define SPI_FREQUENCY				25000000
#define SPI_BUFFER_SIZE				512

#define READ_ID_LEN 				20
#define FLASH_ADDRESS_LIMIT			0x007FFFFF
#define PAGE_SIZE 					256

#define WRITE_STATUS_REG_CMD		0x01
#define PAGE_PROGRAM				0x02
#define READ_CMD 					0x03
#define WRITE_DISABLE_CMD 			0x04
#define READ_STATUS_REG_CMD 		0x05
#define WRITE_ENABLE_CMD 			0x06
#define SUBSECTOR_ERASE_CMD			0x20
#define CLEAR_FLAG_STATUS_REG_CMD	0x50
#define READ_FLAG_STATUS_REG_CMD	0x70
#define BULK_ERASE_CMD				0xC7
#define SECTOR_ERASE_CMD			0xD8
#define WRITE_LOCK_REG_CMD  		0xE5
#define READ_LOCK_REG_CMD   		0xE8
#define READ_ID_CMD 				0x9F

class N25Q
{

private:
	/*
	 * SSON:
	 * Drives the CS low, enabling the slave device/s.
	 */
	void SSON(void);
	/*
	 * SSOFF:
	 * Drives the CS high, disabling the slave device/s.
	 */
	void SSOFF(void);
	/*
	 * SPIinit:
	 * Prepares SPI with all the default parameters.
	 */
	void SPIinit(void);
	/*
	 * SPIread:
	 * Reads a byte array of length. Returns the length
	 * NB: requires SSON.
	 */
	int SPIread(uint8_t *data, int length);

	static SPI * m_SPI;
	static DigitalOut * m_SSEL;

public:
	N25Q();
	virtual ~N25Q();
	/*
	 * ReadID:
	 * Reads the ID as such and return its length.
	 *  a) 1x Manufacturer ID (0x20), assigned by JEDEC
	 *  b) Device ID, assigned by Manufacturer
	 * 	   1x Memory Type (0xBA)
	 * 	   1x Memory Capacity (0x17), 64 Mbit
	 *  c) Unique ID, assigned by Factory
	 *     1x Lenght of data to follow (0x10)
	 *     1x Extended Device ID
	 *     1x Device Configuration
	 *    14x Customised Factory Data
	 */
	int ReadID(void *data);
	/*
	 * ReadFrom:
	 * Reads up to length bytes from the starting address.
	 */
	int ReadFrom(void *data, int startingAddress, int length);
	/*
	 * ClearFlagStatusRegister:
	 * Resets the error bits in the Flag Status Register.
	 */
	void ClearFlagStatusRegister(void);
	/*
	 * ReadFlagStatusRegister:
	 * Reads the Flag Status Register:
	 * 7: program/erase controller 		0=busy				1=ready
	 * 6: erase suspend					0=not in effect 	1=in effect
	 * 5: erase							0=clear				1=failure/protection error
	 * 4: program						0=clear				1=failure/protection error
	 * 3: vpp							0=enabled			1=disabled (default)
	 * 2: program suspend				0=not in effect		1=in effect
	 * 1: protection					0=clear				1=failure/protection error
	 * 0: reserved						X					X
	 */
	void ReadFlagStatusRegister(void *flag);
	/*
	 * ReadStatusRegister:
	 * Reads the Status Register:
	 * 7: status register write enable  0=enabled			1=disabled
	 * 6: block protects bit 3			(see protected area sizes)
	 * 5: protected mem top/bottom		0=top			 	1=bottom
	 * 4: block protects bit 2			(see protected area sizes)
	 * 3: block protects bit 1			(see protected area sizes)
	 * 2: block protects bit 0			(see protected area sizes)
	 * 1: write enabled					0=clear(default)	1=set
	 * 0: write in progress				0=ready				1=busy
	 */
	void ReadStatusRegister(void *status);
	/*
	 * WriteEnable:
	 * Prepares for write.
	 */
	void WriteEnable(void);
	/*
	 * WriteDisable:
	 * Close down write operation.
	 */
	void WriteDisable(void);
	/*
	 * IsBusy:
	 * True if write operation in progress.
	 */
	bool IsBusy(void);
	/*
	 * WaitOperation:
	 * Wait for a write in progress until timeout.
	 */
	bool WaitOperation(float timeout);
	/*
	 * WriteStatusRegister:
	 * Write non-volatile bits (7..2) of the Status Register with mask.
	 */
	bool WriteStatusRegister(int status_mask);
	/*
	 * ReadLockRegister:
	 * Reads the Lock register at the address (see datasheet).
	 */
	void ReadLockRegister(int startingAddress, void *lock);
	/*
	 * WriteLockRegister:
	 * Write the Lock register at the address with the appropriate mask (see datasheet).
	 */
	void WriteLockRegister(int startingAddress, int lock_mask);
	/*
	 * ProgramFrom:
	 * Writes up to length bytes from the starting address.
	 * Returns 0 if protection error or busy or timeout or invalid address. Length if successful.
	 */
	int ProgramPageFrom(const void *data, int startingAddress, int length, float timeout=1);
	/*
	 * SectorErase:
	 * Erase a 64KB sector.
	 * Returns false if protection error or busy or timeout or invalid address. True if successful.
	 */
	bool SectorErase(int startingAddress, float timeout=3);
	/*
	 * SubSectorErase:
	 * Erase a 4KB sector.
	 * Returns false if protection error or busy or timeout or invalid address. True if successful.
	 */
	bool SubSectorErase(int startingAddress, float timeout=3);
	/*
	 * BulkErase:
	 * Erase all sectors.
	 * Returns false if protection error or busy or timeout. True if successful.
	 */
	bool BulkErase(float timeout=480);

protected:

};

#endif
