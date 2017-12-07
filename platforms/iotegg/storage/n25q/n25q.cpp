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

#include "n25q.h"
#include "mbed_debug.h"

#if defined(TARGET_ARCH_PRO)
SPI * N25Q::m_SPI = new SPI(P0_9, P0_8, P0_7);
DigitalOut * N25Q::m_SSEL = new DigitalOut(P0_6);
#endif

N25Q::N25Q() {
    SPIinit();
}

N25Q::~N25Q() {
}

void
N25Q::SPIinit(void) {
	SSOFF(); //default value
	m_SPI->format(SPI_BITS,SPI_MODE_CPHA1_CPOL1);
	m_SPI->frequency(SPI_FREQUENCY); // 25MHz
	debug_if(N25Q_VERBOSE,"MEM> SPI bits:%d, clock:%d MHz, mode:%d\r\n", (int)SPI_BITS,(int)SPI_MODE_CPHA1_CPOL1,((int)SPI_FREQUENCY / 1000000));
}

void
N25Q::SSON(void) {
	//turn on
	m_SSEL->write(0);
}

void
N25Q::SSOFF(void) {
	//turn off
	m_SSEL->write(1);
}

int
N25Q::SPIread(uint8_t *data, int length) {
	debug_if(N25Q_VERBOSE,"MEM> RX:");
	for (int i=0; i<length; i++){
		data[i] = (uint8_t) m_SPI->write(0x00);
		debug_if(N25Q_VERBOSE,"%02x", data[i]);
	}
	debug_if(N25Q_VERBOSE,"\r\n");
	return length;
}

int
N25Q::ReadID(void *data) {
	int retval=0;
	debug_if(N25Q_VERBOSE,"MEM> READ ID\r\n");
	SSON();
	m_SPI->write(READ_ID_CMD);
	retval = SPIread((uint8_t*)data,READ_ID_LEN);
	SSOFF();
	return retval;
}

int
N25Q::ReadFrom(void *data, int startingAddress, int length) {
	int retval=0;
	debug_if(N25Q_VERBOSE,"MEM> READ CMD\r\n");
	if ((startingAddress >=0) && (startingAddress <= FLASH_ADDRESS_LIMIT)){
		SSON();
		m_SPI->write(READ_CMD);
		debug_if(N25Q_VERBOSE,"MEM> ADDRESS: ");
		for (int i=2; i>=0; i--){
			int addressByte = (startingAddress >> 8*i) & 0xFF;
			debug_if(N25Q_VERBOSE,"%02x", addressByte);
			m_SPI->write(addressByte);
		}
		debug_if(N25Q_VERBOSE,"\r\n");
		retval = SPIread((uint8_t*) data,length);
		SSOFF();
	}
	else{
		debug_if(N25Q_VERBOSE,"MEM> ERROR INVALID ADDRESS\r\n");
	}
	return retval;
}

void
N25Q::ClearFlagStatusRegister(void) {
	debug_if(N25Q_VERBOSE,"MEM> CFSR\r\n");
	SSON();
	m_SPI->write(CLEAR_FLAG_STATUS_REG_CMD);
	SSOFF();
}

void
N25Q::ReadFlagStatusRegister(void *flag) {
	SSON();
	m_SPI->write(READ_FLAG_STATUS_REG_CMD);
	SPIread((uint8_t*) flag,1);
	debug_if(N25Q_VERBOSE,"MEM> RFSR %02X\r\n", *((uint8_t*) flag) );
	SSOFF();
}

void
N25Q::ReadStatusRegister(void *status) {
	SSON();
	m_SPI->write(READ_STATUS_REG_CMD);
	SPIread((uint8_t*) status,1);
	debug_if(N25Q_VERBOSE,"MEM> RSR %02X\r\n", *((uint8_t*) status) );
	SSOFF();
}

void
N25Q::WriteEnable(void) {
	uint8_t wesr;
	SSON();
	m_SPI->write(WRITE_ENABLE_CMD);
	SSOFF();
	do{
		ReadStatusRegister(&wesr);
	}while(~wesr & (1 << 1)); //until write enabled (set)
	debug_if(N25Q_VERBOSE,"MEM> WRITE ENABLED\r\n");
}

void
N25Q::WriteDisable(void) {
	uint8_t wdsr;
	SSON();
	m_SPI->write(WRITE_DISABLE_CMD);
	SSOFF();
	do{
		ReadStatusRegister(&wdsr);
	}while(wdsr & (1 << 1)); // until write disabled (cleared)
	debug_if(N25Q_VERBOSE,"MEM> WRITE ENABLED\r\n");
}

bool
N25Q::IsBusy(void) {
	uint8_t ibsr;
	ReadStatusRegister(&ibsr);
	if (ibsr & (1)){ // write in progress?
		return true;
	}
	return false;
}

bool
N25Q::WaitOperation(float timeout){
	Timer t;
	t.start();
	while (IsBusy()){
		if (t.read() >= timeout){
			debug_if(N25Q_VERBOSE,"MEM> WAIT %f SECONDS TIMEOUT!\r\n", timeout);
			return false;
		}
	}
	return true;
}

bool
N25Q::WriteStatusRegister(int status_mask) {
	WriteEnable();
	SSON();
	m_SPI->write(WRITE_STATUS_REG_CMD);
	m_SPI->write(status_mask);  // only bits 7..2 (1 and 0 are not writable)
	SSOFF();
	debug_if(N25Q_VERBOSE,"MEM> WRITE STATUS REGISTER %02X\r\n",status_mask);
	return WaitOperation(1);
}

void
N25Q::ReadLockRegister(int startingAddress, void *lock) {
	debug_if(N25Q_VERBOSE,"MEM> READ LOCK REGISTER\r\n");
	if ((startingAddress >=0) && (startingAddress <= FLASH_ADDRESS_LIMIT)){
		SSON();
		m_SPI->write(READ_LOCK_REG_CMD);
		debug_if(N25Q_VERBOSE,"MEM> ADDRESS: ");
		for (int i=2; i>=0; i--){
			int addressByte = (startingAddress >> 8*i) & 0xFF;
			debug_if(N25Q_VERBOSE,"%02x", addressByte);
			m_SPI->write(addressByte);
		}
		debug_if(N25Q_VERBOSE,"\r\n");
		SPIread((uint8_t*) lock,1);
		debug_if(N25Q_VERBOSE,"MEM> LR %02X\r\n", *((uint8_t*) lock) );
		SSOFF();
	}
	else{
		debug_if(N25Q_VERBOSE,"MEM> ERROR INVALID ADDRESS\r\n");
	}
}

void
N25Q::WriteLockRegister(int startingAddress, int lock_mask) {
	debug_if(N25Q_VERBOSE,"MEM> WRITE LOCK REGISTER\r\n");
	if ((startingAddress >=0) && (startingAddress <= FLASH_ADDRESS_LIMIT)){
		WriteEnable();
		SSON();
		m_SPI->write(WRITE_LOCK_REG_CMD);
		debug_if(N25Q_VERBOSE,"MEM> ADDRESS: ");
		for (int i=2; i>=0; i--){
			int addressByte = (startingAddress >> 8*i) & 0xFF;
			debug_if(N25Q_VERBOSE,"%02x", addressByte);
			m_SPI->write(addressByte);
		}
		debug_if(N25Q_VERBOSE,"\r\n");
		m_SPI->write(lock_mask);
		SSOFF();
		debug_if(N25Q_VERBOSE,"MEM> WRITE LOCK REGISTER %02X\r\n",lock_mask);
	}
	else{
		debug_if(N25Q_VERBOSE,"MEM> ERROR INVALID ADDRESS\r\n");
	}
}

int
N25Q::ProgramPageFrom(const void *data, int startingAddress, int length, float timeout){
	int retval=0;
	bool doneOp;
	uint8_t fsr;
	debug_if(N25Q_VERBOSE,"MEM> PROGRAM PAGE CMD\r\n");
	if ((startingAddress >=0) && (startingAddress <= FLASH_ADDRESS_LIMIT)){
		if (IsBusy()){
			debug_if(N25Q_VERBOSE,"MEM> PREVIOUS OPERATION NOT TERMINATED\r\n");
			return 0;
		}
		WriteEnable();
		SSON();
		m_SPI->write(PAGE_PROGRAM);
		debug_if(N25Q_VERBOSE,"MEM> ADDRESS: ");
		for (int i=2; i>=0; i--){
			int addressByte = (startingAddress >> 8*i) & 0xFF;
			debug_if(N25Q_VERBOSE,"%02x", addressByte);
			m_SPI->write(addressByte);
		}
		debug_if(N25Q_VERBOSE,"\r\n");
		debug_if(N25Q_VERBOSE,"MEM> DATA: ");
		for (int z=0; z<length; z++){
			debug_if(N25Q_VERBOSE,"%02x", ((uint8_t*) data)[z]);
			m_SPI->write((int) ((uint8_t*)data)[z]);
		}
		debug_if(N25Q_VERBOSE,"\r\n");
		SSOFF();
		doneOp = WaitOperation(timeout); // false if timeout
		ReadFlagStatusRegister(&fsr);
		ClearFlagStatusRegister();
		if ((fsr & (1<<1)) && (fsr & (1<<4))){
			doneOp = false; // protection/program error
		}
		if (doneOp){
			return length;
		}
	}
	else{
		debug_if(N25Q_VERBOSE,"MEM> ERROR INVALID ADDRESS\r\n");
	}
	return 0;
}

bool
N25Q::SectorErase(int startingAddress, float timeout) {
	bool doneOp;
	uint8_t fsr;
	debug_if(N25Q_VERBOSE,"MEM> SECTOR ERASE CMD\r\n");
	if ((startingAddress >=0) && (startingAddress <= FLASH_ADDRESS_LIMIT)){
		if (IsBusy()){
			debug_if(N25Q_VERBOSE,"MEM> PREVIOUS OPERATION NOT TERMINATED\r\n");
			return false;
		}
		WriteEnable();
		SSON();
		m_SPI->write(SECTOR_ERASE_CMD);
		debug_if(N25Q_VERBOSE,"MEM> ADDRESS: ");
		for (int i=2; i>=0; i--){
			int addressByte = (startingAddress >> 8*i) & 0xFF;
			debug_if(N25Q_VERBOSE,"%02x", addressByte);
			m_SPI->write(addressByte);
		}
		debug_if(N25Q_VERBOSE,"\r\n");
		SSOFF();
		doneOp = WaitOperation(timeout); // false if timeout
		ReadFlagStatusRegister(&fsr);
		ClearFlagStatusRegister();
		if ((fsr & (1<<1)) && (fsr & (1<<5))){
			doneOp = false; // protection/erase error
		}
		return doneOp;
	}
	else{
		debug_if(N25Q_VERBOSE,"MEM> ERROR INVALID ADDRESS\r\n");
		return false;
	}
}

bool
N25Q::SubSectorErase(int startingAddress, float timeout) {
	bool doneOp;
	uint8_t fsr;
	debug_if(N25Q_VERBOSE,"MEM> SECTOR ERASE CMD\r\n");
	if ((startingAddress >=0) && (startingAddress <= FLASH_ADDRESS_LIMIT)){
		if (IsBusy()){
			debug_if(N25Q_VERBOSE,"MEM> PREVIOUS OPERATION NOT TERMINATED\r\n");
			return false;
		}
		WriteEnable();
		SSON();
		m_SPI->write(SUBSECTOR_ERASE_CMD);
		debug_if(N25Q_VERBOSE,"MEM> ADDRESS: ");
		for (int i=2; i>=0; i--){
			int addressByte = (startingAddress >> 8*i) & 0xFF;
			debug_if(N25Q_VERBOSE,"%02x", addressByte);
			m_SPI->write(addressByte);
		}
		debug_if(N25Q_VERBOSE,"\r\n");
		SSOFF();
		doneOp = WaitOperation(timeout); // false if timeout
		ReadFlagStatusRegister(&fsr);
		ClearFlagStatusRegister();
		if ((fsr & (1<<1)) && (fsr & (1<<5))){
			doneOp = false; // protection/erase error
		}
		return doneOp;
	}
	else{
		debug_if(N25Q_VERBOSE,"MEM> ERROR INVALID ADDRESS\r\n");
		return false;
	}
}

bool
N25Q::BulkErase(float timeout) {
	bool doneOp;
	uint8_t fsr;
	debug_if(N25Q_VERBOSE,"MEM> SECTOR ERASE CMD\r\n");
	if (IsBusy()){
		debug_if(N25Q_VERBOSE,"MEM> PREVIOUS OPERATION NOT TERMINATED\r\n");
		return false;
	}
	WriteEnable();
	SSON();
	m_SPI->write(BULK_ERASE_CMD);
	SSOFF();
	doneOp = WaitOperation(timeout); // false if timeout
	ReadFlagStatusRegister(&fsr);
	ClearFlagStatusRegister();
	if ((fsr & (1<<1)) && (fsr & (1<<5))){
		doneOp = false; // protection/erase error
	}
	return doneOp;
}
