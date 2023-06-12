#include <sensors.h>
#include "mfrc522.h"
#include "fsl_gpio.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "TCA9535.h"
#include "i2c.h"
#include "modem.h"
#include "l6470_motor_driver.h"

uid_t uid;
TaskHandle_t rfid_init_task_handle;

volatile bool rfid_init_done_flag = false;
uint8_t rfidreadvalue[16]={0};

static void rfid_init_task(void *arg);
void string2hexString(char* input, char* output);

void rfid_task_init(void)
{
	/* Create task to initialize RFID */
	if (xTaskCreate(rfid_init_task, "rfid_init_task", 2000, NULL, 3, &rfid_init_task_handle) != pdPASS)
	{
		PRINTF("[ERROR]   RFID: Failed to create RFID init task !\r\n");
	}
}

static void rfid_init_task(void *arg)
{
	picc_type_t picc_type;
	mifare_key_t key;
	uint8_t nuidPICC[8]={0};
	uint8_t counter =0;

	while(!motor_init_done_flag)
	{
		vTaskDelay(10);
	}

	TCA9535_Register outputConfig;

	TCA9535ReadConfig(LPI2C_IO2_SLAVE_ADDR,&outputConfig);
	outputConfig.Port.P1.bit.B7 = 0; //output
	TCA9535WriteConfig(LPI2C_IO2_SLAVE_ADDR,&outputConfig);

	TCA9535ReadOutput(LPI2C_IO2_SLAVE_ADDR,&outputConfig);
	outputConfig.Port.P1.bit.B7 = 0; //output
	TCA9535WriteOutput(LPI2C_IO2_SLAVE_ADDR,&outputConfig);

    taskENTER_CRITICAL();
    pcd_init();
    taskEXIT_CRITICAL();

	//PRINTF("This code scan the MIFARE Classsic NUID using following key:");

	for (uint8_t i = 0; i < 6; i++) {
		key.keyByte[i] = 0xFF;
		//PRINTF("%X" ,key.keyByte[i]);
	}

	rfid_init_done_flag = true;

	for(;;)
	{
		if(scan_RFID_flag == true)
		{
			counter = 0;
			while(signal_strength_reading_flag != false && counter <= 200)
			{
				counter++;
				vTaskDelay(10);
			}
			signal_strength_pause_flag = true;
			get_signal_value_count = 0;
			/*  Look for new cards */
			if (picc_is_new_card_present()){

				/* Verify if the NUID has been readed */
				if ( picc_read_card_serial()){
					picc_type = picc_get_type(uid.sak);

					/* Check is the PICC of Classic MIFARE type */
					if (picc_type != PICC_TYPE_MIFARE_MINI || picc_type !=  PICC_TYPE_MIFARE_1K ||picc_type != PICC_TYPE_MIFARE_4K) {
						PRINTF("Your tag is not of type MIFARE Classic.\r\n");
					}

					if (uid.uidByte[0] != nuidPICC[0] || uid.uidByte[1] != nuidPICC[1] || uid.uidByte[2] != nuidPICC[2] || uid.uidByte[3] != nuidPICC[3] )
					{
						PRINTF("A new card has been detected.\r\n");
						memset(rfidreadvalue,0x00,sizeof(rfidreadvalue));
						/* Store NUID into nuidPICC array */
						for (uint8_t i = 0; i < 7; i++) {
						  nuidPICC[i] = uid.uidByte[i];
						}
						string2hexString(nuidPICC,rfidreadvalue);
						PRINTF("%s",rfidreadvalue);
					}
					else
						PRINTF("Card read previously.\r\n");

					/* Halt PICC */
					picc_halt_A();

					/* Stop encryption on PCD */
					pcd_stop_crypto_1();
					scan_RFID_flag = false;
				}
			}
			signal_strength_pause_flag = false;
		}
		vTaskDelay(8);
	}
}

//function to convert ascii char[] to hex-string (char[])
void string2hexString(char* input, char* output)
{
    int loop;
    int i;

    i=0;
    loop=0;

    while(input[loop] != '\0')
    {
        sprintf((char*)(output+i),"%02X", input[loop]);
        loop+=1;
        i+=2;
    }
    //insert NULL at the end of the output string
    output[i++] = '\0';
}

void pcd_init(void)
{
	uint8_t value[10];
	bool hardReset = false;

	TCA9535_Register outputConfig;

	TCA9535ReadConfig(LPI2C_IO2_SLAVE_ADDR,&outputConfig);
	outputConfig.Port.P1.bit.B7 = 0; //output
	TCA9535WriteConfig(LPI2C_IO2_SLAVE_ADDR,&outputConfig);

	TCA9535ReadOutput(LPI2C_IO2_SLAVE_ADDR,&outputConfig);
	outputConfig.Port.P1.bit.B7 = 0;  /* reset pin reset */
	TCA9535WriteOutput(LPI2C_IO2_SLAVE_ADDR,&outputConfig);
	//GPIO_PinWrite(GPIO1, 3U, 0U);   /* reset pin set */

	vTaskDelay(1);                 // 8.8.1 Reset timing requirements says about 100ns

	TCA9535ReadOutput(LPI2C_IO2_SLAVE_ADDR,&outputConfig);
	outputConfig.Port.P1.bit.B7 = 1;  /* reset pin set */
	TCA9535WriteOutput(LPI2C_IO2_SLAVE_ADDR,&outputConfig);


	//GPIO_PinWrite(GPIO1, 3U, 1U);   /* reset pin set */

	vTaskDelay(50);    // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs
	hardReset = true;



	if (!hardReset) { // Perform a soft reset if we haven't triggered a hard reset above.
		pcd_reset();
	}
//	spi_slave_select(SPI_RFID);

	// Reset baud rates
	spi_write_reg(SPI_RFID,ModeReg, 0x00);

	spi_write_reg(SPI_RFID,RxModeReg, 0x00);

	// Reset ModWidthReg
	spi_write_reg(SPI_RFID,ModWidthReg, 0x26);


	spi_read_reg(SPI_RFID,ModWidthReg);

	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	spi_write_reg(SPI_RFID,TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	spi_write_reg(SPI_RFID,TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
	spi_write_reg(SPI_RFID,TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	spi_write_reg(SPI_RFID,TReloadRegL, 0xE8);

	spi_write_reg(SPI_RFID,TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	spi_write_reg(SPI_RFID,ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)

	pcd_antenna_on();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
	//PRINTF("Initialization successful.\n");
}

void pcd_antenna_on(void)
{
	 uint8_t value;
	spi_read_reg(SPI_RFID,TxControlReg);
	value = spi_rx_data[1];
	if ((value & 0x03) != 0x03) {
		spi_write_reg(SPI_RFID,TxControlReg, value | 0x03);
		}

	 PRINTF("rfid antenna turned on.\n");
}

/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 *
 * @return bool
 */
bool picc_is_new_card_present(void) {
	uint8_t bufferATQA[8];

	//static uint8_t bufferATQA[4]={0,0,0,0};
	uint8_t bufferSize = sizeof(bufferATQA);

    /* Enter quad mode. */
    taskENTER_CRITICAL();
	// Reset baud rates
	spi_write_reg(SPI_RFID,TxModeReg, 0x00);
	spi_write_reg(SPI_RFID,RxModeReg, 0x00);
	// Reset ModWidthReg
	spi_write_reg(SPI_RFID,ModWidthReg, 0x26);

	status_code_t result = picc_reqa(bufferATQA, &bufferSize);
    taskEXIT_CRITICAL();
	return (result == STATUS_OK || result == STATUS_COLLISION);
}

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 *
 * @return bool
 */
bool picc_read_card_serial(void) {
	status_code_t result = picc_select(&uid,0);
	return (result == STATUS_OK);
}

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 *
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code_t picc_select(uid_t *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
											uint8_t validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
										 ) {
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	uint8_t cascadeLevel = 1;
	status_code_t result;
	uint8_t count;
	uint8_t checkBit;
	uint8_t index;
	uint8_t uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	uint8_t bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte.
	uint8_t *responseBuffer;
	uint8_t responseLength;

	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9

	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}

	// Prepare MFRC522
	pcd_clear_register_bit_mask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.

	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;

			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;

			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;

			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}

		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}

		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while (!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//PRINTF("SELECT: currentLevelKnownBits=%d\r\n",currentLevelKnownBits);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = pcd_calculate_crc(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}

			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			spi_write_reg(SPI_RFID,BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]




			// Transmit the buffer and receive the response.
			result = pcd_transceive_data(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign,0);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				uint8_t valueOfCollReg;
				spi_read_reg(SPI_RFID,CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				valueOfCollReg = spi_rx_data[1];
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << checkBit);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}

		} // End of while (!selectDone)

		// We do not check the CBB - it was constructed by us above.

		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}

		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = pcd_calculate_crc(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)

	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
}

/**
 * Used to exit the PCD from its authenticated state.
 * Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
 */
void pcd_stop_crypto_1(void) {
	// Clear MFCrypto1On bit
	pcd_clear_register_bit_mask(Status2Reg, 0x08); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
} // End PCD_StopCrypto1()

/**
 * Translates the SAK (Select Acknowledge) to a PICC type.
 *
 * @return PICC_Type
 */
picc_type_t picc_get_type(uint8_t sak		///< The SAK byte returned from PICC_Select().
										) {
	// http://www.nxp.com/documents/application_note/AN10833.pdf
	// 3.2 Coding of Select Acknowledge (SAK)
	// ignore 8-bit (iso14443 starts with LSBit = bit 1)
	// fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
	sak &= 0x7F;
	switch (sak) {
		case 0x04:	return PICC_TYPE_NOT_COMPLETE;	// UID not complete
		case 0x09:	return PICC_TYPE_MIFARE_MINI;
		case 0x08:	return PICC_TYPE_MIFARE_1K;
		case 0x18:	return PICC_TYPE_MIFARE_4K;
		case 0x00:	return PICC_TYPE_MIFARE_UL;
		case 0x10:
		case 0x11:	return PICC_TYPE_MIFARE_PLUS;
		case 0x01:	return PICC_TYPE_TNP3XXX;
		case 0x20:	return PICC_TYPE_ISO_14443_4;
		case 0x40:	return PICC_TYPE_ISO_18092;
		default:	return PICC_TYPE_UNKNOWN;
	}
}

/**
 * Dumps debug info about the connected PCD to Serial.
 * Shows all known firmware versions
 */
void pcd_dump_version_to_serial(void)
{
	uint8_t version;
	// Get the MFRC522 firmware version
	spi_read_reg(SPI_RFID,VersionReg);
	version = spi_rx_data[1];
	PRINTF("MFRC522 firmware version %X", version);

	// Lookup which version
	switch(version) {
		case 0x88: PRINTF(" = (clone)\r\n");  break;
		case 0x90: PRINTF(" = v0.0\r\n");     break;
		case 0x91: PRINTF(" = v1.0\r\n");     break;
		case 0x92: PRINTF(" = v2.0\r\n");     break;
		default:   PRINTF(" = (unknown)\r\n");
	}
	// When 0x00 or 0xFF is returned, communication probably failed
	if ((version == 0x00) || (version == 0xFF))
		PRINTF("WARNING: Communication failure, is the MFRC522 properly connected?");
}

/**
 * Dumps debug info about the selected PICC to Serial.
 * On success the PICC is halted after dumping the data.
 * For MIFARE Classic the factory default key of 0xFFFFFFFFFFFF is tried.
 *
 * @DEPRECATED Kept for bakward compatibility
 */
void picc_dump_to_serial(uid_t *uid	///< Pointer to Uid struct returned from a successful PICC_Select().
								) {
	mifare_key_t key;

	// Dump UID, SAK and Type
	picc_dump_details_to_serial(uid);

	// Dump contents
	picc_type_t piccType = picc_get_type(uid->sak);
	switch (piccType) {
		case PICC_TYPE_MIFARE_MINI:
		case PICC_TYPE_MIFARE_1K:
		case PICC_TYPE_MIFARE_4K:
			// All keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
			for (uint8_t i = 0; i < 6; i++) {
				key.keyByte[i] = 0xFF;
			}
			picc_dump_mifare_classic_to_serial(uid, piccType, &key);
			break;

		case PICC_TYPE_MIFARE_UL:
			picc_dump_mifare_ultralight_to_serial();
			break;

		case PICC_TYPE_ISO_14443_4:
		case PICC_TYPE_MIFARE_DESFIRE:
		case PICC_TYPE_ISO_18092:
		case PICC_TYPE_MIFARE_PLUS:
		case PICC_TYPE_TNP3XXX:
			PRINTF("Dumping memory contents not implemented for that PICC type.\r\n");
			break;

		case PICC_TYPE_UNKNOWN:
		case PICC_TYPE_NOT_COMPLETE:
		default:
			break; // No memory dump here
	}
	//PICC_HaltA(); // Already done if it was a MIFARE Classic PICC.
}

/**
 * Dumps memory contents of a MIFARE Classic PICC.
 * On success the PICC is halted after dumping the data.
 */
void picc_dump_mifare_classic_to_serial(uid_t *uid,			///< Pointer to Uid struct returned from a successful PICC_Select().
		picc_type_t piccType,	///< One of the PICC_Type enums.
												mifare_key_t *key		///< Key A used for all sectors.
											) {
	uint8_t no_of_sectors = 0;
	switch (piccType) {
		case PICC_TYPE_MIFARE_MINI:
			// Has 5 sectors * 4 blocks/sector * 16 bytes/block = 320 bytes.
			no_of_sectors = 5;
			break;

		case PICC_TYPE_MIFARE_1K:
			// Has 16 sectors * 4 blocks/sector * 16 bytes/block = 1024 bytes.
			no_of_sectors = 16;
			break;

		case PICC_TYPE_MIFARE_4K:
			// Has (32 sectors * 4 blocks/sector + 8 sectors * 16 blocks/sector) * 16 bytes/block = 4096 bytes.
			no_of_sectors = 40;
			break;

		default: // Should not happen. Ignore.
			break;
	}

	// Dump sectors, highest address first.
	if (no_of_sectors) {
		PRINTF("Sector Block   0  1  2  3   4  5  6  7   8  9 10 11  12 13 14 15  AccessBits\r\n");
		for (int8_t i = no_of_sectors - 1; i >= 0; i--) {
			picc_dump_mifare_classic_sector_to_serial(uid, key, i);
		}
	}
	picc_halt_A(); // Halt the PICC before stopping the encrypted session.
	pcd_stop_crypto_1();
}

/**
 * Dumps memory contents of a sector of a MIFARE Classic PICC.
 * Uses PCD_Authenticate(), MIFARE_Read() and PCD_StopCrypto1.
 * Always uses PICC_CMD_MF_AUTH_KEY_A because only Key A can always read the sector trailer access bits.
 */
void picc_dump_mifare_classic_sector_to_serial(uid_t *uid,mifare_key_t *key,uint8_t sector)
{
	uint8_t status;
	uint8_t firstBlock;		// Address of lowest address to dump actually last block dumped)
	uint8_t no_of_blocks;		// Number of blocks in sector
	bool isSectorTrailer;	// Set to true while handling the "last" (ie highest address) in the sector.

	// The access bits are stored in a peculiar fashion.
	// There are four groups:
	//		g[3]	Access bits for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
	//		g[2]	Access bits for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
	//		g[1]	Access bits for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
	//		g[0]	Access bits for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
	// Each group has access bits [C1 C2 C3]. In this code C1 is MSB and C3 is LSB.
	// The four CX bits are stored together in a nible cx and an inverted nible cx_.
	uint8_t c1, c2, c3;		// Nibbles
	uint8_t c1_, c2_, c3_;		// Inverted nibbles
	bool invertedError;		// True if one of the inverted nibbles did not match
	uint8_t g[4];				// Access bits for each of the four groups.
	uint8_t group;				// 0-3 - active group for access bits
	bool firstInGroup;		// True for the first block dumped in the group

	// Determine position and size of sector.
	if (sector < 32) { // Sectors 0..31 has 4 blocks each
		no_of_blocks = 4;
		firstBlock = sector * no_of_blocks;
	}
	else if (sector < 40) { // Sectors 32-39 has 16 blocks each
		no_of_blocks = 16;
		firstBlock = 128 + (sector - 32) * no_of_blocks;
	}
	else { // Illegal input, no MIFARE Classic PICC has more than 40 sectors.
		return;
	}

	// Dump blocks, highest address first.
	uint8_t byteCount;
	uint8_t buffer[18];
	uint8_t blockAddr;
	isSectorTrailer = true;
	invertedError = false;	// Avoid "unused variable" warning.
	for (int8_t blockOffset = no_of_blocks - 1; blockOffset >= 0; blockOffset--) {
		blockAddr = firstBlock + blockOffset;
		// Sector number - only on first line
		if (isSectorTrailer) {
			if(sector < 10)
				PRINTF("   "); // Pad with spaces
			else
				PRINTF("  "); // Pad with spaces
				PRINTF("%d",sector);
				PRINTF("   ");
		}
		else {
			PRINTF("       ");
		}
		// Block number
		if(blockAddr < 10)
			PRINTF("   "); // Pad with spaces
		else {
			if(blockAddr < 100)
				PRINTF("  "); // Pad with spaces
			else
				PRINTF(" "); // Pad with spaces
		}
		PRINTF("%d",blockAddr);
		PRINTF("  ");
		// Establish encrypted communications before reading the first block
		if (isSectorTrailer) {
			status = pcd_authenticate(PICC_CMD_MF_AUTH_KEY_A, firstBlock, key, uid);
			if (status != STATUS_OK) {
				PRINTF("PCD_Authenticate() failed: ");
				get_status_code_name(status);
				return;
			}
		}
		// Read block
		byteCount = sizeof(buffer);
		status = mifare_read(blockAddr, buffer, &byteCount);
		if (status != STATUS_OK) {
			PRINTF("MIFARE_Read() failed: ");
			get_status_code_name(status);
			PRINTF("\n");
			continue;
		}
		// Dump data
		for (uint8_t index = 0; index < 16; index++) {
			if(buffer[index] < 0x10)
				PRINTF(" 0");
			else
				PRINTF(" ");
			PRINTF("%x",buffer[index]);
			if ((index % 4) == 3) {
				PRINTF(" ");
			}
		}
		// Parse sector trailer data
		if (isSectorTrailer) {
			c1  = buffer[7] >> 4;
			c2  = buffer[8] & 0xF;
			c3  = buffer[8] >> 4;
			c1_ = buffer[6] & 0xF;
			c2_ = buffer[6] >> 4;
			c3_ = buffer[7] & 0xF;
			invertedError = (c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF));
			g[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
			g[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
			g[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
			g[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);
			isSectorTrailer = false;
		}

		// Which access group is this block in?
		if (no_of_blocks == 4) {
			group = blockOffset;
			firstInGroup = true;
		}
		else {
			group = blockOffset / 5;
			firstInGroup = (group == 3) || (group != (blockOffset + 1) / 5);
		}

		if (firstInGroup) {
			// Print access bits
			PRINTF(" [ ");
			PRINTF("%d",(g[group] >> 2) & 1); PRINTF(" ");
			PRINTF("%d",(g[group] >> 1) & 1); PRINTF(" ");
			PRINTF("%d",(g[group] >> 0) & 1);
			PRINTF(" ] ");
			if (invertedError) {
				PRINTF(" Inverted access bits did not match! ");
			}
		}

		if (group != 3 && (g[group] == 1 || g[group] == 6)) { // Not a sector trailer, a value block
			uint32_t value = ((uint32_t)(buffer[3])<<24) | ((uint32_t)(buffer[2])<<16) | ((uint32_t)(buffer[1])<<8) | (uint32_t)(buffer[0]);
			PRINTF(" Value=0x"); PRINTF("%x",value);
			PRINTF(" Adr=0x"); PRINTF("%x",buffer[12]);
		}
		PRINTF("\n");
	}

	return;
}

void get_status_code_name(uint8_t code	///< One of the StatusCode enums.
										) {
	switch (code) {
		case STATUS_OK:				PRINTF("Success.\n"); break;
		case STATUS_ERROR:			PRINTF("Error in communication.\n"); break;
		case STATUS_COLLISION:		PRINTF("Collission detected.\n")  ; break;
		case STATUS_TIMEOUT:		PRINTF("Timeout in communication.\n");break;
		case STATUS_NO_ROOM:		PRINTF("A buffer is not big enough.\n");break;
		case STATUS_INTERNAL_ERROR:	PRINTF("Internal error in the code. Should not happen.\n");break;
		case STATUS_INVALID:		PRINTF("Invalid argument.\n");break;
		case STATUS_CRC_WRONG:		PRINTF("The CRC_A does not match.\n");break;
		case STATUS_MIFARE_NACK:	PRINTF("A MIFARE PICC responded with NAK.\n");break;
		default:					PRINTF("Unknown error\n");
	}
} // End GetStatusCodeName()

/**
 * Executes the MFRC522 MFAuthent command.
 * This command manages MIFARE authentication to enable a secure communication to any MIFARE Mini, MIFARE 1K and MIFARE 4K card.
 * The authentication is described in the MFRC522 datasheet section 10.3.1.9 and http://www.nxp.com/documents/data_sheet/MF1S503x.pdf section 10.1.
 * For use with MIFARE Classic PICCs.
 * The PICC must be selected - ie in state ACTIVE(*) - before calling this function.
 * Remember to call PCD_StopCrypto1() after communicating with the authenticated PICC - otherwise no new communications can start.
 *
 * All keys are set to FFFFFFFFFFFFh at chip delivery.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise. Probably STATUS_TIMEOUT if you supply the wrong key.
 */
status_code_t pcd_authenticate(uint8_t command,		///< PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
		uint8_t blockAddr, 	///< The block number. See numbering in the comments in the .h file.
		mifare_key_t *key,	///< Pointer to the Crypteo1 key to use (6 bytes)
		uid_t *uid			///< Pointer to Uid struct. The first 4 bytes of the UID is used.
											) {
	uint8_t waitIRq = 0x10;		// IdleIRq

	// Build command buffer
	uint8_t sendData[12];
	sendData[0] = command;
	sendData[1] = blockAddr;
	for (uint8_t i = 0; i < MF_KEY_SIZE; i++) {	// 6 key bytes
		sendData[2+i] = key->keyByte[i];
	}
	// Use the last uid bytes as specified in http://cache.nxp.com/documents/application_note/AN10927.pdf
	// section 3.2.5 "MIFARE Classic Authentication".
	// The only missed case is the MF1Sxxxx shortcut activation,
	// but it requires cascade tag (CT) byte, that is not part of uid.
	for (uint8_t i = 0; i < 4; i++) {				// The last 4 bytes of the UID
		sendData[8+i] = uid->uidByte[i+uid->size-4];
	}

	// Start the authentication.
	return pcd_communicate_with_picc(PCD_MFAuthent, waitIRq, &sendData[0], sizeof(sendData),NULL,0,NULL, 0, false);
} // End PCD_Authenticate()

/**
 * Dumps memory contents of a MIFARE Ultralight PICC.
 */
void picc_dump_mifare_ultralight_to_serial(void)
{
	uint8_t status;
	uint8_t byteCount;
	uint8_t buffer[18];
	uint8_t i;

	PRINTF("Page  0  1  2  3\r\n");
	// Try the mpages of the original Ultralight. Ultralight C has more pages.
	for (uint8_t page = 0; page < 16; page +=4) { // Read returns data for 4 pages at a time.
		// Read pages
		byteCount = sizeof(buffer);
		status = mifare_read(page, buffer, &byteCount);
		if (status != STATUS_OK) {
			PRINTF("MIFARE_Read() failed: ");
//			Serial.println(GetStatusCodeName(status));
			break;
		}
		// Dump data
		for (uint8_t offset = 0; offset < 4; offset++) {
			i = page + offset;
			if(i < 10)
				PRINTF("  "); // Pad with spaces
			else
				PRINTF(" "); // Pad with spaces
			PRINTF("%d",i);
			PRINTF("  ");
			for (uint8_t index = 0; index < 4; index++) {
				i = 4 * offset + index;
				if(buffer[i] < 0x10)
					PRINTF(" 0");
				else
					PRINTF(" ");


				PRINTF("%x",buffer[i]);
			}
			PRINTF("\r\n");
		}
	}
}

/**
 * Reads 16 bytes (+ 2 bytes CRC_A) from the active PICC.
 *
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 *
 * For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
 * The MF0ICU1 returns a NAK for higher addresses.
 * The MF0ICU1 responds to the READ command by sending 16 bytes starting from the page address defined by the command argument.
 * For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
 * A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.
 *
 * The buffer must be at least 18 bytes because a CRC_A is also returned.
 * Checks the CRC_A before returning STATUS_OK.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code_t mifare_read(uint8_t blockAddr, 	///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
		uint8_t *buffer,		///< The buffer to store the data in
		uint8_t *bufferSize	///< Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
										) {
	status_code_t result;

	// Sanity check
	if (buffer == NULL || *bufferSize < 18) {
		return STATUS_NO_ROOM;
	}

	// Build command buffer
	buffer[0] = PICC_CMD_MF_READ;
	buffer[1] = blockAddr;
	// Calculate CRC_A
	result = pcd_calculate_crc(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}

	// Transmit the buffer and receive the response, validate CRC_A.
	return pcd_transceive_data(buffer, 4, buffer, bufferSize, NULL, 0, true);
} // End MIFARE_Read()

/**
 * MIFARE Decrement subtracts the delta from the value of the addressed block, and stores the result in a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code_t mifare_decrement(uint8_t blockAddr, ///< The block (0-0xff) number.
												int32_t delta		///< This number is subtracted from the value of block blockAddr.
											) {
	return mifare_two_step_helper(PICC_CMD_MF_DECREMENT, blockAddr, delta);
} // End MIFARE_Decrement()


/**
 * Ensure that a given block is formatted as a Value Block.
 */
void format_value_block(uint8_t blockAddr) {
    uint8_t buffer[18];
    uint8_t size = sizeof(buffer);
    status_code_t status;

    PRINTF("Reading block %d\r\n",blockAddr);
    status = mifare_read(blockAddr, buffer, &size);
    if (status != STATUS_OK) {
		PRINTF("MIFARE_Read() failed: ");
		get_status_code_name(status);
		return;
    }

    if (    (buffer[0] == (uint8_t)~buffer[4])
        &&  (buffer[1] == (uint8_t)~buffer[5])
        &&  (buffer[2] == (uint8_t)~buffer[6])
        &&  (buffer[3] == (uint8_t)~buffer[7])

        &&  (buffer[0] == buffer[8])
        &&  (buffer[1] == buffer[9])
        &&  (buffer[2] == buffer[10])
        &&  (buffer[3] == buffer[11])

        &&  (buffer[12] == (uint8_t)~buffer[13])
        &&  (buffer[12] ==        buffer[14])
        &&  (buffer[12] == (uint8_t)~buffer[15])) {
        PRINTF("Block has correct Value Block format.\r\n");
    }
    else {
    	PRINTF("Formatting as Value Block...\r\n");
        uint8_t valueBlock[] = {
            0, 0, 0, 0,
            255, 255, 255, 255,
            0, 0, 0, 0,
            blockAddr, ~blockAddr, blockAddr, ~blockAddr };
        status = mifare_write(blockAddr, valueBlock, 16);
        if (status != STATUS_OK) {
       		PRINTF("MIFARE_Write() failed: ");
       		get_status_code_name(status);
       		return;
        }
    }
}

/**
 * MIFARE Increment adds the delta to the value of the addressed block, and stores the result in a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code_t mifare_increment(uint8_t blockAddr, ///< The block (0-0xff) number.
												int32_t delta		///< This number is added to the value of block blockAddr.
											)
{
	return mifare_two_step_helper(PICC_CMD_MF_INCREMENT, blockAddr, delta);
} // End MIFARE_Increment()

/**
 * MIFARE Transfer writes the value stored in the volatile memory into one MIFARE Classic block.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code_t mifare_transfer(	uint8_t blockAddr ///< The block (0-0xff) number.
											) {
	status_code_t result;
	uint8_t cmdBuffer[2]; // We only need room for 2 bytes.

	// Tell the PICC we want to transfer the result into block blockAddr.
	cmdBuffer[0] = PICC_CMD_MF_TRANSFER;
	cmdBuffer[1] = blockAddr;
	result = pcd_mifare_transceive(	cmdBuffer, 2,false); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}
	return STATUS_OK;
} // End MIFARE_Transfer()

/**
 * Helper function for the two-step MIFARE Classic protocol operations Decrement, Increment and Restore.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code_t mifare_two_step_helper(uint8_t command,	///< The command to use
													uint8_t blockAddr,	///< The block (0-0xff) number.
													int32_t data		///< The data to transfer in step 2
													) {
	status_code_t result;
	uint8_t cmdBuffer[2]; // We only need room for 2 bytes.

	// Step 1: Tell the PICC the command and block address
	cmdBuffer[0] = command;
	cmdBuffer[1] = blockAddr;
	result = pcd_mifare_transceive(cmdBuffer, 2,false); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}

	// Step 2: Transfer the data
	result = pcd_mifare_transceive(	(uint8_t *)&data, 4, true); // Adds CRC_A and accept timeout as success.
	if (result != STATUS_OK) {
		return result;
	}

	return STATUS_OK;
} // End MIFARE_TwoStepHelper()

/**
 * Helper routine to write a specific value into a Value Block.
 *
 * Only for MIFARE Classic and only for blocks in "value block" mode, that
 * is: with access bits [C1 C2 C3] = [110] or [001]. The sector containing
 * the block must be authenticated before calling this function.
 *
 * @param[in]   blockAddr   The block (0x00-0xff) number.
 * @param[in]   value       New value of the Value Block.
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code_t mifare_set_value(uint8_t blockAddr, int32_t value) {
	uint8_t buffer[18];

	// Translate the int32_t into 4 bytes; repeated 2x in value block
	buffer[0] = buffer[ 8] = (value & 0xFF);
	buffer[1] = buffer[ 9] = (value & 0xFF00) >> 8;
	buffer[2] = buffer[10] = (value & 0xFF0000) >> 16;
	buffer[3] = buffer[11] = (value & 0xFF000000) >> 24;
	// Inverse 4 bytes also found in value block
	buffer[4] = ~buffer[0];
	buffer[5] = ~buffer[1];
	buffer[6] = ~buffer[2];
	buffer[7] = ~buffer[3];
	// Address 2x with inverse address 2x
	buffer[12] = buffer[14] = blockAddr;
	buffer[13] = buffer[15] = ~blockAddr;

	// Write the whole data block
	return mifare_write(blockAddr, buffer, 16);
} // End MIFARE_SetValue()

/**
 * Helper routine to read the current value from a Value Block.
 *
 * Only for MIFARE Classic and only for blocks in "value block" mode, that
 * is: with access bits [C1 C2 C3] = [110] or [001]. The sector containing
 * the block must be authenticated before calling this function.
 *
 * @param[in]   blockAddr   The block (0x00-0xff) number.
 * @param[out]  value       Current value of the Value Block.
 * @return STATUS_OK on success, STATUS_??? otherwise.
  */
status_code_t mifare_get_value(uint8_t blockAddr, int32_t *value) {
	status_code_t status;
	uint8_t buffer[18];
	uint8_t size = sizeof(buffer);

	// Read the block
	status = mifare_read(blockAddr, buffer, &size);
	if (status == STATUS_OK) {
		// Extract the value
		*value = (int32_t)(buffer[3] <<24) | (int32_t)(buffer[2]<<16) | (int32_t)(buffer[1]<<8) | (int32_t)(buffer[0]);

	}
	return status;
} // End MIFARE_GetValue()

/**
 * Calculates the bit pattern needed for the specified access bits. In the [C1 C2 C3] tuples C1 is MSB (=4) and C3 is LSB (=1).
 */
void mifare_set_access_bits(uint8_t *accessBitBuffer,	///< Pointer to byte 6, 7 and 8 in the sector trailer. Bytes [0..2] will be set.
		uint8_t g0,				///< Access bits [C1 C2 C3] for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
		uint8_t g1,				///< Access bits C1 C2 C3] for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
		uint8_t g2,				///< Access bits C1 C2 C3] for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
		uint8_t g3					///< Access bits C1 C2 C3] for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
								) {
	uint8_t c1 = ((g3 & 4) << 1) | ((g2 & 4) << 0) | ((g1 & 4) >> 1) | ((g0 & 4) >> 2);
	uint8_t c2 = ((g3 & 2) << 2) | ((g2 & 2) << 1) | ((g1 & 2) << 0) | ((g0 & 2) >> 1);
	uint8_t c3 = ((g3 & 1) << 3) | ((g2 & 1) << 2) | ((g1 & 1) << 1) | ((g0 & 1) << 0);

	accessBitBuffer[0] = (~c2 & 0xF) << 4 | (~c1 & 0xF);
	accessBitBuffer[1] =          c1 << 4 | (~c3 & 0xF);
	accessBitBuffer[2] =          c3 << 4 | c2;
} // End MIFARE_SetAccessBits()

/**
 * Writes 16 bytes to the active PICC.
 *
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 *
 * For MIFARE Ultralight the operation is called "COMPATIBILITY WRITE".
 * Even though 16 bytes are transferred to the Ultralight PICC, only the least significant 4 bytes (bytes 0 to 3)
 * are written to the specified address. It is recommended to set the remaining bytes 04h to 0Fh to all logic 0.
 * *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code_t mifare_write(uint8_t blockAddr, ///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The page (2-15) to write to.
		uint8_t *buffer,	///< The 16 bytes to write to the PICC
		uint8_t bufferSize	///< Buffer size, must be at least 16 bytes. Exactly 16 bytes are written.
										) {
	status_code_t result;

	// Sanity check
	if (buffer == NULL || bufferSize < 16) {
		return STATUS_INVALID;
	}

	// Mifare Classic protocol requires two communications to perform a write.
	// Step 1: Tell the PICC we want to write to block blockAddr.
	uint8_t cmdBuffer[2];
	cmdBuffer[0] = PICC_CMD_MF_WRITE;
	cmdBuffer[1] = blockAddr;
	result = pcd_mifare_transceive(cmdBuffer, 2,false); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}

	// Step 2: Transfer the data
	result = pcd_mifare_transceive(buffer, bufferSize,false); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}

	return STATUS_OK;
} // End MIFARE_Write()

/**
 * Dumps card info (UID,SAK,Type) about the selected PICC to Serial.
 *
 * @DEPRECATED kept for backward compatibility
 */
void picc_dump_details_to_serial(uid_t *uid	///< Pointer to Uid struct returned from a successful PICC_Select().
									) {
	// UID
	PRINTF("Card UID:");
	for (uint8_t i = 0; i < uid->size; i++) {
		if(uid->uidByte[i] < 0x10)
			PRINTF(" 0");
		else
			PRINTF(" ");
		PRINTF("%x",uid->uidByte[i]);
	}
	PRINTF("\r\n");

	// SAK
	PRINTF("Card SAK: ");
	if(uid->sak < 0x10)
		PRINTF(" 0");
	PRINTF("%x\n",uid->sak);

	// (suggested) PICC type
	picc_type_t piccType = picc_get_type(uid->sak);
	PRINTF("PICC type: ");
	picc_get_type_name(piccType);
	PRINTF("\n");
} // End PICC_DumpDetailsToSerial()


void picc_get_type_name(picc_type_t piccType) {
	switch (piccType) {
		case PICC_TYPE_ISO_14443_4:		PRINTF("PICC compliant with ISO/IEC 14443-4\n"); break;
		case PICC_TYPE_ISO_18092:		PRINTF("PICC compliant with ISO/IEC 18092 (NFC)\n"); break;
		case PICC_TYPE_MIFARE_MINI:		PRINTF("MIFARE Mini, 320 bytes\n"); break;
		case PICC_TYPE_MIFARE_1K:		PRINTF("MIFARE 1KB\n"); break;
		case PICC_TYPE_MIFARE_4K:		PRINTF("MIFARE 4KB\n"); break;
		case PICC_TYPE_MIFARE_UL:		PRINTF("MIFARE Ultralight or Ultralight C\n"); break;
		case PICC_TYPE_MIFARE_PLUS:		PRINTF("MIFARE Plus\n"); break;
		case PICC_TYPE_MIFARE_DESFIRE:	PRINTF("MIFARE DESFire\n");break;
		case PICC_TYPE_TNP3XXX:			PRINTF("MIFARE TNP3XXX\n");break;
		case PICC_TYPE_NOT_COMPLETE:	PRINTF("SAK indicates UID is not complete.\n");break;
		case PICC_TYPE_UNKNOWN:
		default:						PRINTF("Unknown type\n");
	}
} // End PICC_GetTypeName()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code_t picc_reqa(	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
											uint8_t *bufferSize	///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
										) {
	return picc_reqa_or_wupa(PICC_CMD_REQA, bufferATQA, bufferSize);
}


/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code_t picc_reqa_or_wupa(	uint8_t command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
												uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
												uint8_t *bufferSize	///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
											) {
	uint8_t validBits;
	status_code_t status;

	if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 uint8_ts long.
		return STATUS_NO_ROOM;
	}
	pcd_clear_register_bit_mask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) uint8_t. TxLastBits = BitFramingReg[2..0]
	status = pcd_transceive_data(&command, 1, bufferATQA, bufferSize, &validBits,0,0);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		//PRINTF("TQA must be exactly 16 bits.\r\n");
		return STATUS_ERROR;
	}
	return STATUS_OK;
}

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code_t pcd_transceive_data(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
													uint8_t sendLen,		///< Number of uint8_ts to transfer to the FIFO.
													uint8_t *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
													uint8_t *backLen,		///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
													uint8_t *validBits,	///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits. Default nullptr.
													uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
													bool checkCRC		///< In: True => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
								 ) {
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	return pcd_communicate_with_picc(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}


/**
 * Wrapper for MIFARE protocol communication.
 * Adds CRC_A, executes the Transceive command and checks that the response is MF_ACK or a timeout.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code_t pcd_mifare_transceive(uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A.
		uint8_t sendLen,		///< Number of bytes in sendData.
													bool acceptTimeout	///< True => A timeout is also success
												) {
	status_code_t result;
	uint8_t cmdBuffer[18]; // We need room for 16 bytes data and 2 bytes CRC_A.

	// Sanity check
	if (sendData == NULL || sendLen > 16) {
		return STATUS_INVALID;
	}

	// Copy sendData[] to cmdBuffer[] and add CRC_A
	memcpy(cmdBuffer, sendData, sendLen);
	result = pcd_calculate_crc(cmdBuffer, sendLen, &cmdBuffer[sendLen]);
	if (result != STATUS_OK) {
		return result;
	}
	sendLen += 2;

	// Transceive the data, store the reply in cmdBuffer[]
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	uint8_t cmdBufferSize = sizeof(cmdBuffer);
	uint8_t validBits = 0;
	result = pcd_communicate_with_picc(PCD_Transceive, waitIRq, cmdBuffer, sendLen, cmdBuffer, &cmdBufferSize, &validBits,0,false);
	if (acceptTimeout && result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result != STATUS_OK) {
		return result;
	}
	// The PICC must reply with a 4 bit ACK
	if (cmdBufferSize != 1 || validBits != 4) {
		return STATUS_ERROR;
	}
	if (cmdBuffer[0] != MF_ACK) {
		return STATUS_MIFARE_NACK;
	}
	return STATUS_OK;
} // End PCD_MIFARE_Transceive()


/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code_t pcd_communicate_with_picc(	uint8_t command,		///< The command to execute. One of the PCD_Command enums.
		uint8_t waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
		uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
		uint8_t sendLen,		///< Number of uint8_ts to transfer to the FIFO.
		uint8_t *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
		uint8_t *backLen,		///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
		uint8_t *validBits,	///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits.
		uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
		uint8_t checkCRC		///< In: True => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
									 )
{

	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

	spi_write_reg(SPI_RFID,CommandReg, PCD_Idle);			// Stop any active command.
	spi_write_reg(SPI_RFID,ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	spi_write_reg(SPI_RFID,FIFOLevelReg, 0x80);				// FlushBuffer = 1, FIFO initialization

	int sendData_l = 0;

	for(sendData_l = 0; sendData_l<sendLen;sendData_l++ )
		spi_write_reg(SPI_RFID,FIFODataReg,sendData[sendData_l]);  // Write sendData to the FIFO

	spi_write_reg(SPI_RFID,BitFramingReg, bitFraming);		// Bit adjustments
	spi_write_reg(SPI_RFID,CommandReg, command);				// Execute the command
	if (command == PCD_Transceive) {
		pcd_set_register_bit_mask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}
	vTaskDelay(1);    /* 1 ms */

	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86μs.
	// TODO check/modify for other architectures than Arduino Uno 16bit
	int i;
	uint8_t n=0;
	for (i = 20000; i > 0; i--) {

		spi_read_reg(SPI_RFID,ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		n = spi_rx_data[1];

		if (n  & waitIRq) {					// One of the interrupts that signal success has been set.
			break;
		}
		if (n  & 0x01) {						// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
	}
	// 35.7ms and nothing happend. Communication with the MFRC522 might be down.
	if (i == 0) {
		return STATUS_TIMEOUT;
	}

	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue;
	spi_read_reg(SPI_RFID,ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	errorRegValue = spi_rx_data[1];
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		PRINTF("status error\r\n");
//		scan_RFID_flag = false;
		return STATUS_ERROR;
	}

	uint8_t _validBits = 0;
	uint8_t h;

	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {

		spi_read_reg(SPI_RFID,FIFOLevelReg);	// Number of uint8_ts in the FIFO
		h = spi_rx_data[1];
		if (h > *backLen) {
			PRINTF("no room....\r\n");
			return STATUS_NO_ROOM;
		}
		*backLen = h;											// Number of uint8_ts returned

		int k;
		for(k=0 ; k<h;k++)
		{
			spi_read_reg(SPI_RFID,FIFODataReg);
			*(backData+k) =  spi_rx_data[1];

		}

		spi_read_reg(SPI_RFID,ControlReg);
		_validBits = spi_rx_data[1];
		_validBits = _validBits & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received uint8_t. If this value is 000b, the whole uint8_t is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}

	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		PRINTF("status collision\r\n");
		return STATUS_COLLISION;
	}

//	// Perform CRC_A validation if requested.
//	if (backData && backLen && checkCRC) {
//		// In this case a MIFARE Classic NAK is not OK.
//		if (*backLen == 1 && _validBits == 4) {
//			return STATUS_MIFARE_NACK;
//		}
//		// We need at least the CRC_A value and all 8 bits of the last uint8_t must be received.
//		if (*backLen < 2 || _validBits != 0) {
//			return STATUS_CRC_WRONG;
//		}
//		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
//		uint8_t controlBuffer[2];
//		status_code_t status = pcd_calculate_crc(&backData[0], *backLen - 2, &controlBuffer[0]);
//		if (status != STATUS_OK) {
//			return status;
//		}
//		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
//			return STATUS_CRC_WRONG;
//		}
//	}

	return STATUS_OK;
}

/**
 * Sets the bits given in mask in register reg.
 */
void pcd_set_register_bit_mask(	pcd_reg_t reg,	///< The register to update. One of the pcd_reg_t enums.
										uint8_t mask			///< The bits to set.
									) {
//	uint8_t tmp;
	spi_read_reg(SPI_RFID,reg);
	spi_write_reg(SPI_RFID,reg, spi_rx_data[1] | mask);			// set bit mask
}

/**
 * Clears the bits given in mask from register reg.
 */
void pcd_clear_register_bit_mask(pcd_reg_t reg,	///< The register to update. One of the PCD_Register enums.
		uint8_t mask			///< The bits to clear.
									  ) {
//	uint8_t tmp;
	spi_read_reg(SPI_RFID,reg);
	spi_write_reg(SPI_RFID,reg, spi_rx_data[1] & (~mask)); // clear bit mask
}

/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code_t pcd_calculate_crc(	uint8_t *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
												uint8_t length,	///< In: The number of uint8_ts to transfer.
												uint8_t *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low uint8_t first.
					 ) {



	int sendData_l = 0;

	spi_write_reg(SPI_RFID,CommandReg, PCD_Idle);		// Stop any active command.
	spi_write_reg(SPI_RFID,DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	spi_write_reg(SPI_RFID,FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization

	for(sendData_l = 0; sendData_l<length;sendData_l++ )
		spi_write_reg(SPI_RFID,FIFODataReg, data[sendData_l]);   // Write data to the FIFO

	spi_write_reg(SPI_RFID,CommandReg, PCD_CalcCRC);		// Start the calculation

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73μs.
	// TODO check/modify for other architectures than Arduino Uno 16bit

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us.
	for (uint16_t i = 5000; i > 0; i--) {
		// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		uint8_t n;
		spi_read_reg(SPI_RFID,DivIrqReg);
		n = spi_rx_data[1];

		if (n & 0x04) {									// CRCIRq bit set - calculation done
			spi_write_reg(SPI_RFID,CommandReg, PCD_Idle);	// Stop calculating CRC for new content in the FIFO.
			// Transfer the result from the registers to the result buffer
			spi_read_reg(SPI_RFID,CRCResultRegL);
			result[0] = spi_rx_data[1];
			spi_read_reg(SPI_RFID,CRCResultRegH);
			result[1] = spi_rx_data[1];
			return STATUS_OK;
		}
	}
	// 89ms passed and nothing happend. Communication with the MFRC522 might be down.
	return STATUS_TIMEOUT;
}

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
status_code_t picc_halt_A(void) {
	status_code_t result;
	uint8_t buffer[4];

	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = pcd_calculate_crc(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}

	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	//result = pcd_transceive_data(buffer, sizeof(buffer), NULL, 0,0,0,0);
	result = pcd_transceive_data(buffer, sizeof(buffer), NULL, 0,NULL,0,false);
	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
}

//Write specific block
status_code_t write_block(uint8_t blockNumber, uint8_t *buffer)
{

	status_code_t status;

  //this makes sure that we only write into data blocks. Every 4th block is a trailer block for the access/security info.
  int largestModulo4Number=blockNumber/4*4;
  uint8_t trailerBlock=largestModulo4Number+3;//determine trailer block for the sector
  if (blockNumber > 2 && (blockNumber+1)%4 == 0)
  {
	  PRINTF("%d is a trailer block\r\n",blockNumber);
	  return STATUS_ERROR;
  }
  PRINTF("%d is a data block\r\n",blockNumber);

  //authentication of the desired block for access
  status = pcd_authenticate(PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(uid));
  if (status != STATUS_OK) {
	  get_status_code_name(status);
      return STATUS_ERROR;
  }

  //writing the block
  status = mifare_write(blockNumber, (char*)buffer, 16);

  if (status != STATUS_OK) {
	  get_status_code_name(status);
	  return STATUS_ERROR;
  }
}

status_code_t read_block(uint8_t blockNumber, uint8_t *buffer)
{
status_code_t status;
  int largestModulo4Number=blockNumber/4*4;
  uint8_t trailerBlock=largestModulo4Number+3;//determine trailer block for the sector

  //authentication of the desired block for access
   status = pcd_authenticate(PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(uid));
   if (status != STATUS_OK) {
 	  get_status_code_name(status);
       return STATUS_ERROR;
   }

//reading a block
   uint8_t buffersize = 18;//we need to define a variable with the read buffer size, since the MIFARE_Read method below needs a pointer to the variable that contains the size...
   status = mifare_read(blockNumber, buffer, &buffersize);//&buffersize is a pointer to the buffersize variable; MIFARE_Read requires a pointer instead of just a number
   if (status != STATUS_OK) {
   	  get_status_code_name(status);
   	  return STATUS_ERROR;
     }
}


/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void pcd_reset(void) {
	//uint8_t read_value;
	spi_write_reg(SPI_RFID,CommandReg, PCD_SoftReset);	// Issue the SoftReset command.
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg)
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
	uint8_t count = 0;

	spi_read_reg(SPI_RFID,CommandReg);

	do {
		// Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
		//SysTick_DelayTicks(2192982);    /* 50 ms */
		vTaskDelay(50);
		//delay(50);
	} while ((spi_rx_data[1] & (1 << 4)) && (++count) < 3);
}
