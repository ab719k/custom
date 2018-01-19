// basic defines for the CP2120 SPI slave to I2C master bridge

#define CP2120_WORD_SIZE	1
#define CP2120_SPI_SPEED	1
#define CP2120_SPI_MODE		2

// some commands
#define CP2120_CMD_I2C_WRITE		0x00
#define CP2120_CMD_I2C_READ		0x01
#define CP2120_CMD_I2C_RD_AFTER_WR	0x02	// write to one slave, then read from it
#define CP2120_CMD_I2C_WR_AFTER_WR	0x03	// write to one slave, then write to another
#define CP2120_CMD_READ_BUF		0x06 	// read the data from the rx buffer
#define CP2120_CMD_WR_TO_MULTIPLE	0x09	// write same data to multiple slaves
#define CP2120_CMD_SPI_CFG		0x18	// tell whether MSB or LSB
#define CP2120_CFG_MSB			0x81
#define CP2120_CFG_LSB			0x42
#define CP2120_CMD_WRITE_REG		0x20	// one byte write
#define CP2120_CMD_READ_REG		0x21	// one byte read
#define CP2120_CMD_REV_NUM		0x40	// returns two-byte revision number

// registers
#define CP2120_REG_IOCONFIG		0x00	// set pins as input, output, or quasi-bidirectional (0-3)
#define CP2120_REG_IOSTATE		0x01	// read/write state of GPIO
#define CP2120_REG_I2CCLOCK		0x02	// set I2C clock freq = 2000/(regValue:(5-255)) kHz
#define CP2120_REG_I2CTO		0x03	// set I2C timeout (if zero, only one transaction attempt made)
#define CP2120_REG_I2CSTAT		0x04	// I2C status
#define CP2120_I2C_SUCCESS		0xF0
#define CP2120_I2C_SLAVE_ADDR_NACK	0xF1
#define CP2120_I2C_SLAVE_DATA_NACK	0xF2
#define CP2120_I2C_IN_PROGRESS		0xF3
#define CP2120_I2C_TIMEOUT		0xF8
#define CP2120_I2C_TX_RX_BUF_MISMATCH	0xF9
#define CP2120_I2C_SCL_LOW_TIMEOUT	0xFA
#define CP2120_I2C_BUS_NOT_FREE		0xFB
#define CP2120_REG_I2CADR		0x05	// the I2C address (reset value 0x00)
#define CP2120_REG_RXBUFF		0x06	// how many bytes stored in the rx buffer
#define CP2120_REG_IOCONFIG2		0x07	// set pins as input, output, or quasi-bidirectional (4-7)
#define CP2120_REG_EDGEINT		0x08	// enable interrupt generation on EI_INT pin, which will trigger INT pin
#define CP2120_EI_FLAG			0x80	// 0 indicates no event, 1 indicates an event has occurred on the EI_INT pin
#define CP2120_EI_EN			0x40	// enable edge-triggered interrupts
#define CP2120_EI_POS_TO_NEG		0x20	// if this is set, interrupts are on a falling edge	
#define CP2120_REG_I2CTO2		0x09	// second I2C timeout - configure SMBus polling


// this as a macro is probably a bad idea
/*.macro CP2120_RESET
	CLR	CS_PINS, CP2120_RST_PIN
	LDI	ITER, 4000
cp2120RstLoop:
	QBEQ	cp2120RstLoopEnd, ITER, 0
        SUB	ITER, ITER, 1
        JMP	cp2120RstLoop	
cp2120RstLoopEnd:
	SET	CS_PINS, CP2120_RST_PIN
.endm */

