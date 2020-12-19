/*
 Name:		SpiSlave32.ino
 Created:	15/12/2020 22:44:11
 Author:	Y@@J and many others...

	SPI sniffer for the SSD1306/SSD1309 ; for marlin UI, u8glib
	rebuilds the bitmap, displays the data as ASCII art if __SERIAL_DEBUG

	thread on stm32duino.com :
	https://www.stm32duino.com/viewtopic.php?f=47&t=815&e=1&view=unread#unread
	
	usefull links :
	https://web.archive.org/web/20190316162926/https://www.stm32duino.com/viewtopic.php?f=9&t=2846&start=10
	https://github.com/rogerclarkmelbourne/Arduino_STM32/pull/503
	http://stm32duinoforum.com/forum/viewtopic_f_14_t_3527_start_10.html
	https://web.archive.org/web/20190316162929/https://www.stm32duino.com/viewtopic.php?f=9&t=2846
	https://web.archive.org/web/20190316155730/https://github.com/stevstrong/Audio-sample/blob/master/stm32/STM32_ADC_Host.ino
	https://github.com/stevstrong/Audio-sample/blob/master/stm32/STM32_ADC_Host.ino
	https://www.stm32duino.com/viewtopic.php?f=9&t=2846&start=10
	https://www.stm32duino.com/viewtopic.php?f=2&t=3
	https://sparklogic.ru/libraries-hardware/hard-time-to-make-spi-work-in-slave-mode.html

	SSD1306-9			SPI			STM32

	SCL					SCLK		PA5			/!\	NOT 5V TOLERANT /!\
	SDA					MOSI		PA7			/!\	NOT 5V TOLERANT /!\
	RES					N/A			not used
	DC					N/A			not used
	SS					SS			PA4			/!\	NOT 5V TOLERANT /!\

	SS : LOW for pages, including commands ; HIGH between pages and between frames
	DC : LOW for commands only

	SPI data :

	1 SPI frame = 8 pages
	1 page = 128 pixels
	1 page : 3 command bytes + 128 bytes (= 8 lines on the display)
	1 command byte = 0x10 0x00 0XBn , n = page # (0...7)
	total : 8 pages ; 8 * 131 = 1048 bytes, CLK @ 1MHz

	SPI : SPI1, DMA1, DMA_CH2, SPI_MODE0, DATA_SIZE_8BIT
*/

#include <SPI.h>

#define __SERIAL_DEBUG
#define BAUD_RATE 250000

// data (SSD1306/SSD1309) : 1 frame = 8 pages, 1 page = 3 command bytes + 128 bytes, 1 page = 8 graphic lines
#define GFX_PAGE_COUNT			8
#define	GFX_LINES_PER_PAGE		8
#define GFX_CMD_SIZE			3
#define GFX_PAGE_SIZE			128
#define SPI_PAGE_SIZE			(GFX_CMD_SIZE + GFX_PAGE_SIZE)
#define SPI_FRAME_SIZE			(GFX_PAGE_COUNT * SPI_PAGE_SIZE)

// SPI Rx buffer
#define DMA_BUFFER_SIZE (2 * SPI_FRAME_SIZE) // for 2 frames (DMA tube configuration with DMA_CFG_CIRC)
volatile uint8_t rx_buffer[DMA_BUFFER_SIZE] = { 0 };

// bitmap
volatile uint8_t bitmap[GFX_PAGE_COUNT][GFX_PAGE_SIZE] = { 0 };

// ISR
volatile bool dma_transfer_complete = false;

void rxDMAirq(void)
{
	dma_irq_cause cause = dma_get_irq_cause(DMA1, DMA_CH2);

	if (cause == DMA_TRANSFER_COMPLETE) // also resets DMA bits
	{
		dma_transfer_complete = true;
		return;
	}

#ifdef __SERIAL_DEBUG
	Serial.print("rxDMAirq error : ");
	Serial.println(cause);
#endif
}

// decoding function : copy SPI buffer to bitmap, removes the command bytes
// 30µs (vs 480µs with the previous one)
bool rebuildBitmap()
{
	for (int i = 0; i < GFX_PAGE_COUNT; i++)
	{
		uint8_t* pRxBuf = (uint8_t*)rx_buffer + SPI_FRAME_SIZE + i * SPI_PAGE_SIZE;
		uint16_t cmd = *(uint16_t*)pRxBuf; // command bytes
		uint8_t pageNum = *(uint8_t*)(pRxBuf + 2);

		// test 1st 3 bytes (= command bytes) of each frame ; 0x10 0x00 0xBn , n = frame #
		if(cmd != 0x10 || pageNum != i + 0xB0)
			return false; // error, not in sync, etc.

		// copy pages to bitmap, drop command bytes
		memcpy((uint8_t*)&bitmap + i * GFX_PAGE_SIZE, pRxBuf + GFX_CMD_SIZE, GFX_PAGE_SIZE);
	}
	return true;
}

// setup functions
void setup()
{
#ifdef __SERIAL_DEBUG
	Serial.begin(BAUD_RATE);
#endif
	delay(100);
	setupSPI();
	setupDMA();
}

void setupSPI(void)
{
	// MOSI, MISO, SCK pins are set by the library
	pinMode(BOARD_SPI_DEFAULT_SS, INPUT);
	
	// number of the SPI peripheral : ????
	SPI.setModule(1);

	// The clock value is not used, SPI1 is selected by default
	SPISettings spiSettings(0, MSBFIRST, SPI_MODE0, DATA_SIZE_8BIT);

	SPI.beginTransactionSlave(spiSettings);

	// Clear Rx register in case we already received SPI data
	spi_rx_reg(SPI.dev());
}

void setupDMA(void)
{
	dma_init(DMA1);						// SPI is on DMA1
	spi_rx_dma_disable(SPI.dev());		// Disable in case already enabled
	dma_disable(DMA1, DMA_CH2);			// Disable in case it was already enabled
	dma_detach_interrupt(DMA1, DMA_CH2);

	// DMA tube configuration for SPI1 Rx
	dma_tube_config rx_tube_cfg =
	{
		&SPI1->regs->DR,		// Source of data
		DMA_SIZE_8BITS,			// Source transfer size
		&rx_buffer,				// Destination of data
		DMA_SIZE_8BITS,			// Destination transfer size
		DMA_BUFFER_SIZE,		// Number of data to transfer
								// Flags :
		DMA_CFG_DST_INC |		// auto increment destination address
		DMA_CFG_CIRC |		// circular buffer
		DMA_CFG_CMPLT_IE |		// set tube full IRQ
		DMA_CCR_PL_VERY_HIGH,	// very high priority
		0,						// reserved
		DMA_REQ_SRC_SPI1_RX		// Hardware DMA request source
	};

	// SPI1 Rx is channel 2
	int ret_rx = dma_tube_cfg(DMA1, DMA_CH2, &rx_tube_cfg);

	if (ret_rx != DMA_TUBE_CFG_SUCCESS)
	{
		while (1)
		{
#ifdef __SERIAL_DEBUG
			Serial.print("Rx DMA configuration error: ");
			Serial.println(ret_rx, HEX);
			Serial.println("Reset is needed!");
			delay(100);
#else
			// will trigger watchdog...
#endif
		}
	}

	spi_rx_reg(SPI.dev());							// Clear RX register in case we already received SPI data
	dma_attach_interrupt(DMA1, DMA_CH2, rxDMAirq);	// Attach interrupt to catch end of DMA transfer
	dma_enable(DMA1, DMA_CH2);						// Rx : Enable DMA configurations
	spi_rx_dma_enable(SPI.dev());					// Tx : SPI DMA requests for Rx and Tx 
}

// loop() and display
void loop()
{
	if (dma_transfer_complete)
	{
#ifdef __SERIAL_DEBUG
		if (rebuildBitmap())
			printFrame();
#else
		rebuildBitmap();
#endif
		dma_transfer_complete = false;
	}

	delay(500);
}

#ifdef __SERIAL_DEBUG

// Serial.print() bitmap as ASCII art
void printFrame()
{
	for (int page = 0; page < 8; page++)
	{
		for (int line = 0; line < GFX_LINES_PER_PAGE; line++)
		{
			for (int col = 0; col < GFX_PAGE_SIZE; col++)
			{
				uint8_t val = bitmap[page][col];
				if (val & (1 << line))
					Serial.print('#');
				else
					Serial.print(' ');
			}

			Serial.println();
		}
	}
	for (int i = 0; i < 128; i++)
		Serial.print('X');

	Serial.println();
}

#endif

// END OF FILE
