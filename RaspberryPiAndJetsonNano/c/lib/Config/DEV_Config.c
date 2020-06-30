/*****************************************************************************
* | File      	:   DEV_Config.c
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*----------------
* |	This version:   V3.0
* | Date        :   2019-07-31
* | Info        :   
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of theex Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#include "DEV_Config.h"
#include <fcntl.h>

/**
 * GPIO
**/
int EPD_RST_PIN;
int EPD_DC_PIN;
int EPD_CS_PIN;
int EPD_BUSY_PIN;

#ifdef INTEL_EDISON
	mraa_gpio_context gpioReset;
	mraa_gpio_context gpioDataCommand;
	mraa_gpio_context gpioChipSelect;
	mraa_gpio_context gpioBusy;
#endif

/**
 * GPIO read and write
**/
void DEV_Digital_Write(UWORD Pin, UBYTE Value)
{
#ifdef RPI
#ifdef USE_BCM2835_LIB
	bcm2835_gpio_write(Pin, Value);
#elif USE_WIRINGPI_LIB
	digitalWrite(Pin, Value);
#elif USE_DEV_LIB
	SYSFS_GPIO_Write(Pin, Value);
#endif
#endif

#ifdef JETSON
#ifdef USE_DEV_LIB
	SYSFS_GPIO_Write(Pin, Value);
#elif USE_HARDWARE_LIB
	Debug("not support");
#endif
#endif

#ifdef INTEL_EDISON

	//catch any call to the CS pin since that is controlled by MRAA
	//if(Pin == EPD_CS_PIN) {
	//	fprintf(stderr, "Bypassed write to CS on Edison GPIO%d\n", Pin);
	//	return;
	//}

	mraa_gpio_context gpioEdison;

	if(Pin == EPD_RST_PIN)
		gpioEdison = gpioReset;
	else if(Pin == EPD_DC_PIN)
		gpioEdison = gpioDataCommand;
	else if(Pin == EPD_CS_PIN)
		gpioEdison = gpioChipSelect;
	else if(Pin == EPD_BUSY_PIN)
		gpioEdison = gpioBusy;

	mraa_result_t status;

	status = mraa_gpio_write(gpioEdison, Value);

	if(status != MRAA_SUCCESS) {
		fprintf(stderr, "Failed to write from Edison GPIO%d\n", Pin);
		return;
	}
#endif
}

UBYTE DEV_Digital_Read(UWORD Pin)
{
	UBYTE Read_value = 0;
#ifdef RPI
#ifdef USE_BCM2835_LIB
	Read_value = bcm2835_gpio_lev(Pin);
#elif USE_WIRINGPI_LIB
	Read_value = digitalRead(Pin);
#elif USE_DEV_LIB
	Read_value = SYSFS_GPIO_Read(Pin);
#endif
#endif

#ifdef JETSON
#ifdef USE_DEV_LIB
	Read_value = SYSFS_GPIO_Read(Pin);
#elif USE_HARDWARE_LIB
	Debug("not support");
#endif
#endif

#ifdef INTEL_EDISON
	mraa_gpio_context gpioEdison;

	if(Pin == EPD_RST_PIN)
		gpioEdison = gpioReset;
	else if(Pin == EPD_DC_PIN)
		gpioEdison = gpioDataCommand;
	else if(Pin == EPD_CS_PIN)
		gpioEdison = gpioChipSelect;
	else if(Pin == EPD_BUSY_PIN)
		gpioEdison = gpioBusy;

	Read_value = mraa_gpio_read(gpioEdison);

#endif
	return Read_value;
}

/**
 * SPI
**/
void DEV_SPI_WriteByte(uint8_t Value)
{
#ifdef RPI
#ifdef USE_BCM2835_LIB
	bcm2835_spi_transfer(Value);
#elif USE_WIRINGPI_LIB
	wiringPiSPIDataRW(0,&Value,1);
#elif USE_DEV_LIB
	DEV_HARDWARE_SPI_TransferByte(Value);
#elif INTEL_EDISON
#endif
#endif

#ifdef JETSON
#ifdef USE_DEV_LIB
	SYSFS_software_spi_transfer(Value);
#elif USE_HARDWARE_LIB
	Debug("not support");
#endif
#endif

#ifdef INTEL_EDISON
	mraa_spi_write(edisonSPI, Value);
#endif
}

void DEV_SPI_Write_nByte(uint8_t *pData, uint32_t Len)
{
#ifdef RPI
#ifdef USE_BCM2835_LIB
	char rData[Len];
	bcm2835_spi_transfernb(pData,rData,Len);
#elif USE_WIRINGPI_LIB
	wiringPiSPIDataRW(0, pData, Len);
#elif USE_DEV_LIB
	DEV_HARDWARE_SPI_Transfer(pData, Len);
#endif
#endif

#ifdef JETSON
#ifdef USE_DEV_LIB
	//JETSON nano waits for hardware SPI
	Debug("not support");
#elif USE_HARDWARE_LIB
	Debug("not support");
#endif
#endif

#ifdef INTEL_EDISON
	mraa_spi_write_buf(edisonSPI, pData, Len);
#endif
}

/**
 * GPIO Mode
**/
void DEV_GPIO_Mode(UWORD Pin, UWORD Mode)
{
#ifdef RPI
#ifdef USE_BCM2835_LIB
	if(Mode == 0 || Mode == BCM2835_GPIO_FSEL_INPT) {
		bcm2835_gpio_fsel(Pin, BCM2835_GPIO_FSEL_INPT);
	} else {
		bcm2835_gpio_fsel(Pin, BCM2835_GPIO_FSEL_OUTP);
	}
#elif USE_WIRINGPI_LIB
	if(Mode == 0 || Mode == INPUT) {
		pinMode(Pin, INPUT);
		pullUpDnControl(Pin, PUD_UP);
	} else {
		pinMode(Pin, OUTPUT);
		// Debug (" %d OUT \r\n",Pin);
	}
#elif USE_DEV_LIB
	SYSFS_GPIO_Export(Pin);
	if(Mode == 0 || Mode == SYSFS_GPIO_IN) {
		SYSFS_GPIO_Direction(Pin, SYSFS_GPIO_IN);
		// Debug("IN Pin = %d\r\n",Pin);
	} else {
		SYSFS_GPIO_Direction(Pin, SYSFS_GPIO_OUT);
		// Debug("OUT Pin = %d\r\n",Pin);
	}
#endif
#endif

#ifdef JETSON
#ifdef USE_DEV_LIB
	SYSFS_GPIO_Export(Pin);
	SYSFS_GPIO_Direction(Pin, Mode);
#elif USE_HARDWARE_LIB
	Debug("not support");
#endif
#endif

#ifdef INTEL_EDISON
	// Refer to edison spi.pdf, page 22 for a table that translates arduino shield IO pin numbers to the GPxxx pin numbers used by Edison
	fprintf(stderr, "Edison GPIO%d ", Pin);

	mraa_gpio_context gpioEdison;
	mraa_result_t status;

	if(Pin == EPD_RST_PIN) {
		gpioReset = mraa_gpio_init(Pin);

		gpioEdison = gpioReset;
	}
	else if(Pin == EPD_DC_PIN) {
		gpioDataCommand = mraa_gpio_init(Pin);

		gpioEdison = gpioDataCommand;
	}
	else if(Pin == EPD_CS_PIN) {
		gpioChipSelect = mraa_gpio_init(Pin);

		gpioEdison = gpioChipSelect;
	}
	else if(Pin == EPD_BUSY_PIN) {
		gpioBusy = mraa_gpio_init(Pin);

		gpioEdison = gpioBusy;
	}

	if(gpioEdison == NULL) {
		fprintf(stderr, "failed to initialize\n");
		return;
	}

	if(Mode == !MRAA_GPIO_OUT) {
		status = mraa_gpio_mode(gpioEdison, MRAA_GPIO_PULLUP);

		printf("OUT Mode status: %d\r\n", status);

		status = mraa_gpio_dir(gpioEdison, MRAA_GPIO_OUT);

		printf("Out DIR status: %d\r\n", status);
	}
	else {
		status = mraa_gpio_dir(gpioEdison, MRAA_GPIO_IN);

		printf("IN DIR status: %d\r\n", status);
	}

	if(status != MRAA_SUCCESS) {
		fprintf(stderr, "failed to set pin mode and direction\r\n");
		return;
	}
	else {
		printf("initialized OK\r\n");
	}

#endif
}

/**
 * delay x ms
**/
void DEV_Delay_ms(UDOUBLE xms)
{
#ifdef RPI
#ifdef USE_BCM2835_LIB
	bcm2835_delay(xms);
#elif USE_WIRINGPI_LIB
	delay(xms);
#elif USE_DEV_LIB
	UDOUBLE i;
	for(i=0; i < xms; i++) {
		usleep(1000);
	}
#endif
#endif

#ifdef JETSON
	UDOUBLE i;
	for(i=0; i < xms; i++) {
		usleep(1000);
	}
#endif

#ifdef INTEL_EDISON
	UDOUBLE i;
	for(i=0; i < xms; i++) {
		usleep(1000);
	}
#endif
}

static int DEV_Equipment_Testing(void)
{
	int i;
	int fd;
	char value_str[20];
	fd = open("/etc/issue", O_RDONLY);
    printf("Current environment: ");
	while(1) {
		if (fd < 0) {
			Debug( "Read failed Pin\n");
			return -1;
		}
		for(i=0;; i++) {
			if (read(fd, &value_str[i], 1) < 0) {
				Debug( "failed to read value!\n");
				return -1;
			}
			if(value_str[i] ==32) {
				printf("\r\n");
				break;
			}
			printf("%c",value_str[i]);
		}
		break;
	}
#ifdef RPI
	if(i<5) {
		printf("Unrecognizable\r\n");
	} else {
		char RPI_System[10]   = {"Raspbian"};
		for(i=0; i<6; i++) {
			if(RPI_System[i]!= value_str[i]) {
				printf("Please make JETSON !!!!!!!!!!\r\n");
				return -1;
			}
		}
	}
#endif
#ifdef JETSON
	if(i<5) {
		Debug("Unrecognizable\r\n");
	} else {
		char JETSON_System[10]= {"Ubuntu"};
		for(i=0; i<6; i++) {
			if(JETSON_System[i]!= value_str[i] ) {
				printf("Please make RPI !!!!!!!!!!\r\n");
				return -1;
			}
		}
	}
#endif
	return 0;
}



void DEV_GPIO_Init(void)
{
#ifdef RPI
	EPD_RST_PIN     = 17;
	EPD_DC_PIN      = 25;
	EPD_CS_PIN      = 8;
	EPD_BUSY_PIN    = 24;
#elif JETSON
	EPD_RST_PIN     = GPIO17;
	EPD_DC_PIN      = GPIO25;
	EPD_CS_PIN      = SPI0_CS0;
	EPD_BUSY_PIN    = GPIO24;
#elif INTEL_EDISON
	/* drowan_NOTES_20200620: on the Edison arduino dev board, the pin numbers seem to correspond to the arduino header pin numbers.
	 * For example: MRAA_INTEL_EDISON_GP20 = 7, means the Edison pin maps to arduino header GPIO pin 7
	*/
	/*
	 * 9 J17-10 GP111 GPIO-111 SPI-5-CS1
	 * 10 J17-11 GP109 GPIO-109 SPI-5-SCK
	 * 11 J17-12 GP115 GPIO-115 SPI-5-MOSI
	 * 23 J18-10 GP110 GPIO-110 SPI-5-CS0
	 * 24 J18-11 GP114 GPIO-114 SPI-5-MISO
	 *
Fairhead, Harry. Explore Intel Edison (p. 154). I/O Press. Kindle Edition.

	MRAA_INTEL_EDISON_GP135 = 4,
    MRAA_INTEL_EDISON_GP27 = 6,
    MRAA_INTEL_EDISON_GP20 = 7,
    MRAA_INTEL_EDISON_GP28 = 8,
	 */
	EPD_RST_PIN     = MRAA_INTEL_EDISON_GP135;  // pin 4
	EPD_DC_PIN      = MRAA_INTEL_EDISON_GP27; // pin 6
	EPD_CS_PIN      = MRAA_INTEL_EDISON_GP20; // pin 7
	EPD_BUSY_PIN    = MRAA_INTEL_EDISON_GP28;  // pin 8
#endif

	DEV_GPIO_Mode(EPD_RST_PIN, 1);
	DEV_GPIO_Mode(EPD_DC_PIN, 1);
	DEV_GPIO_Mode(EPD_CS_PIN, 1);
	DEV_GPIO_Mode(EPD_BUSY_PIN, 0);

	DEV_Digital_Write(EPD_CS_PIN, 1);
}
/******************************************************************************
function:	Module Initialize, the library and initialize the pins, SPI protocol
parameter:
Info:
******************************************************************************/
UBYTE DEV_Module_Init(void)
{
    printf("/***********************************/ \r\n");
	if(DEV_Equipment_Testing() < 0) {
		return 1;
	}
#ifdef RPI
#ifdef USE_BCM2835_LIB
	if(!bcm2835_init()) {
		printf("bcm2835 init failed  !!! \r\n");
		return 1;
	} else {
		printf("bcm2835 init success !!! \r\n");
	}

	// GPIO Config
	DEV_GPIO_Init();

	bcm2835_spi_begin();                                         //Start spi interface, set spi pin for the reuse function
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);     //High first transmission
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                  //spi mode 0
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128);  //Frequency
	bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                     //set CE0
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);     //enable cs0

#elif USE_WIRINGPI_LIB
	//if(wiringPiSetup() < 0)//use wiringpi Pin number table
	if(wiringPiSetupGpio() < 0) { //use BCM2835 Pin number table
		printf("set wiringPi lib failed	!!! \r\n");
		return 1;
	} else {
		printf("set wiringPi lib success !!! \r\n");
	}

	// GPIO Config
	DEV_GPIO_Init();
	wiringPiSPISetup(0,10000000);
	// wiringPiSPISetupMode(0, 32000000, 0);
#elif USE_DEV_LIB
	printf("Write and read /dev/spidev0.0 \r\n");
	DEV_GPIO_Init();
	DEV_HARDWARE_SPI_begin("/dev/spidev0.0");
    DEV_HARDWARE_SPI_setSpeed(10000000);
#endif

#elif JETSON
#ifdef USE_DEV_LIB
	DEV_GPIO_Init();
	printf("Software spi\r\n");
	SYSFS_software_spi_begin();
	SYSFS_software_spi_setBitOrder(SOFTWARE_SPI_MSBFIRST);
	SYSFS_software_spi_setDataMode(SOFTWARE_SPI_Mode0);
	SYSFS_software_spi_setClockDivider(SOFTWARE_SPI_CLOCK_DIV4);
#elif USE_HARDWARE_LIB
	printf("Write and read /dev/spidev0.0 \r\n");
	DEV_GPIO_Init();
	DEV_HARDWARE_SPI_begin("/dev/spidev0.0");
#endif

#elif INTEL_EDISON
	printf("Configuring Intel Edison SPI\n");
	//mraa_init();

	//drowan_DEBUG_20200620:
	DEV_GPIO_Init();

	edisonSPI = mraa_spi_init(0);
	mraa_spi_mode(edisonSPI, MRAA_SPI_MODE0);
	mraa_spi_frequency(edisonSPI, 20e6);
	mraa_spi_lsbmode(edisonSPI, 0);
	mraa_spi_bit_per_word(edisonSPI, 8);

	/*
    uint8_t data;
    int recv_int;
    int i;

    while(1) {
		for(i = 0; i < 100; i++) {
			data = i;
			recv_int = mraa_spi_write(edisonSPI, data);
			//printf("Received: %d\n", recv_int);
			usleep(2000);
		}
    }
	/**/

#endif
    printf("/***********************************/ \r\n");
	return 0;
}

/******************************************************************************
function:	Module exits, closes SPI and BCM2835 library
parameter:
Info:
******************************************************************************/
void DEV_Module_Exit(void)
{
#ifdef RPI
#ifdef USE_BCM2835_LIB
	DEV_Digital_Write(EPD_CS_PIN, LOW);
	DEV_Digital_Write(EPD_DC_PIN, LOW);
	DEV_Digital_Write(EPD_RST_PIN, LOW);

	bcm2835_spi_end();
	bcm2835_close();
#elif USE_WIRINGPI_LIB
	DEV_Digital_Write(EPD_CS_PIN, 0);
	DEV_Digital_Write(EPD_DC_PIN, 0);
	DEV_Digital_Write(EPD_RST_PIN, 0);
#elif USE_DEV_LIB
	DEV_HARDWARE_SPI_end();
	DEV_Digital_Write(EPD_CS_PIN, 0);
	DEV_Digital_Write(EPD_DC_PIN, 0);
	DEV_Digital_Write(EPD_RST_PIN, 0);
#endif

#elif JETSON
#ifdef USE_DEV_LIB
	SYSFS_GPIO_Unexport(EPD_CS_PIN);
	SYSFS_GPIO_Unexport(EPD_DC_PIN);
	SYSFS_GPIO_Unexport(EPD_RST_PIN);
	SYSFS_GPIO_Unexport(EPD_BUSY_PIN);
#elif USE_HARDWARE_LIB
	Debug("not support");
#endif
#elif INTEL_EDISON
	printf("Closing Edison SPI...");
	mraa_spi_stop(edisonSPI);

	mraa_result_t status;
	mraa_gpio_context gpioEdison;

	//DataCommand GPIO
	gpioEdison = mraa_gpio_init(EPD_DC_PIN);

	if(gpioEdison == NULL) {
		fprintf(stderr, "Failed to initialize Edison GPIO%d\n", EPD_DC_PIN);
	}

	status = mraa_gpio_close(gpioEdison);

	if(status != MRAA_SUCCESS) {
		fprintf(stderr, "Failed to close Edison GPIO%d\n", EPD_DC_PIN);
	}

	//Reset GPIO
	gpioEdison = mraa_gpio_init(EPD_RST_PIN);

	if(gpioEdison == NULL) {
		fprintf(stderr, "Failed to initialize Edison GPIO%d\n", EPD_RST_PIN);
	}

	status = mraa_gpio_close(gpioEdison);

	if(status != MRAA_SUCCESS) {
		fprintf(stderr, "Failed to close Edison GPIO%d\n", EPD_RST_PIN);
	}

	//Busy GPIO
	gpioEdison = mraa_gpio_init(EPD_BUSY_PIN);

	if(gpioEdison == NULL) {
		fprintf(stderr, "Failed to initialize Edison GPIO%d\n", EPD_BUSY_PIN);
	}

	status = mraa_gpio_close(gpioEdison);

	if(status != MRAA_SUCCESS) {
		fprintf(stderr, "Failed to close Edison GPIO%d\n", EPD_BUSY_PIN);
	}

	mraa_deinit();

#endif
}
