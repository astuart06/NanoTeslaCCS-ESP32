
#include <stdio.h>
#include <stddef.h>
#include <string.h>

#include "sdkconfig.h"

#include "serial.h"
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "freertos/timers.h"
#include "freertos/queue.h"

#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "soc/gpio_struct.h"
#include "soc/timer_group_struct.h"
#include "soc/io_mux_reg.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp32/rom/ets_sys.h"

#define LOG_LOCAL_LEVEL ESP_LOG_WARN
#include "esp_log.h"

#include "globals.h"

#define PI 3.14159
/*
// PHASED SPEAKER ARRAY PINOUT
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 18
#define PIN_NUM_CLK  23
#define PIN_NUM_CS   5
*/
// NANO TESLA PINOUT
#define PIN_NUM_MISO 33
#define PIN_NUM_MOSI 18
#define PIN_NUM_CLK  17
#define PIN_NUM_CS   5

#define PIN_NUM_LDAC 16
#define PIN_NUM_RESET 19

#define GPIO_TIMER_EN	GPIO_NUM_27
#define GPIO_DEBUG_LED	GPIO_NUM_12
#define GPIO_DEBUG_PIN	GPIO_NUM_26
#define GPIO_TRANS_EN	GPIO_NUM_32


#define DAC_A				0x11				// Array indexes for each of the DAC channels
#define DAC_B				0x12				// These also set the command and address byte in the DACs
#define DAC_C				0x14
#define DAC_D				0x18
#define DAC_UPDATE_PERIOD	0x21				// Time period between the DAC data transmission.
#define DAC_NUM_ICS			0x22				// Number of DACs in daisy-chain.
#define TIMER_STATE			0x23

#define DAC_1	0								// Array indexes for each of the individual DAC ICs
#define DAC_2	1
#define DAC_3	2
#define DAC_4	3

#define BUFFER_SIZE_MAX 		48					// Number of data points in sine wave array
#define SERIAL_BUFFER_SIZE_MAX 	(BUFFER_SIZE_MAX * 2) 	// 2 bytes per data point.
#define DAC_ICS_MAX 			6					// Initial testings with 3 DAC ICs for 12 speakers on outer ring.

#define TIMER_DIVIDER 80						// 1us per timer tick
#define TIMER_PERIOD_US 100	 					// With 1us per timer tick

// GLOBAL VARIABLES
spi_device_handle_t spi;

// These 4 buffers are pointed to by the transaction structs for the tx_data. Currently they are fixed at a large size.
uint8_t sineBufferCharA[BUFFER_SIZE_MAX * 3 * DAC_ICS_MAX];
uint8_t sineBufferCharB[BUFFER_SIZE_MAX * 3 * DAC_ICS_MAX];
uint8_t sineBufferCharC[BUFFER_SIZE_MAX * 3 * DAC_ICS_MAX];
uint8_t sineBufferCharD[BUFFER_SIZE_MAX * 3 * DAC_ICS_MAX];
static spi_transaction_t transA[BUFFER_SIZE_MAX];
static spi_transaction_t transB[BUFFER_SIZE_MAX];
static spi_transaction_t transC[BUFFER_SIZE_MAX];
static spi_transaction_t transD[BUFFER_SIZE_MAX];
esp_err_t ret;

int dacDataPoints;
int dacNumICs;

// Terminal logging tag
static const char* TAG = "main";

typedef struct{
	uint8_t size;
	uint8_t type;
	uint8_t data[SERIAL_BUFFER_SIZE_MAX * DAC_ICS_MAX];
} serialRx_t;

int bufferIndex;
int spiSendNow;
int dataReady;

QueueHandle_t xQueue;
TaskHandle_t xHandlingTask = NULL;
TaskHandle_t xSerialTask = NULL;

// Function Prototypes
void TimerInit(void);
void SpiInit(void);
void DacInit(void);
void GpioInit(void);

void UpdateTimerPeriod(serialRx_t *ptr_rxBuffer);
void UpdateDacNumICs(serialRx_t *ptr_rxBuffer);
void UpdateTimerState(serialRx_t *ptr_rxBuffer);

void PopulateTransData(spi_transaction_t *ptr_trans, uint8_t *ptr_buffer, serialRx_t *ptr_rxBuffer);
void PrintfChannelBuffer(uint8_t *ptr_buffer);

static void SerialTask(void *pvParameters);
static void ParseSerialDataTask(void *pvParameters);
static void SpiTask(void *pvParameters);

/***************************************************************************************
 *  Function: PingSPI_isr
 *
 *	This ISR is called by the timer interrupt firing. It notfies the SpiTask that it
 *	should send the next data transmission.
 *
 *  Returns:
 *  - Void
 *  Parameters:
 *  - *pvParameters
 *****************************************************************************************/
void IRAM_ATTR PingSPI_isr()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint32_t ulStatusRegister;

	ulStatusRegister = 1;

    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[0].config.alarm_en = 1;

    // Take LDAC low to update DAC outputs
    gpio_set_level(PIN_NUM_LDAC, 0);
    gpio_set_level(PIN_NUM_LDAC, 1);

	spiSendNow = 1;

    xTaskNotifyFromISR( xHandlingTask, ulStatusRegister, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR();
}


/***************************************************************************************
 *  Function: SerialTask
 *
 *
 *
 *  Returns:
 *  Parameters:
 *  - *pvParameters
 *****************************************************************************************/
static void SerialTask(void *pvParameters)
{
	int len;
	BaseType_t xStatus;
    // Configure a temporary buffer for the incoming data
    uint8_t *serialData = (uint8_t *) malloc((SERIAL_BUFFER_SIZE_MAX * DAC_ICS_MAX) + 3);

	while(1)
	{
		len = uart_read_bytes(UART_NUM_2, serialData, ((SERIAL_BUFFER_SIZE_MAX * DAC_ICS_MAX) + 3), 100 / portTICK_PERIOD_MS);
		if(len > 0){
			xStatus = xQueueSendToBack(xQueue, serialData, 0);
			if( xStatus != pdPASS ){
				printf( "Could not send to the queue.\n" );
			}
		}
		// Is this needed? As long as Parse has higher priority could remove??
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
}
/***************************************************************************************
 *  Function: ParseSerialDataTask
 *
 *
 *
 *  Returns:
 *  - Void
 *  Parameters:
 *  - *pvParameters
 *****************************************************************************************/
static void ParseSerialDataTask(void *pvParameters)
{
	BaseType_t xStatus;
	serialRx_t packet;

	while(1){
		xStatus = xQueueReceive(xQueue, &packet, 0);
		if( xStatus == pdPASS ){
			ESP_LOGI(TAG, "Parsing new data...");
			ESP_LOGI(TAG, "Size: %d", packet.size);
			ESP_LOGI(TAG, "Type: %x", packet.type);
			ESP_LOGI(TAG, "Delim: '%c'", packet.data[packet.size - 3]);

			switch(packet.type){
				case DAC_A:
					printf("Updating channel A\n");
					PopulateTransData(transA, &sineBufferCharA[0], &packet);
					PrintfChannelBuffer(&sineBufferCharA[0]);
					break;
				case DAC_B:
					printf("Updating channel B\n");
					PopulateTransData(transB, &sineBufferCharB[0], &packet);
					PrintfChannelBuffer(&sineBufferCharB[0]);
					break;
				case DAC_C:
					printf("Updating channel C\n");
					PopulateTransData(transC, &sineBufferCharC[0], &packet);
					PrintfChannelBuffer(&sineBufferCharC[0]);
					break;
				case DAC_D:
					printf("Updating channel D\n");
					PopulateTransData(transD, &sineBufferCharD[0], &packet);
					PrintfChannelBuffer(&sineBufferCharD[0]);
					break;
				case DAC_UPDATE_PERIOD:
					UpdateTimerPeriod(&packet);
					break;
				case DAC_NUM_ICS:
					UpdateDacNumICs(&packet);
					DacInit();
					break;
				case TIMER_STATE:
					UpdateTimerState(&packet);
					break;
			}
			dataReady = 1;
	    }
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

/***************************************************************************************
 *  Function: UpdateTimerState
 *
 *
 *
 *  Returns:
 *  - Void
 *  Parameters:
 *  -
 *****************************************************************************************/
void UpdateTimerState(serialRx_t *ptr_rxBuffer)
{
	if(ptr_rxBuffer->data[0] == 1){
		timer_start(TIMER_GROUP_0, TIMER_0);
		gpio_set_level(GPIO_DEBUG_LED, 1);
		printf("Timer interrupts ON\n");
	}
	else if(ptr_rxBuffer->data[0] == 0){
			timer_pause(TIMER_GROUP_0, TIMER_0);
			gpio_set_level(GPIO_DEBUG_LED, 0);
			printf("Timer interrupts OFF\n");
	}
	else{
		printf("Invalid Timer state data\n");
	}
}

/***************************************************************************************
 *  Function: UpdateTimerPeriod
 *
 *
 *
 *  Returns:
 *  - Void
 *  Parameters:
 *  -
 *****************************************************************************************/
void UpdateTimerPeriod(serialRx_t *ptr_rxBuffer)
{
	unsigned int msb, lsb, value;

	msb = ptr_rxBuffer->data[0];
	lsb = ptr_rxBuffer->data[1];
	value = (msb << 8) | lsb;

	timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, value);
	printf("Timer period (us): %d\n", value);
}

/***************************************************************************************
 *  Function: UpdateDacNumICs
 *
 *	Retieves the data from the rx packet and stores it in the global dacNumICs variable.
 *	The DacInit() function will actual use the SPI bus to update the daisy-chain
 *	register within each DAC.
 *
 *  Returns:
 *  - Void
 *  Parameters:
 *  - *ptr_rxBuffer		Ptr to serial rx data struct ready for formatting.
 *****************************************************************************************/
void UpdateDacNumICs(serialRx_t *ptr_rxBuffer)
{
	dacNumICs = ptr_rxBuffer->data[0];
	ESP_LOGI(TAG, "Number of DAC ICs: %d", dacNumICs);
}
/***************************************************************************************
 *  Function: PopulateTransData
 *
 *	For each SPI transmission we need a buffer with the data for 1 channel of each DAC.
 *	The data is formatted to 3 bytes per DAC (Cmd/Addr, MSB, LSB). ptr_trans allows many
 *	transaction structs to facilitate the setup of AC waveforms.
 *	Data is sent little-endian, so DAC 1 should occupy the highest bytes (thus tx'ed last).
 *
 *  Returns:
 *  - Void
 *  Parameters:
 *  - *ptr_trans 		Ptr to transaction struct array.
 *  - *ptr_buffer		Ptr to data buffer for each transaction.
 *  - *ptr_rxBuffer		Ptr to serial rx data struct ready for formatting.
 *****************************************************************************************/
void PopulateTransData(spi_transaction_t *ptr_trans, uint8_t *ptr_buffer, serialRx_t *ptr_rxBuffer)
{
	int i, j;
	int offset;

	// Determine the number of data points by taking the number of bytes in the data field
	// and dividing by the number of DACs in circuit.
	dacDataPoints = ((ptr_rxBuffer->size) - 3) / (2 * dacNumICs);
	ESP_LOGI(TAG, "PopulateTransData Function");
	ESP_LOGI(TAG, "Buffer size: %d", ptr_rxBuffer->size);
	ESP_LOGI(TAG, "dacDataPoints: %d", dacDataPoints);
	ESP_LOGI(TAG, "dacNumICs: %d", dacNumICs);

	for(i = 0; i < dacDataPoints; i++){
		// For each data point setup a transaction structure
		ptr_trans->flags = 0;
		ptr_trans->length = 8 * 3 * dacNumICs;		// bit per byte * bytes per DAC * Number of DACs
		ptr_trans->tx_buffer = ptr_buffer;			// Point to start of 'sineBufferCharX' before it is
		ptr_trans->addr = 0;						// incremented below.
		ptr_trans->cmd = 0;
		ptr_trans->rxlength = 0;

		ptr_trans++;								// Point to the next transmit structure.

		offset = i * 2;
	    for(j = 0; j < dacNumICs; j++){
			*ptr_buffer++ = ptr_rxBuffer->type;				// Command byte
			*ptr_buffer++ = ptr_rxBuffer->data[offset];	    // MSB
			*ptr_buffer++ = ptr_rxBuffer->data[offset + 1];	// LSB
			offset += dacDataPoints * 2;
	    }
	}
}
/***************************************************************************************
 *  Function: SpiTask
 *
 *	Sends the transX buffer data over the SPI. This function is normally called after a
 *	timer interrupt. The transX buffers are 2D arrays, each call of this function
 *	transmits 1 row, as indicated by the bufferIndex.
 *
 *  Returns:
 *  - Void
 *  Parameters:
 *  - *pvParameters
 *****************************************************************************************/
void SpiTask(void *pvParameters)
{
	uint32_t ulInterruptStatus;

    while(1){
    	xTaskNotifyWait( 0x00, ULONG_MAX, &ulInterruptStatus, portMAX_DELAY );

		if((spiSendNow == 1) && (dataReady == 1)){
			gpio_set_level(GPIO_NUM_15, 1);

			spi_device_polling_transmit(spi, &transA[bufferIndex]);
			spi_device_polling_transmit(spi, &transB[bufferIndex]);
			spi_device_polling_transmit(spi, &transC[bufferIndex]);
			spi_device_polling_transmit(spi, &transD[bufferIndex]);

			spiSendNow = 0;
			bufferIndex++;
			if(bufferIndex >= dacDataPoints)	bufferIndex = 0;
			gpio_set_level(GPIO_NUM_15, 0);
		}
    }
}

/***************************************************************************************
 *  Function: PrintfChannelBuffer
 *
 *	In table form show all the data in 16-bit hex format for the specified channel.
 *	The columns indicate which DAC number and the rows indicate the sample number.
 *
 *  Returns:
 *  - Void
 *  Parameters:
 *  - *ptr_buffer
 *****************************************************************************************/
void PrintfChannelBuffer(uint8_t *ptr_buffer)
{
	int i, j;

	printf("T");
	for(i = 1; i <= dacNumICs; i ++){
		printf("\tDAC %d\t", i);
	}
	printf("\n");

	for(i = 0; i < dacDataPoints; i++){
		printf("[%02d]\t", i);

		for(j = 0; j < dacNumICs; j++){
			printf("0x%02X ", *ptr_buffer++);
			printf("0x%02X",  *ptr_buffer++);
			printf("%02X\t",  *ptr_buffer++);
		}
		printf("\n");
	}
}

/***************************************************************************************
 *  Function: TimerInit
 *
 *	Configure, but do not start the timer. Se the period with a default value.
 *
 *  Returns:
 *  - Void
 *  Parameters:
 *  - Void
 *****************************************************************************************/
void TimerInit()
{
	timer_config_t tmrcfg;

	tmrcfg.divider = TIMER_DIVIDER;
	tmrcfg.counter_dir = TIMER_COUNT_UP;
	tmrcfg.counter_en = TIMER_PAUSE;
	tmrcfg.alarm_en = TIMER_ALARM_EN;
	tmrcfg.intr_type = TIMER_INTR_LEVEL;
	tmrcfg.auto_reload = TIMER_AUTORELOAD_EN;

	timer_init(TIMER_GROUP_0, TIMER_0, &tmrcfg);
	timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

	timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_PERIOD_US);
	timer_enable_intr(TIMER_GROUP_0, TIMER_0);
	timer_isr_register(TIMER_GROUP_0, TIMER_0, PingSPI_isr, NULL, 0, NULL);

	timer_pause(TIMER_GROUP_0, TIMER_0);
}

/***************************************************************************************
 *  Function: SpiInit
 *
 *	Configure the SPI bus. e.g. pins, freq, spi mode.
 *
 *  Returns:
 *  - Void
 *  Parameters:
 *  - Void
 *****************************************************************************************/
void SpiInit()
{
	// Configure SPI pins and frequency, mode etc...
	spi_bus_config_t buscfg={
		.miso_io_num=PIN_NUM_MISO,
		.mosi_io_num=PIN_NUM_MOSI,
		.sclk_io_num=PIN_NUM_CLK,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1
	};

	spi_device_interface_config_t devcfg={
		.clock_speed_hz=1000*1000*20,           //Clock out at 10 MHz
		.mode=1,                                //SPI mode 1
		.spics_io_num=PIN_NUM_CS,               //CS pin
		.queue_size=7,                          //We want to be able to queue 7 transactions at a time
		.cs_ena_pretrans = 4,
		.cs_ena_posttrans = 4,
		.flags = SPI_DEVICE_HALFDUPLEX,
	};

	//Initialize the SPI bus
	ret=spi_bus_initialize(HSPI_HOST, &buscfg, 0);
	assert(ret==ESP_OK);
	//Attach the DAC to the SPI bus
	ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
	assert(ret==ESP_OK);

	// Acquire the bus for faster transactions since there is only on SPI device.
	ret = spi_device_acquire_bus(spi, portMAX_DELAY);
}

/***************************************************************************************
 *  Function: DACInit
 *
 *  To enable daisy-chain mode, set bit-0 to 1 in the daisy-chain register (0x80).
 *  Write 0x800001 over SPI to do this. Since the previous DACs daisy-chain mode must be
 *  enabled first before it can pass information onto the next DAC, the command must be
 *  repeated according to the number of DACs in the chain.
 *
 *  Returns:
 *  - Void
 *  Parameters:
 *  - Void
 *****************************************************************************************/
void DacInit()
{
	int i, j;
	static spi_transaction_t transSetup;
	uint8_t setupBuffer[3 * 6];					// 3 bytes for each of 6 eurocards

	// Repeat 3 bytes for each DAC we need to enable daisy-chain mode
	for(i = 0; i < dacNumICs; i++){
		j = 3*i;
		setupBuffer[j] = 0x80;					// Set the DCEN bit to enter register (MSB)
		setupBuffer[j + 1] = 0x00;				// (LSB)
		setupBuffer[j + 2] = 0x01;				// Set DB0 to 1 to enable daisy-chain mode

		transSetup.length = 24 * (i + 1);		// length in bits to transmit
		transSetup.tx_buffer = &setupBuffer;
		transSetup.flags = 0;
		transSetup.cmd = 0;
		transSetup.addr = 0;

		//spi_device_polling_end(spi, 100);
		spi_device_polling_transmit(spi, &transSetup);
	}
	printf("Number of DACs in daisy-chain: %d\n", dacNumICs);
}
/***************************************************************************************
 *  Function: GpioInit
 *
 *  Setup the digital input/output pins.
 *
 *  Returns:
 *  - Void
 *  Parameters:
 *  - Void
 *****************************************************************************************/
void GpioInit(void)
{
	// Setup LDAC and RESET pins - these outputs are not controlled by the
	// SPI library functions.
	PIN_FUNC_SELECT(IO_MUX_GPIO16_REG, PIN_FUNC_GPIO);
	gpio_set_direction(PIN_NUM_LDAC, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_NUM_LDAC, 1);

	PIN_FUNC_SELECT(IO_MUX_GPIO19_REG, PIN_FUNC_GPIO);
	gpio_set_direction(PIN_NUM_RESET, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_NUM_RESET, 1);

	// Setup Debug red LED GPIO pin
	PIN_FUNC_SELECT(IO_MUX_GPIO12_REG, PIN_FUNC_GPIO);
	gpio_set_direction(GPIO_DEBUG_LED, GPIO_MODE_OUTPUT);

	// Setup Debug output GPIO pin
	PIN_FUNC_SELECT(IO_MUX_GPIO26_REG, PIN_FUNC_GPIO);
	gpio_set_direction(GPIO_DEBUG_PIN, GPIO_MODE_OUTPUT);

	// Setup logic translator enable pin
	PIN_FUNC_SELECT(IO_MUX_GPIO32_REG, PIN_FUNC_GPIO);
	gpio_set_direction(GPIO_TRANS_EN, GPIO_MODE_OUTPUT);
	// And permanently enable the translators
	gpio_set_level(GPIO_TRANS_EN, 1);

	// Timer enable switch input
	PIN_FUNC_SELECT(IO_MUX_GPIO27_REG, PIN_FUNC_GPIO);
	gpio_set_direction(GPIO_TIMER_EN, GPIO_MODE_INPUT);
	gpio_set_pull_mode(GPIO_TIMER_EN, GPIO_PULLUP_ONLY);

}
/***************************************************************************************
 *  Function: PrintEsp32Info
 *
 *	Print out ESP32 information.
 *
 *  Returns:
 *  - Void
 *  Parameters:
 *  - Void
 *****************************************************************************************/
void PrintEsp32Info()
{
	printf("----------------------\n");
	printf("NanoTeslaCCS - ESP32\n");
    printf("Version: 8I\n");
    printf("----------------------\n");
    printf("FreeRTOS tick rate: %dHz\n", configTICK_RATE_HZ );
    printf("Interrupt period (default): %dus\n", TIMER_PERIOD_US);

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
}

/***************************************************************************************
 *  Function: app_main

 *
 *  Returns:
 *  - Void
 *  Parameters:
 *  - Void
 *****************************************************************************************/
void app_main()
{
	esp_log_level_set(TAG, ESP_LOG_DEBUG);

	bufferIndex = 0;
	spiSendNow = 0;
	dataReady = 0;

	dacNumICs = 6;

	GpioInit();
	SpiInit();
	SerialInit();
	DacInit();
	TimerInit();

    vTaskDelay(100);
    // Take LDAC high, it will be toggled low during interrupt to update DAC outputs.
    gpio_set_level(PIN_NUM_LDAC, 1);

    // Create Queue
    xQueue = xQueueCreate(5, (((SERIAL_BUFFER_SIZE_MAX * DAC_ICS_MAX) + 3) * sizeof(int)));

    if(xQueue != NULL){
    	printf("Init Free heap size: %d\n", xPortGetFreeHeapSize());
    	// Create Tasks
    	xTaskCreate(SpiTask, "Spi_task", 50 * 1024, NULL, 6, &xHandlingTask);
		xTaskCreate(SerialTask, "uart_2_task", 50 * 1024, NULL, 5, NULL);
		xTaskCreate(ParseSerialDataTask, "parse_task", 50 * 1024, NULL, 4, NULL);

		printf("All task created, remaining heap size: %d\n", xPortGetFreeHeapSize());
		printf("Waiting...\n");
		vTaskDelay(100);
    }
    else{
    	printf("Failed to create xQueue");
    }
    // Loop here waiting for tasks...
    while(1){
    	vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
