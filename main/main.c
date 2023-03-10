// tjek om buadrate overhoved virker på LOGI.. 
// Måske skal CS trækkes lav, før DRDY kan skabe interrupt. i ADS_ready eller blot inden if(); fx
// Tjek hvordan DRDY og CS!
// hvis ikke det virker, skal 2ms timer måske sættes op til CS 
//eller kan CS være konstant lav, og DRDY dekterer 2ms interrupt
// Skitser Dataflow fra de 2 kerner

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_pm.h"
//#include "esp_intr_alloc.h" 
#include "driver/uart.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "esp_intr_alloc.h"
#include "esp32/rom/ets_sys.h"
#include "stdint.h"
#include <stdlib.h>
#include <string.h>

//TEST
// SPI pins
#define HSPI_MISO_PIN 12
#define HSPI_MOSI_PIN 13
#define HSPI_SCLK_PIN 14
#define HSPI_CS_PIN   15

// ADS pins 
#define ADS_INTERRUPT_GPIO 27
#define ADS_RESET_GPIO 26
#define POWERDOWN_GPIO 25
#define START_GPIO 33
#define GPIO_OUTPUT_PIN_SEL ((1ULL<<ADS_RESET_GPIO)|(1ULL<<POWERDOWN_GPIO)|\
                            (1ULL<<START_GPIO)|(1ULL<<HSPI_CS_PIN))
//20, 24, 28, 29, 30, 31, 37, 38 does not exit on ESP32 DevKit1!!
//#define GPIO_INPUT_PIN_SEL (1ULL<<ADS_INTERRUPT_GPIO)

// Falling egde -> GPIO_INTR_NEGEDGE // Rising edge -> GPIO_INTR_POSEDGE // any edge  -> GPIO_INTR_ANYEDGE
#define ANYEDGE GPIO_INTR_ANYEDGE
#define RISSING GPIO_INTR_POSEDGE
#define FALLING GPIO_INTR_NEGEDGE

#define ARRAY_SIZE1 9
#define ARRAY_SIZE2 3
#define ADSsize 27
#define GPIO_NUM_MAX 40

#define RunningAverageCount 10
#define toogle 15

// test int
volatile int ads_flag = 0;
volatile bool interrupt_enabled = true;

//##Global variables##
//uint8_t received_data[ADSsize];

TaskHandle_t state_machine_handle;
TaskHandle_t DSP_BLE_handle;
spi_device_handle_t hspi;
SemaphoreHandle_t xSemaphore = NULL;
// Create queues for passing data between the tasks
QueueHandle_t xqueue1 = NULL;
QueueHandle_t xqueue2 = NULL;

// SPI clock set to 2MHz
#define SPI_CLOCK     2000000

// Set SPI mode to MODE1 -> CPHA = 1 && CPOL = 0
#define SPI_MODE1 1

// Define the state machine states
typedef enum {
    ADS,
    senddata,
    state2
} statetype;

typedef enum {
    receive,
    process,
    BLE
} StateType;

// Declare HSPI functions
void hspi_init(void);
uint8_t hspi_transfer_byte(uint8_t data);
//uint8_t* hspi_transfer_bytes(uint8_t* tx_data, size_t n_bytes);
void hspi_transfer_bytes(uint8_t* tx_data, size_t n_bytes, uint8_t* rx_buffer);
void baseline(int32_t* raw_data, int32_t* new_data);
uint8_t lsb_to_msb(uint8_t lsb_byte);

// Declaration of interrupt, can be used for any service rutine!
void ads_isr_handler(void *arg);
void ads_attachInterrupt(void);
void ads_detachInterrupt(void);

// Declare some more functions!
void ADS_ready(void);
void ADS1298_init_conf(void);

// Define the state machine task
void state_machine(void *pvParameter)
{
    // get the task handle
    state_machine_handle = xTaskGetCurrentTaskHandle();
    uint8_t tx[ADSsize] = {0};
    uint8_t ADS_buffer[ADSsize] = {0};
    uint32_t Datapack[9];
    uint8_t data2[ARRAY_SIZE2];
    // Set Powerdown to logic high, wait 500ms
    gpio_set_level(POWERDOWN_GPIO, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Set ADSReset to logic high, wait 1000ms, then set to logic low
    gpio_set_level(ADS_RESET_GPIO, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(ADS_RESET_GPIO, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(START_GPIO, 0); 
    gpio_set_level(HSPI_CS_PIN, 0);  //pull SS low to prep for another transfer
    vTaskDelay(50 / portTICK_PERIOD_MS);

    // Initialize HSPI    
    hspi_init();

    // Attach interrupt on falling edge
    ads_attachInterrupt();

    // Initialize the state machine
    statetype state = ADS;

    while (1) {
        switch (state) {
            case ADS:            
                ADS_ready();

                if(ads_flag == 1){
                    //esp_task_wdt_reset();
                    // for (int i = 0; i < ADSsize; i++) {
                    //         ADS_buffer[i] = hspi_transfer_byte(0x00);
                    //         //hspi_transfer(0x00, 1, ADS_buffer[i], 1);
                    //         //ADS_buffer[i] = hspi_transfer(0x00, ADS_buffer[i], 1);
                    //     }
                    hspi_transfer_bytes(tx, ADSsize, ADS_buffer);
                    state = state2;
                    break;
                }
            
                
            state = senddata;
            break;


            case state2:
                //ESP_LOGI("STATE MACHINE", "In state 2");

                if(ads_flag == 1){
                    for (int j = 0; j < (9 * 3); j = j + 3) {
                        Datapack[j / 3] = ((ADS_buffer[j]) << 16 | (ADS_buffer[j + 1]) << 8 | (ADS_buffer[j + 2]));
                    }
                    for (int i = 0; i < ARRAY_SIZE2; i++) {
                    data2[i] = i;
                    }
                    state = senddata;
                    break;
                    
                }

            state = ADS;
            break;


            case senddata:
                if(ads_flag == 1){
                    
                    
                    // Send data1 to task2
                    xQueueSend(xqueue1, &Datapack, 0);
                    // Send data2 to task2
                    xQueueSend(xqueue2, &data2, 0);

                    // Give the semaphore to wake up the task called DSP_BLE on core 0
                    xSemaphoreGive(xSemaphore);

                    // suspend the task until the next interrupt
                    ads_flag = 0;
                    vTaskSuspend(NULL);
                }
                
            state = ADS;
            break;


        }
    }
}

void DSP_BLE(void *pvParameters)
{
    uint32_t ads_buffer[ARRAY_SIZE1];
    uint8_t array2[ARRAY_SIZE2];
    uint32_t Datapack[ARRAY_SIZE1];
    uint8_t array4[ARRAY_SIZE2];
    int32_t ECG_raw[ARRAY_SIZE1];
    StateType state = receive;

    while(1) {
        switch (state) {
            case receive:   
        
                // Wait for data from state machine
                xSemaphoreTake(xSemaphore, portMAX_DELAY);

                xQueueReceive(xqueue1, &ads_buffer, 0);
                xQueueReceive(xqueue2, &array2, 0);

                memcpy(Datapack, ads_buffer, ARRAY_SIZE1 * sizeof(uint32_t));
                memcpy(array4, array2, ARRAY_SIZE2 * sizeof(uint8_t));

            case process:
            
                gpio_set_level(HSPI_CS_PIN, 1);
                baseline((int32_t*)Datapack, ECG_raw);
                gpio_set_level(HSPI_CS_PIN, 0);
                

            state = BLE;
            break;

            case BLE:

                // for (int i = 0; i < ARRAY_SIZE2; i++) {
                // printf("%u ", array4[i]);
                // }
                // printf("\n");
                printf("%li\n", ECG_raw[5]);

            state = receive;
            break;
        


        }
        
    }
}

void app_main(void)

{   
 
    // Configure ADSReset, Powerdown, and Start as outputs    
    gpio_config_t io_con = {
    .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE
   }; 
    gpio_config(&io_con);

    // Set the CPU frequency to 240 MHz on both cores
    esp_pm_config_esp32_t pm_config = {
        .max_freq_mhz = 240,
        .min_freq_mhz = 80,
        .light_sleep_enable = true
    };
    esp_pm_configure(&pm_config);   
    
    // Create the semaphores for inter-core interrupt communication
    xSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xSemaphore);

    // Create two queues for passing data between the tasks
    xqueue1 = xQueueCreate(1, ARRAY_SIZE1 * sizeof(int));
    xqueue2 = xQueueCreate(1, ARRAY_SIZE2 * sizeof(int));


    // Create the state machine task and pin it to core 0
    xTaskCreatePinnedToCore(&state_machine, "state_machine", 4096, NULL, 1, &state_machine_handle, 1);
    xTaskCreatePinnedToCore(&DSP_BLE, "DSP_BLE", 4096, NULL, 1, &DSP_BLE_handle, 0);


}


// Initialize HSPI
void hspi_init(void) {
    esp_err_t ret;
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = HSPI_MOSI_PIN, 
        .miso_io_num = HSPI_MISO_PIN,
        .sclk_io_num = HSPI_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = SPI_CLOCK,
        .mode = SPI_MODE1,
        .input_delay_ns = 0,
        .spics_io_num = -1,
        .queue_size = 27,
        .flags = SPI_DEVICE_NO_DUMMY
    };

    ret = spi_bus_initialize(HSPI_HOST, &bus_cfg, 1);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(HSPI_HOST, &dev_cfg, &hspi);
    ESP_ERROR_CHECK(ret);
}

//function for recieve and transmit byte with HSPI
uint8_t hspi_transfer_byte(uint8_t data) {
    esp_err_t ret;
    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .cmd = 0,
        .addr = 0,
        .length = 8,
        .rxlength = 8,
        .tx_data[0] = data,
        .rx_data[0] = data
    };

    ret = spi_device_transmit(hspi, &trans);
    ESP_ERROR_CHECK(ret);

    return trans.rx_data[0];
    
    }
void hspi_transfer_bytes(uint8_t* tx_data, size_t n_bytes, uint8_t* rx_buffer) {
    esp_err_t ret;
    uint8_t* rx_data = (uint8_t*) malloc(n_bytes);

    for (int i = 0; i < n_bytes; i += 3) {
        spi_transaction_t trans;
        memset(&trans, 0, sizeof(trans));
        trans.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        trans.length = 24;
        trans.rxlength = 24;
        trans.tx_data[0] = tx_data[i];
        trans.tx_data[1] = tx_data[i+1];
        trans.tx_data[2] = tx_data[i+2];
        trans.rx_data[0] = 0;
        trans.rx_data[1] = 0;
        trans.rx_data[2] = 0;

        ret = spi_device_transmit(hspi, &trans);
        ESP_ERROR_CHECK(ret);

        rx_buffer[i] = trans.rx_data[0];
        rx_buffer[i+1] = trans.rx_data[1];
        rx_buffer[i+2] = trans.rx_data[2];
    }

}

int32_t RunningAverageBuffer[ARRAY_SIZE1][RunningAverageCount];
int NextRunningAverage[ARRAY_SIZE1] = {0};
int64_t AverageData[ARRAY_SIZE1] = {0};
int toogleAVG[ARRAY_SIZE1];
int numCalls = 0;

void baseline(int32_t* raw_data, int32_t* new_data) {
    numCalls++;

    for(int j = 0; j < ARRAY_SIZE1; j++){
        toogleAVG[j]++;
        if(toogleAVG[j] > toogle){
            RunningAverageBuffer[j][NextRunningAverage[j]++] = raw_data[j];
            if (NextRunningAverage[j] >= RunningAverageCount)
            {
                NextRunningAverage[j] = 0; 
            }

            int numSamples = numCalls < RunningAverageCount ? numCalls : RunningAverageCount;
            AverageData[j] = 0;
            for(int i=0; i< numSamples; ++i)
            {
                AverageData[j] += RunningAverageBuffer[j][i];
            }
            AverageData[j] /= numSamples;
            toogleAVG[j] = 0;
        }
    }

    for(int j = 1; j < ARRAY_SIZE1; j++){
        //new_data[j] = raw_data[j] - AverageData[j];
        if(AverageData[5] != 0){
        //printf("%li\n", AverageData[j]);
        
        new_data[j] = raw_data[j] - (int32_t)AverageData[j];
        }
    }

}

void ads_attachInterrupt(void) {

    gpio_config_t io_conf;
    // Interrupt on falling edge
    io_conf.intr_type = FALLING;
    // Bit mask of the pins that you want to set
    //io_conf.pin_bit_mask = (1ULL << ADS_INTERRUPT_GPIO); 
    io_conf.pin_bit_mask = 1 << ADS_INTERRUPT_GPIO; 
    // Set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // Enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    //esp_intr_alloc(ADS_INTERRUPT_GPIO, ESP_INTR_FLAG_IRAM, ads_isr_handler, NULL, NULL);


    //pio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add(ADS_INTERRUPT_GPIO, ads_isr_handler, NULL);
    // Enable the interrupt
    gpio_intr_enable(ADS_INTERRUPT_GPIO);
}

void ads_detachInterrupt(void) {
    gpio_isr_handler_remove(ADS_INTERRUPT_GPIO);
    gpio_intr_disable(ADS_INTERRUPT_GPIO);
    gpio_uninstall_isr_service();
}


//void IRAM_ATTR ads_isr_handler(void *arg) {
void IRAM_ATTR ads_isr_handler(void *arg) {
     vTaskSuspend(state_machine_handle);
    ads_flag = 1;
    vTaskResume(state_machine_handle);
}

// Definetion of the random functions
void ADS_ready(void)
{
    // flag for ADS1298_init_conf (run only once)
    static int flag_ADS1298 = 0;
    
    //ESP_LOGI("FUNCTION 1", "Running function 1");
    


    //complete setup (once only)
    if (flag_ADS1298 == 0) {
        //  slettes igen.. måske tjek om dette kun køres en gang.
        //ADS reset, enable and required delays
        vTaskDelay(1 / portTICK_PERIOD_MS);
        gpio_set_level(ADS_RESET_GPIO, 0);
        vTaskDelay(2 / portTICK_PERIOD_MS);
        gpio_set_level(ADS_RESET_GPIO, 1); 
        vTaskDelay(10 / portTICK_PERIOD_MS);
        flag_ADS1298 = 1;
        ADS1298_init_conf();
    }
}

void ADS1298_init_conf(void)
{   

    ESP_LOGI("ADS1298_init_conf", "Running configuration of ADS1298");
    //WAKEUP ADS1298 opcode
     hspi_transfer_byte(0x02);
        
     //SDATAC opcode (stop RDATAC mode)
     hspi_transfer_byte(0x11);
     vTaskDelay(2 / portTICK_PERIOD_MS);

     // Set device in HR (High resolution) Mode and DR (Data Rate) = Fmode/1024 500 SPS
     //WREG opcode:
     hspi_transfer_byte(0x41);  // 4=Write Registers, 1=Starting at register with address 1
     hspi_transfer_byte(0x02);  // Write 3 registers-1=2
     //Config1 0x86.
     hspi_transfer_byte(0x86);  //0x86=10000110 --> HR Mode and DR (Data Rate) = Fmode/1024 500 SPS
     //Config2 0x00
     hspi_transfer_byte(0x10);  //0x00= Reset Value --> Everything by default
     //Config3 0xDC RLDREF generated internally
     hspi_transfer_byte(0x10);  //0xC1=11000001--> Enable internal reference buffer &RLD connected
     //LOFF 0x03
    //SPI.transfer(0x03); //DC lead-off detection turned on

    //Set all channels to Input Shorts and Power Down
    //Except for Ch2 and Ch3 to gain 12
    hspi_transfer_byte(0x45);   //0x45= Write(4) Starting at register with address 5 (CH1)
    hspi_transfer_byte(0x07);    //Write 8 registers-1=7
    //CH1 address 05
    hspi_transfer_byte(0x40);
    //CH2 address 06
    hspi_transfer_byte(0x40);
    //CH3 address 07
    hspi_transfer_byte(0x40);
    //CH4 address 08
    hspi_transfer_byte(0x40);
    //CH5 address 09
    hspi_transfer_byte(0x40);
    //CH6 address 0A
    hspi_transfer_byte(0x40);
    //CH7 address 0B
    hspi_transfer_byte(0x40);
    //CH8 address 0C
    hspi_transfer_byte(0x40);

    //  Wilsons central terminalsetup:
    hspi_transfer_byte(0x58); //0x58=01011000 (WriteReg) Starting at register with address 18 (WCT1)
    hspi_transfer_byte(0x01); //Write 3 registers-1=2

    // config 4
    //hspi->transfer(0x04);

    // WCT1
    hspi_transfer_byte(0x0A); // WCTA Powered up, Channel2 positive input connected to WCTA amplifier

    // WCT2
    hspi_transfer_byte(0xE3);// WCTA Powered up, Channel3 positive input connected to WCTB amplifier, Channel2 negative input connected to WCTC amplifier

    //START Opcode. Start/sincronize conversion
    hspi_transfer_byte(0x08);

    vTaskDelay(1 / portTICK_PERIOD_MS);

    //Opcode RDATAC. Read Data Continous from now on.
    hspi_transfer_byte(0x10);

    
}
