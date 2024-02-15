#include "Arduino.h"
#include "freertos/FreeRTOS.h"

#define DEF_BAUD 115200

// #define USE_ARDUINO
#define USE_RTOS

#ifdef USE_RTOS

#define DMA_TASK_ID "DMA_TASK"
#define DMA_TASK_PRIO 1
#define DMA_TASK_STACK_DEPTH 10000

#define PRINT_TASK_ID "TEST_TASK"
#define PRINT_TASK_PRIO 1
#define PRINT_TASK_STACK_DEPTH 10000

#define CORE_0 0
#define CORE_1 1

#endif

/* The code is defining an array `channel` of type `adc1_channel_t` with 4 elements. Each element
represents a specific ADC channel on the ESP32 microcontroller. The comments next to each element
indicate the corresponding GPIO pin number for that channel. */
adc1_channel_t channel[4] = {
    ADC1_CHANNEL_0, // GPIO36
    ADC1_CHANNEL_3, // GPIO39
    ADC1_CHANNEL_6, // GPIO34
    ADC1_CHANNEL_7, // GPIO35
};

/**
 * The function `dma_init` initializes DMA for continuous ADC conversion and handles any exceptions
 * that occur during the initialization process.
 */
void dma_init()
{
    try
    {
        memset(result, SRAM_ADDR_1, TIMES);
        continuous_adc_init(channel, sizeof(channel) / sizeof(adc1_channel_t));
        adc_digi_start();
    }
    catch (const std::exception &e)
    {
        Serial.println("Exception during DMA INIT");
        Serial.println(e.what());
    }
}

#ifdef USE_RTOS

TaskHandle_t DMA_TASK_handle;
TaskHandle_t PRINT_BUFFER_TASK_handle;

TickType_t xDelay1000 = 1000 / portTICK_PERIOD_MS;
TickType_t xDelay500 = 500 / portTICK_PERIOD_MS;
TickType_t xDelay250 = 250 / portTICK_PERIOD_MS;
TickType_t xDelay125 = 125 / portTICK_PERIOD_MS;

/**
 * The function `dma_read_task` continuously performs an ADC DMA read and prints a message to the
 * serial monitor, with exception handling.
 *
 * @param pvParameters The pvParameters parameter is a void pointer that can be used to pass any
 * additional parameters to the task. In this case, it is not being used and is set to NULL.
 */
void dma_read_task(void *pvParameters)
{
    try
    {
        while (true)
        {
            Serial.println("DMA TASK IS RUNNING!");
            perform_adc_dma_read();
            vTaskDelay(xDelay500);
            taskYIELD();
        }
    }
    catch (const std::exception &e)
    {
        Serial.println("Exception DMA_READ_TASK");
        Serial.println(e.what());
    }
}

/**
 * The function `print_buffer_task` continuously prints the contents of a buffer to the serial monitor.
 *
 * @param pvParamaters The parameter `pvParamaters` is a void pointer, which means it can be used to
 * pass any type of data to the task. In this case, it is not being used in the task function, so it
 * can be ignored.
 */
void print_buffer_task(void *pvParamaters)
{
    try
    {
        while (true)
        {
            Serial.println("TASK 2 IS RUNNING!");
            print_buffer(adc_samples_CH0_ptr, ADC_SAMPLE_BUFFER_SIZE, "channel 0");
            vTaskDelay(xDelay1000);
            taskYIELD();
        }
    }
    catch (const std::exception &e)
    {
        Serial.println("Exception PRINT_BUFFER_TASK");
        Serial.println(e.what());
    }
}

/**
 * The setup function initializes the Arduino, starts the serial communication, initializes the DMA,
 * creates tasks for reading DMA data and printing the buffer, and assigns the tasks to core 0.
 */
void setup(void)
{
    initArduino();
    Serial.begin(DEF_BAUD);
    dma_init();

    xTaskCreatePinnedToCore(
        &dma_read_task,
        DMA_TASK_ID,
        DMA_TASK_STACK_DEPTH,
        NULL,
        DMA_TASK_PRIO,
        &DMA_TASK_handle,
        CORE_0);

    xTaskCreatePinnedToCore(
        &print_buffer_task,
        PRINT_TASK_ID,
        PRINT_TASK_STACK_DEPTH,
        NULL,
        PRINT_TASK_PRIO,
        &PRINT_BUFFER_TASK_handle,
        CORE_0);
}

void loop(void){};

#endif

#ifdef USE_ARDUINO

void setup(void)
{
    initArduino();
    Serial.begin(DEF_BAUD);
    dma_init();
}

void loop(void)
{
    perform_adc_dma_read();
    print_buffer(adc_samples_CH0_ptr, ADC_SAMPLE_BUFFER_SIZE, "channel: 0");
    Serial.println("LOOP");
    delay(150);
}

#endif