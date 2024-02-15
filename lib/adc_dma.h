/*
 * adc_dma.h
 *
 * Created on 2022. júl 12.
 * Author: Zoltán Berkula
 */

#ifndef ADC_DMA_H
#define ADC_DMA_H

#ifdef __cplusplus
extern "C"
{
#endif

	extern "C"
	{
#include "adc_dma_bridge.h"
#include <esp_log.h>
	}

#define TIMES 128				  // 128 bytes for each channel
#define DMA_BUFFER_SIZE 128		  // size of the DMA's buffer
#define ADC_SAMPLE_BUFFER_SIZE 64 // size of each adc sample buffer
#define SRAM_ADDR_1 0xcc		  // DMA write section starting address
#define CHANNEL_0_ID 0			  // adc unit 1 channel ID 0 -> GPIO36
#define CHANNEL_1_ID 1			  // adc unit 1 channel ID 1 -> GPIOXX
#define CHANNEL_2_ID 2			  // adc unit 1 channel ID 2 -> GPIOXX
#define CHANNEL_3_ID 3			  // adc unit 1 channel ID 3 -> GPIO39
#define CHANNEL_4_ID 4			  // adc unit 1 channel ID 4 -> GPIO32
#define CHANNEL_5_ID 5			  // adc unit 1 channel ID 5 -> GPIO33
#define CHANNEL_6_ID 6			  // adc unit 1 channel ID 6 -> GPIO34
#define CHANNEL_7_ID 7			  // adc unit 1 channel ID 7 -> GPIO35

#define GET_UNIT(x) ((x >> 3) & 0x1)
#define ADC_RESULT_BYTE 2
#define ADC_CONV_LIMIT_EN 1
#define ADC_CONV_LIMIT_NUM 250
#define ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define ADC_SAMPLE_FREQ 10 * 1000
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1

	static uint16_t adc1_chan_mask = BIT(7);
	static uint16_t adc2_chan_mask = 0;

	esp_err_t ret;
	uint32_t ret_num = 0;

	uint8_t result[TIMES];		  // an array that holds the measured values
	uint8_t *result_ptr = result; // ptr to result array

	uint16_t adc_samples_CH0[ADC_SAMPLE_BUFFER_SIZE]; // buffer for channel 0
	uint16_t adc_samples_CH1[ADC_SAMPLE_BUFFER_SIZE]; // buffer for channel 1
	uint16_t adc_samples_CH2[ADC_SAMPLE_BUFFER_SIZE]; // buffer for channel 2
	uint16_t adc_samples_CH3[ADC_SAMPLE_BUFFER_SIZE]; // buffer for channel 3
	uint16_t adc_samples_CH4[ADC_SAMPLE_BUFFER_SIZE]; // buffer for channel 4
	uint16_t adc_samples_CH5[ADC_SAMPLE_BUFFER_SIZE]; // buffer for channel 5
	uint16_t adc_samples_CH6[ADC_SAMPLE_BUFFER_SIZE]; // buffer for channel 6
	uint16_t adc_samples_CH7[ADC_SAMPLE_BUFFER_SIZE]; // buffer for channel 7

	uint16_t *adc_samples_CH0_ptr = adc_samples_CH0; // ptr to buffer for channel 0
	uint16_t *adc_samples_CH1_ptr = adc_samples_CH1; // ptr to buffer for channel 1
	uint16_t *adc_samples_CH2_ptr = adc_samples_CH2; // ptr to buffer for channel 2
	uint16_t *adc_samples_CH3_ptr = adc_samples_CH3; // ptr to buffer for channel 3
	uint16_t *adc_samples_CH4_ptr = adc_samples_CH4; // ptr to buffer for channel 4
	uint16_t *adc_samples_CH5_ptr = adc_samples_CH5; // ptr to buffer for channel 5
	uint16_t *adc_samples_CH6_ptr = adc_samples_CH6; // ptr to buffer for channel 6
	uint16_t *adc_samples_CH7_ptr = adc_samples_CH7; // ptr to buffer for channel 7

	static uint8_t CH0_buff_index = 0; // adc sample ch0 buffer index
	static uint8_t CH1_buff_index = 0; // adc sample ch1 buffer index
	static uint8_t CH2_buff_index = 0; // adc sample ch2 buffer index
	static uint8_t CH3_buff_index = 0; // adc sample ch3 buffer index
	static uint8_t CH4_buff_index = 0; // adc sample ch4 buffer index
	static uint8_t CH5_buff_index = 0; // adc sample ch5 buffer index
	static uint8_t CH6_buff_index = 0; // adc sample ch6 buffer index
	static uint8_t CH7_buff_index = 0; // adc sample ch7 buffer index

	const char *ADC_DMA_TAG = "ADC_DMA_TAG";

	adc_digi_init_config_s adc_dma_config = {
		.max_store_buf_size = DMA_BUFFER_SIZE,
		.conv_num_each_intr = TIMES,
		.adc1_chan_mask = adc1_chan_mask,
		.adc2_chan_mask = adc2_chan_mask,
	};

	adc_digi_configuration_t dig_cfg = {
		.conv_limit_en = ADC_CONV_LIMIT_EN,
		.conv_limit_num = ADC_CONV_LIMIT_NUM,
		.sample_freq_hz = ADC_SAMPLE_FREQ,
		.conv_mode = ADC_CONV_MODE,
		.format = ADC_OUTPUT_TYPE,
	};

	// The `print_buffer` function is a helper function that prints the contents of an array. It takes three parameters: `buff`, which is a pointer to the array, `size`, which is the size of the array, and `channel_id`, which is a string identifier for the channel.
	static void print_buffer(uint16_t *buff, int size, const char *channel_id)
	{
		for (size_t i = 0; i < size; i++)
		{
			printf("iteration: %d  %s val: %d\n", i, channel_id, buff[i]);
		}
		return;
	}

	static void continuous_adc_init(adc1_channel_t *channel, uint8_t channel_num)
	{
		ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));
		adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
		dig_cfg.pattern_num = channel_num;
		for (int i = 0; i < channel_num; i++)
		{
			uint8_t unit = GET_UNIT(channel[i]);
			uint8_t ch = channel[i] & 0x7;
			adc_pattern[i].atten = ADC_ATTEN_DB_11;
			adc_pattern[i].channel = ch;
			adc_pattern[i].unit = unit;
			adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
		}
		dig_cfg.adc_pattern = adc_pattern;
		ESP_ERROR_CHECK(adc_digi_controller_configure(&dig_cfg));
	}

	void channel_sampler(uint16_t ch, uint16_t data) // TODO REFACTOR
	{
		if (ch == CHANNEL_0_ID)
		{
			adc_samples_CH0[CH0_buff_index] = data;
			if (++CH0_buff_index >= ADC_SAMPLE_BUFFER_SIZE)
			{
				CH0_buff_index = 0;
			}
		}
		else if (ch == CHANNEL_1_ID)
		{
			adc_samples_CH1[CH1_buff_index] = data;
			if (++CH1_buff_index >= ADC_SAMPLE_BUFFER_SIZE)
			{
				CH1_buff_index = 0;
			}
		}
		else if (ch == CHANNEL_2_ID)
		{
			adc_samples_CH2[CH2_buff_index] = data;
			if (++CH2_buff_index >= ADC_SAMPLE_BUFFER_SIZE)
			{
				CH2_buff_index = 0;
			}
		}
		else if (ch == CHANNEL_3_ID)
		{
			adc_samples_CH3[CH3_buff_index] = data;
			if (++CH3_buff_index >= ADC_SAMPLE_BUFFER_SIZE)
			{
				CH3_buff_index = 0;
			}
		}
		else if (ch == CHANNEL_4_ID)
		{
			adc_samples_CH4[CH4_buff_index] = data;
			if (++CH4_buff_index >= ADC_SAMPLE_BUFFER_SIZE)
			{
				CH4_buff_index = 0;
			}
		}
		else if (ch == CHANNEL_5_ID)
		{
			adc_samples_CH5[CH5_buff_index] = data;
			if (++CH5_buff_index >= ADC_SAMPLE_BUFFER_SIZE)
			{
				CH5_buff_index = 0;
			}
		}
		else if (ch == CHANNEL_6_ID)
		{
			adc_samples_CH6[CH6_buff_index] = data;
			if (++CH6_buff_index >= ADC_SAMPLE_BUFFER_SIZE)
			{
				CH6_buff_index = 0;
			}
		}
		else if (ch == CHANNEL_7_ID)
		{
			if (++CH7_buff_index >= ADC_SAMPLE_BUFFER_SIZE)
			{
				CH7_buff_index = 0;
			}
			adc_samples_CH7[CH7_buff_index] = data;
		}
		else
		{
			return; // ESP_LOGE(ADC_DMA_TAG, "Channel undetermined!");
		}
	}

	void perform_adc_dma_read(void)
	{
		uint16_t iter = 0;
		ret = adc_digi_read_bytes(result, TIMES, &ret_num, ADC_MAX_DELAY);
		const uint16_t temp_ret_num = ret_num;
		if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE)
		{
			for (int i = 0; i < temp_ret_num; i += ADC_RESULT_BYTE)
			{
				iter++;
				adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
				channel_sampler(p->type1.channel, p->type1.data);
			}
			vTaskDelay(1);
		}
		else if (ret == ESP_ERR_TIMEOUT)
		{
			vTaskDelay(1000);
		}
	}

#ifdef __cplusplus
}
#endif

#endif
