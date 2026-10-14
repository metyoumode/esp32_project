#include "spi_ads1298.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "driver/ledc.h"

static const char TAG[] = "main";

// Pin configuration for ESP32-C3 WROOM-02 (adapt to your schematic)
#define ADS_HOST SPI2_HOST
#define PIN_NUM_MOSI 7
#define PIN_NUM_MISO 8
#define PIN_NUM_SCLK 5
#define PIN_NUM_CS 6
#define PIN_NUM_DRDY 4
#define PIN_NUM_RESET -1 // unused
#define PIN_NUM_START -1 // unused
/* external clock to ADS1298 CLK */
#define PIN_NUM_ADS_CLK    GPIO_NUM_3
#define ADS_CLK_FREQ_HZ    2048000
#define ADS1298_FRAME_BYTES_SINGLE 27U
#define ADS1298_NUM_CHANNELS       8U

static void parse_ads1298_device_frame(const uint8_t *frame,
									   int32_t channels[ADS1298_CHANNELS],
									   uint8_t status[ADS1298_STATUS_BYTES]) {
	for (size_t i = 0; i < ADS1298_STATUS_BYTES; i++) {
		status[i] = frame[i];
	}

	for (size_t ch = 0; ch < ADS1298_CHANNELS; ch++) {
		channels[ch] = ads1298_decode_24bit(
			&frame[ADS1298_STATUS_BYTES + ch * ADS1298_BYTES_PER_CHANNEL]);
	}
}

static esp_err_t ads1298_clock_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_1_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = ADS_CLK_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        return err;
    }

    ledc_channel_config_t channel_cfg = {
        .gpio_num = PIN_NUM_ADS_CLK,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 1,
        .hpoint = 0,
    };

    err = ledc_channel_config(&channel_cfg);
    if (err != ESP_OK) {
        return err;
    }

    return ledc_update_duty(LEDC_LOW_SPEED_MODE,
                            LEDC_CHANNEL_0);
}


static void ads1298_print_dual_channels(
    const int32_t ads1[ADS1298_CHANNELS],
    const int32_t ads2[ADS1298_CHANNELS])
{
    ESP_LOGI(TAG,
             "ADS1: "
             "CH1=%ld CH2=%ld CH3=%ld CH4=%ld "
             "CH5=%ld CH6=%ld CH7=%ld CH8=%ld",
             (long)ads1[0], (long)ads1[1],
             (long)ads1[2], (long)ads1[3],
             (long)ads1[4], (long)ads1[5],
             (long)ads1[6], (long)ads1[7]);

    ESP_LOGI(TAG,
             "ADS2: "
             "CH1=%ld CH2=%ld CH3=%ld CH4=%ld "
             "CH5=%ld CH6=%ld CH7=%ld CH8=%ld",
             (long)ads2[0], (long)ads2[1],
             (long)ads2[2], (long)ads2[3],
             (long)ads2[4], (long)ads2[5],
             (long)ads2[6], (long)ads2[7]);
}

static void config_inputs_for_all(ads1298_handle_t ads_handle,
                                  uint8_t ch_set)
{
    for (uint8_t ch = 0; ch < ADS1298_NUM_CHANNELS; ch++) {
        ESP_ERROR_CHECK(ads1298_write_reg(
            ads_handle,
            ADS1298_REG_CH1SET + ch,
            ch_set));
    }
}



void app_main(void) {
	esp_err_t ret;

	// Initialize SPI bus
	ESP_LOGI(TAG, "Initializing SPI bus...");
	spi_bus_config_t buscfg = {
		.mosi_io_num = PIN_NUM_MOSI,
		.miso_io_num = PIN_NUM_MISO,
		.sclk_io_num = PIN_NUM_SCLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = ADS1298_RAW_BYTES_DAISY,
	};

	ret = spi_bus_initialize(ADS_HOST, &buscfg, SPI_DMA_CH_AUTO);
	ESP_ERROR_CHECK(ret);

	ESP_LOGI(TAG,
         "Starting ADS1298 external clock on GPIO%d at %d Hz",
         PIN_NUM_ADS_CLK,
         ADS_CLK_FREQ_HZ);

     ESP_ERROR_CHECK(ads1298_clock_init());

     /* Let the generated clock stabilize before talking to ADS1298. */
    vTaskDelay(pdMS_TO_TICKS(1));

	ads1298_config_t ads_cfg = {
		.host = ADS_HOST,
		.cs_io = PIN_NUM_CS,
		.drdy_io = PIN_NUM_DRDY, 
		.reset_io = PIN_NUM_RESET,
		.start_io = PIN_NUM_START,
		.spi_clock_hz = 2 * 1000 * 1000, // 2 MHz
		.spi_mode = 1,					 // adjust per ADS1298 datasheet
		.drdy_intr_used = true,
	};

	ads1298_handle_t ads_handle = NULL;

	ESP_LOGI(TAG, "Initializing ADS1298...");
	ret = ads1298_init(&ads_cfg, &ads_handle);
	ESP_ERROR_CHECK(ret);

	// resetto il dispositivo
	ESP_ERROR_CHECK(ads1298_send_command(ads_handle, ADS1298_CMD_RESET));
	vTaskDelay(pdMS_TO_TICKS(1));

    //stop continuous data mode before register configuraton
	ESP_ERROR_CHECK(ads1298_send_command(ads_handle, ADS1298_CMD_SDATAC));
	vTaskDelay(pdMS_TO_TICKS(1));

    // Scrive sul registro CONFIG3: Riferimento interno, si DRL
	ESP_ERROR_CHECK(ads1298_write_reg(ads_handle, ADS1298_REG_CONFIG3, 0xCC));
	vTaskDelay(pdMS_TO_TICKS(1));

    // No electrodes feeds the DRL
	ESP_ERROR_CHECK(ads1298_write_reg(ads_handle, ADS1298_REG_RLD_SENSN, 0x00));
	vTaskDelay(pdMS_TO_TICKS(1));

    // No electrodes feeds the DRL
	ESP_ERROR_CHECK(ads1298_write_reg(ads_handle, ADS1298_REG_RLD_SENSP, 0x00));
	vTaskDelay(pdMS_TO_TICKS(10));
     
	/* Example: enable all channels, gain 6, normal electrode input. */
	//qui il codice del stm32 ha uno switch che ti permette ti cambiare il gain
	//li viene fatto con i gain: 2,4,6,8,12 ma di default c'è gain=8
	
    config_inputs_for_all(ads_handle, 0x50); //0x50 setta gain=8

	/* Start continuous data output and conversions. */
  	ESP_ERROR_CHECK(ads1298_send_command(ads_handle, ADS1298_CMD_START));
    vTaskDelay(pdMS_TO_TICKS(1));

	ESP_ERROR_CHECK(ads1298_send_command(ads_handle, ADS1298_CMD_RDATAC));

	ESP_LOGI(TAG, "ADS1298 daisy chain started");

	uint8_t frame[ADS1298_FRAME_BYTES_DAISY];
	int32_t ads1_channels[ADS1298_CHANNELS];
	int32_t ads2_channels[ADS1298_CHANNELS];
	uint8_t ads1_status[ADS1298_STATUS_BYTES];
	uint8_t ads2_status[ADS1298_STATUS_BYTES];

	while (1) {
		// Wait for new data
		ret = ads1298_wait_drdy(ads_handle,
                        ADS1298_DRDY_TIMEOUT_MS);

        if (ret != ESP_OK) {
            ESP_LOGW(TAG,"DRDY timeout: %s; GPIO%d level=%d",esp_err_to_name(ret),(int)PIN_NUM_DRDY,gpio_get_level(PIN_NUM_DRDY));

    continue;
}

		// Read one sample set
		ret = ads1298_read_data(ads_handle, frame, sizeof(frame));
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Error reading dual ADS1298 frame: %s",esp_err_to_name(ret));
            vTaskDelay(10);
            continue;
		}

		/* Assumption: first 27-byte block is ADS1, second is ADS2. */
		parse_ads1298_device_frame(&frame[0], ads1_channels,ads1_status); // ads1

		parse_ads1298_device_frame(&frame[ADS1298_FRAME_BYTES_SINGLE],ads2_channels, ads2_status); // ads2

		ads1298_print_dual_channels(ads1_channels, ads2_channels);

		/* Process or queue the 16 decoded channels here. */
		// TODO: parse frame (status + 24-bit channels)

		vTaskDelay(1); // optional small delay
	}
}
