
 #include "spi_ads1298.h"
 #include "driver/spi_master.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 #include "driver/gpio.h"
 #include "esp_log.h"
 #include <stdlib.h>
 #include <string.h>


 
 #define ADS1298_CMD_RDATA   0x12  
 #define ADS1298_CMD_RDATAC  0x10  
 #define ADS1298_CMD_SDATAC  0x11 

 struct ads1298_context_t {
     ads1298_config_t cfg;
     spi_device_handle_t spi;
     SemaphoreHandle_t drdy_sem;
     bool drdy_intr_enabled;
 };

 typedef struct ads1298_context_t ads1298_context_t;


 //wrapper for spi_device_polling_transmit
 static esp_err_t ads1298_transmit(ads1298_context_t *ctx,
                                  spi_transaction_t *transaction)
{
    return spi_device_polling_transmit(ctx->spi, transaction);
}

 // ISR per DRDY, quando drdy è low e quindi ho un dato da leggere sul ads1298
 static void IRAM_ATTR ads1298_drdy_isr(void *arg)
{
    ads1298_context_t *ctx = (ads1298_context_t *)arg;
    BaseType_t higher_priority_task_woken = pdFALSE;

    xSemaphoreGiveFromISR(ctx->drdy_sem, &higher_priority_task_woken);

    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}
//function to configure GPIO output pin
 static esp_err_t ads1298_configure_gpio_output(gpio_num_t pin, int level)
{
    if (pin < 0) {
        return ESP_OK;
    }

    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&cfg);
    if (err == ESP_OK) {
        err = gpio_set_level(pin, level);
    }
    return err;
}

// to configure ads1298
 esp_err_t ads1298_init(const ads1298_config_t *cfg, ads1298_handle_t *out_handle)
 {
    //controllo che i parametri siano validi
	 if (cfg == NULL || out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    //controllo che i pin obbligatori siano validi
    if (cfg->cs_io < 0 || cfg->drdy_io < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    //controllo che la memoria sia allocata correttamente
    ads1298_context_t *ctx = calloc(1, sizeof(*ctx));
    if (ctx == NULL) {
        return ESP_ERR_NO_MEM;
    }

     ctx->cfg = *cfg;
     ctx->drdy_intr_enabled = cfg->drdy_intr_used;

// configure GPIOs for CS, RESET, START
    esp_err_t err = ads1298_configure_gpio_output(cfg->cs_io, 1);
    if (err != ESP_OK) {
        goto cleanup;
    }

    err = ads1298_configure_gpio_output(cfg->reset_io, 1);
    if (err != ESP_OK) {
        goto cleanup;
    }

    err = ads1298_configure_gpio_output(cfg->start_io, 0);
    if (err != ESP_OK) {
        goto cleanup;
    }
//configuro il pin DRDY come input e se è abilitato l'interrupt lo registro
gpio_config_t drdy_cfg = {
        .pin_bit_mask = 1ULL << cfg->drdy_io,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = cfg->drdy_intr_used ? GPIO_INTR_NEGEDGE : GPIO_INTR_DISABLE,
    };

    err = gpio_config(&drdy_cfg);
    if (err != ESP_OK) {
        goto cleanup;
    }

    if (cfg->drdy_intr_used) {
        ctx->drdy_sem = xSemaphoreCreateBinary();
        if (ctx->drdy_sem == NULL) {
            err = ESP_ERR_NO_MEM;
            goto cleanup;
        }

        err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            goto cleanup;
        }

        err = gpio_isr_handler_add(cfg->drdy_io,
                                   ads1298_drdy_isr,
                                   ctx);
        if (err != ESP_OK) {
            goto cleanup;
        }
    }

     // SPI device configuration
     spi_device_interface_config_t devcfg = {
         .clock_speed_hz = cfg->spi_clock_hz,
         .mode = cfg->spi_mode,
         .spics_io_num = cfg->cs_io,
         .queue_size = 1,
         .flags = 0,  //full duplex
     };

     err = spi_bus_add_device(cfg->host, &devcfg, &ctx->spi);
     if (err != ESP_OK) {
         goto cleanup;
     }

     *out_handle = (ads1298_handle_t)ctx;
     return ESP_OK;
	 
//gestisco gli errori 
 cleanup:
     if (ctx->spi) {
         spi_bus_remove_device(ctx->spi);
     }
     if (ctx->drdy_sem) {
         vSemaphoreDelete(ctx->drdy_sem);
     }
     if (ctx->drdy_intr_enabled) {
         gpio_isr_handler_remove(cfg->drdy_io);
     }
	 //libero la memoria
     free(ctx);
     return err;
 }
// de-inizializzo il dispositivo, lo tolgo dl bus SPI, deregistro ISR e libero la memoria
 esp_err_t ads1298_deinit(ads1298_handle_t handle)
 {
     if (!handle) {
         return ESP_ERR_INVALID_ARG;
     }
     ads1298_context_t *ctx = (ads1298_context_t *)handle;

     if (ctx->spi) {
         spi_bus_remove_device(ctx->spi);
     }
     if (ctx->drdy_intr_enabled) {
         gpio_isr_handler_remove(ctx->cfg.drdy_io);
     }
     if (ctx->drdy_sem) {
         vSemaphoreDelete(ctx->drdy_sem);
     }
     free(ctx);
     return ESP_OK;
 }

 esp_err_t ads1298_send_command(ads1298_handle_t handle, uint8_t cmd)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ads1298_context_t *ctx = (ads1298_context_t *)handle;

    spi_transaction_t t = {
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA,
    };
    t.tx_data[0] = cmd;

    return ads1298_transmit(ctx, &t);
}

 esp_err_t ads1298_write_reg(ads1298_handle_t handle, uint8_t reg_addr, uint8_t value)
 {
    //controllo che l'handle e il registro siano validi
     if (handle == NULL || reg_addr > 0x1F) {
        return ESP_ERR_INVALID_ARG;
    }

    ads1298_context_t *ctx = (ads1298_context_t *)handle;

     // WREG command format: 010r rrrr with r rrrr the starting register address 
	 uint8_t tx[3] = {
        ADS1298_CMD_WREG | (reg_addr & 0x1F),
        0x00,
        value,
    };

    spi_transaction_t t = {
        .length = sizeof(tx) * 8,
        .tx_buffer = tx,
    };

    return ads1298_transmit(ctx, &t);
 }

 esp_err_t ads1298_read_reg(ads1298_handle_t handle, uint8_t reg_addr, uint8_t *out_value)
 {
      if (handle == NULL || out_value == NULL || reg_addr > 0x1F) {
        return ESP_ERR_INVALID_ARG;
    }

    ads1298_context_t *ctx = (ads1298_context_t *)handle;


     // RREG command format: 001r rrrr with r rrrr the starting register address 
      /* RREG requires the command/count phase followed by clocking data. */
    uint8_t tx[3] = {
        ADS1298_CMD_RREG | (reg_addr & 0x1F),
        0x00,
        0x00,
    };

    uint8_t rx[3] = {0};

    spi_transaction_t t = {
        .length = sizeof(tx) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

     esp_err_t err = ads1298_transmit(ctx, &t);
    if (err == ESP_OK) {
        *out_value = rx[2];
    }
    return err;
}



 esp_err_t ads1298_wait_drdy(ads1298_handle_t handle, uint32_t timeout_ms)
 {
      if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ads1298_context_t *ctx = (ads1298_context_t *)handle;


     if (ctx->drdy_intr_enabled) {
        if (xSemaphoreTake(ctx->drdy_sem, 0) == pdTRUE) {
            /* Clear a previously queued event. */
        }

        return (xSemaphoreTake(ctx->drdy_sem,
                               pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
                   ? ESP_OK
                   : ESP_ERR_TIMEOUT;
    }

    TickType_t start = xTaskGetTickCount();
    while (gpio_get_level(ctx->cfg.drdy_io) != 0) {
        if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(timeout_ms)) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }

    return ESP_OK;
 }
 
esp_err_t ads1298_read_data(ads1298_handle_t handle,
                           uint8_t *out_buf,
                           size_t buf_len)
{
    if (handle == NULL || out_buf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (buf_len != ADS1298_FRAME_BYTES_SINGLE &&
        buf_len != ADS1298_FRAME_BYTES_DAISY) {
        return ESP_ERR_INVALID_SIZE;
    }

    ads1298_context_t *ctx =
        (ads1298_context_t *)handle;

    /*
     * Single-device operation:
     * no inter-device bit needs to be removed.
     */
    if (buf_len == ADS1298_FRAME_BYTES_SINGLE) {
        uint8_t dummy[ADS1298_FRAME_BYTES_SINGLE] = {0};

        spi_transaction_t t = {
            .length = ADS1298_FRAME_BYTES_SINGLE * 8U,
            .rxlength = ADS1298_FRAME_BYTES_SINGLE * 8U,
            .tx_buffer = dummy,
            .rx_buffer = out_buf,
        };

        return ads1298_transmit(ctx, &t);
    }

    /*
     * Two-device daisy chain:
     * 216 data bits + 1 inter-device bit + 216 data bits.
     */
    uint8_t raw[ADS1298_RAW_BYTES_DAISY] = {0};
uint8_t dummy[ADS1298_RAW_BYTES_DAISY] = {0};

/*
 * Read whole bytes to avoid the ESP32-C3 8n+1-bit limitation.
 * Useful bits: 216 + 1 + 216 = 433.
 * Clocked bits: 55 * 8 = 440.
 */
spi_transaction_t t = {
    .length = ADS1298_RAW_BYTES_DAISY * 8U,
    .rxlength = ADS1298_RAW_BYTES_DAISY * 8U,
    .tx_buffer = dummy,
    .rx_buffer = raw,
};

esp_err_t err = ads1298_transmit(ctx, &t);

if (err != ESP_OK) {
    return err;
}

/* First device: already aligned. */
memcpy(out_buf,
       raw,
       ADS1298_FRAME_BYTES_SINGLE);

/* Second device: remove the inter-device bit. */
for (size_t i = 0; i < ADS1298_FRAME_BYTES_SINGLE; i++) {
    const size_t j = ADS1298_FRAME_BYTES_SINGLE + i;

    out_buf[j] = (uint8_t)(
        ((uint32_t)raw[j] << 1) |
        ((uint32_t)raw[j + 1U] >> 7)
    );
}

return ESP_OK;
}


 esp_err_t ads1298_configure_daisy_chain(ads1298_handle_t handle,
                                        uint8_t config1_value)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Clear DAISY_EN. Keep the data-rate bits selected by the caller. */
    config1_value &= (uint8_t)~0x01U;
    return ads1298_write_reg(handle,ADS1298_REG_CONFIG1,config1_value);
}

int32_t ads1298_decode_24bit(const uint8_t *p)
{
    if (p == NULL) {
        return 0;
    }

    int32_t value = ((int32_t)p[0] << 16) |
                    ((int32_t)p[1] << 8) |
                    (int32_t)p[2];

    if ((value & 0x00800000) != 0) {
        value |= (int32_t)0xFF000000;
    }

    return value;
}

