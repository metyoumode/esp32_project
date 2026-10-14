#ifndef ADS1298_H
#define ADS1298_H


#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"


#ifdef __cplusplus
extern "C" {
#endif

#define		MULTIPLEADS1298

#define     HI_RES_MODE

#define		NUM_SHIFT_GAIN_ADS	3

#define		DEFAULT_GAIN_24		2		// Guadagno di default per l'ADS1298 se a 24 bit
#define		DEFAULT_GAIN_16		8		// Guadagno di default per l'ADS1298 se a 16 bit

//all ads1298 command
/* ADS1298 commands */
#define ADS1298_CMD_WAKEUP   0x02
#define ADS1298_CMD_STANDBY  0x04
#define ADS1298_CMD_RESET    0x06
#define ADS1298_CMD_START    0x08
#define ADS1298_CMD_STOP     0x0A
#define ADS1298_CMD_RDATAC   0x10
#define ADS1298_CMD_SDATAC   0x11
#define ADS1298_CMD_RDATA    0x12
#define ADS1298_CMD_RREG     0x20
#define ADS1298_CMD_WREG     0x40

/* ADS1298 register addresses */
#define ADS1298_REG_ID        0x00
#define ADS1298_REG_CONFIG1   0x01
#define ADS1298_REG_CONFIG2   0x02
#define ADS1298_REG_CONFIG3   0x03
#define ADS1298_REG_LOFF      0x04
#define ADS1298_REG_CH1SET    0x05
#define ADS1298_REG_CH2SET    0x06
#define ADS1298_REG_CH3SET    0x07
#define ADS1298_REG_CH4SET    0x08
#define ADS1298_REG_CH5SET    0x09
#define ADS1298_REG_CH6SET    0x0A
#define ADS1298_REG_CH7SET    0x0B
#define ADS1298_REG_CH8SET    0x0C
#define ADS1298_REG_RLD_SENSP 0x0D
#define ADS1298_REG_RLD_SENSN 0x0E
#define ADS1298_REG_LOFF_SENSP 0x0F
#define ADS1298_REG_LOFF_SENSN 0x10
#define ADS1298_REG_GPIO      0x14
#define ADS1298_REG_PACE      0x15
#define ADS1298_REG_RESP      0x16
#define ADS1298_REG_CONFIG4   0x17
#define ADS1298_REG_WCT1      0x18
#define ADS1298_REG_WCT2      0x19

#define ADS1298_CHANNELS              8U
#define ADS1298_STATUS_BYTES         3U
#define ADS1298_BYTES_PER_CHANNEL    3U
#define ADS1298_FRAME_BYTES_SINGLE   27U //(ADS1298_STATUS_BYTES + ADS1298_CHANNELS * ADS1298_BYTES_PER_CHANNEL)
#define ADS1298_FRAME_BYTES_DAISY    54U //(2U * ADS1298_FRAME_BYTES_SINGLE)
#define ADS1298_RAW_BYTES_DAISY      55U    // including the extra inter-device bit, since we are using 2 ads1298
#define ADS1298_RAW_BITS_DAISY      433U //216 + 1 + 216 = 433 bits, rounded up to 55 bytes.
#define ADS1298_DRDY_TIMEOUT_MS      100U

/// Configuration for the ADS1298 device
typedef struct {
    spi_host_device_t host;     ///< SPI host (SPI2_HOST or SPI3_HOST)
    gpio_num_t cs_io;           ///< Chip select (active low)
    gpio_num_t drdy_io;         ///< DRDY input (active low)
    gpio_num_t reset_io;        ///< RESET (optional, set to -1 if unused)
    gpio_num_t start_io;        ///< START (optional, set to -1 if unused)
    uint32_t spi_clock_hz;      ///< SPI clock frequency (e.g. 2 MHz)
    uint8_t spi_mode;           ///< SPI mode (0â€“3), see ADS1298 datasheet
    bool drdy_intr_used;        ///< Use interrupt for DRDY (recommended)
} ads1298_config_t;

typedef struct ads1298_context_t* ads1298_handle_t;

/**
 * @brief Initialize the ADS1298 hardware and SPI device.
 *
 * @param config Configuration structure.
 * @param out_handle Output handle for subsequent calls.
 * @return ESP_OK on success, or other esp_err_t from SPI/GPIO functions.
 */
esp_err_t ads1298_init(const ads1298_config_t *config, ads1298_handle_t *out_handle);

/**
 * @brief Deinitialize the ADS1298 device and free resources.
 *
 * @param handle Device handle.
 * @return ESP_OK
 */
esp_err_t ads1298_deinit(ads1298_handle_t handle);

/**
 * @brief Write an ADS1298 register.
 *
 * @param handle Device handle.
 * @param reg_addr Register address (see datasheet).
 * @param value Value to write (8-bit).
 * @return ESP_OK on success, or error from SPI functions.
 */
esp_err_t ads1298_write_reg(ads1298_handle_t handle, uint8_t reg_addr, uint8_t value);

/**
 * @brief Read an ADS1298 register.
 *
 * @param handle Device handle.
 * @param reg_addr Register address.
 * @param out_value Pointer to store the read value.
 * @return ESP_OK on success, or error from SPI functions.
 */
esp_err_t ads1298_read_reg(ads1298_handle_t handle, uint8_t reg_addr, uint8_t *out_value);
/**
 * @brief Wait for DRDY to go low (new data ready).
 *
 * @param handle Device handle.
 * @param timeout_ms Timeout in milliseconds.
 * @return ESP_OK if DRDY detected, ESP_ERR_TIMEOUT on timeout.
 */
esp_err_t ads1298_wait_drdy(ads1298_handle_t handle, uint32_t timeout_ms);

/**
 * @brief Read one sample set from the ADS1298 (status + N channels).
 *
 * @note Adjust buffer size and parsing according to your channel count
 *       and data format (24-bit per channel).
 *
 * @param handle Device handle.
 * @param out_buf Buffer to store raw data (status + channels).
 * @param buf_len Length of out_buf in bytes.
 * @return ESP_OK on success, or error from SPI functions.
 */
esp_err_t ads1298_read_data(ads1298_handle_t handle, uint8_t *out_buf, size_t buf_len);

/**
 * @brief Send a command byte to the ADS1298 (e.g. RDATA, RDATAC, STOP, START).
 *
 * @param handle Device handle.
 * @param cmd Command byte (see datasheet).
 * @return ESP_OK on success, or error from SPI functions.
 */
esp_err_t ads1298_send_command(ads1298_handle_t handle, uint8_t cmd);

/* Broadcast one value to both devices on the shared DIN/CS bus. */
esp_err_t ads1298_configure_daisy_chain(ads1298_handle_t handle, uint8_t config1_value);


/* Decode one big-endian signed 24-bit ADS1298 sample. */
int32_t ads1298_decode_24bit(const uint8_t *p);

#ifdef __cplusplus
}
#endif

#endif
