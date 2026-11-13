#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_system.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_gc9a01.h"
#include "driver/spi_master.h"
#include "esp_attr.h"

#include "i2cdev.h"
#include "scd4x.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_spiffs.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"


/*
 * ============================================================
 * ZIGBEE 1.6.8 API
 * ============================================================
 */
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zigbee_ota.h"


// ============================================================
// Настройки
// ============================================================

#define TAG "CO2"
#define ZIGBEE_TAG "ZIGBEE"


// ------------------------------------------------------------
// I2C
// ------------------------------------------------------------

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_GPIO    GPIO_NUM_10
#define I2C_SCL_GPIO    GPIO_NUM_11
#define I2C_SPEED_HZ    100000


// ------------------------------------------------------------
// SCD40
// ------------------------------------------------------------

#define SCD40_ADDRESS   0x62
#define SCD40_STARTUP_SETTLE_DELAY_MS 500
#define FIRST_ZIGBEE_REPORT_DELAY_MS 10000


// ------------------------------------------------------------
// GC9A01 round TFT (SPI, 240x240)
// ------------------------------------------------------------

#define DISPLAY_WIDTH               240
#define DISPLAY_HEIGHT              240
#define DISPLAY_STRIPE_HEIGHT       68
#define DISPLAY_SPI_HOST            SPI2_HOST
#define DISPLAY_SPI_SCLK_GPIO       GPIO_NUM_12
#define DISPLAY_SPI_MOSI_GPIO       GPIO_NUM_14
#define DISPLAY_DC_GPIO             GPIO_NUM_1
#define DISPLAY_RST_GPIO            GPIO_NUM_3
#define DISPLAY_CS_GPIO             GPIO_NUM_2  // CS is connected to ESP32-H2 GPIO2
#define BOARD_BLUE_LED_GPIO         GPIO_NUM_13
#define DISPLAY_SPI_CLOCK_HZ        (20 * 1000 * 1000)

// Light sensor divider: LDR to +3.3 V, 10 kOhm resistor to GND, midpoint to GPIO5.
#define LIGHT_SENSOR_GPIO           GPIO_NUM_5
#define LIGHT_SENSOR_ADC_CHANNEL    ADC_CHANNEL_4  // ESP32-H2 GPIO5 = ADC1_CH4
#define LIGHT_ADC_DARK_RAW          30
#define LIGHT_ADC_BRIGHT_RAW        3000
#define LIGHT_SAMPLE_INTERVAL_MS    500
#define LIGHT_CAL_49_LUX_RAW        2277
#define LIGHT_CAL_304_LUX_RAW       3017
#define LIGHT_CAL_49_LUX_ZCL        16903U
#define LIGHT_CAL_304_LUX_ZCL       24830U
#define BACKLIGHT_FADE_STEP_PERCENT  1
#define BACKLIGHT_MIN_PERCENT       10
#define BACKLIGHT_MAX_PERCENT       100

// GPIO0 drives the N-MOSFET gate. MOSFET ON pulls BLK low and turns the backlight off.
#define BACKLIGHT_GPIO              GPIO_NUM_0
#define BACKLIGHT_LEDC_MODE         LEDC_LOW_SPEED_MODE
#define BACKLIGHT_LEDC_TIMER        LEDC_TIMER_0
#define BACKLIGHT_LEDC_CHANNEL      LEDC_CHANNEL_0
#define BACKLIGHT_PWM_FREQUENCY_HZ  20000
#define BACKLIGHT_PWM_DUTY_MAX      1023U
#define BACKLIGHT_PWM_RESOLUTION    LEDC_TIMER_10_BIT

// ------------------------------------------------------------
// Кнопка
// ------------------------------------------------------------

#define ZIGBEE_BUTTON_GPIO      GPIO_NUM_4
#define ZIGBEE_LONG_PRESS_MS    3000


// ------------------------------------------------------------
// Zigbee
// ------------------------------------------------------------

#define ZIGBEE_ENDPOINT                 1
#define SENSOR_CALIBRATION_CLUSTER_ID   0xFC00U
#define SENSOR_CALIBRATION_TEMP_ATTR_ID 0x0001U
#define SENSOR_CALIBRATION_HUM_ATTR_ID  0x0002U

/* Bump OTA_FILE_VERSION for every published image. Keep manufacturer/image
 * identifiers identical in the Zigbee OTA file and the Zigbee2MQTT index. */
#define FW_APPLICATION_VERSION          1U
#define OTA_FILE_VERSION                0x00010006UL
#define OTA_MANUFACTURER_CODE           0x1234U
#define OTA_IMAGE_TYPE                  0x5678U
#define OTA_MAX_DATA_SIZE               223U
#define OTA_HEADER_SIZE                 56U
#define OTA_SUBELEMENT_HEADER_SIZE      6U

#define ZIGBEE_STORAGE_PARTITION_NAME   "zb_storage"

/*
 * Zigbee channel 25
 *
 * channel 25 = bit 25
 * 1 << 25 = 0x02000000
 */
#define ZIGBEE_PRIMARY_CHANNEL_MASK 0x07FFF800UL

#define ZIGBEE_SECONDARY_CHANNEL_MASK   0U


/*
 * Zigbee Basic cluster strings.
 *
 * First byte = string length.
 */
#define ZIGBEE_MANUFACTURER_NAME \
    "\x06" "Custom"

#define ZIGBEE_MODEL_IDENTIFIER \
    "\x0B" "ESP32H2_CO2"


#define ZIGBEE_MAX_CHILDREN 10


// ============================================================
// Глобальные устройства
// ============================================================

// Настоящий тип из esp-idf-lib/scd4x
static i2c_dev_t scd40;

// Общая I2C-шина, которой управляет i2cdev
static i2c_master_bus_handle_t i2c_bus;

// ESP-IDF LCD I2C interface
static esp_lcd_panel_io_handle_t display_io;

// ESP-IDF GC9A01 panel
static esp_lcd_panel_handle_t display_panel;

// The SPI panel transfers each stripe asynchronously; don't reuse its DMA buffer until done.
static SemaphoreHandle_t display_transfer_done;
static adc_oneshot_unit_handle_t light_sensor_adc;
static bool light_sensor_adc_ready;
static bool backlight_pwm_ready;
static volatile int light_sensor_filtered_raw = -1;
static uint8_t backlight_brightness_percent = BACKLIGHT_MAX_PERCENT;
static uint8_t backlight_target_brightness_percent = BACKLIGHT_MAX_PERCENT;
static TickType_t last_light_sample_tick;
static TickType_t last_light_log_tick;

static bool display_color_transfer_done(
    esp_lcd_panel_io_handle_t panel_io,
    esp_lcd_panel_io_event_data_t *event_data,
    void *user_ctx
)
{
    (void)panel_io;
    (void)event_data;
    SemaphoreHandle_t transfer_done = (SemaphoreHandle_t)user_ctx;
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(transfer_done, &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}

// One DMA stripe fits a complete CO2 digit row, so it is sent in one transfer.
DMA_ATTR static uint16_t display_buffer[DISPLAY_WIDTH * DISPLAY_STRIPE_HEIGHT];

typedef struct { uint16_t width, height; uint8_t *coverage; } display_asset_t;
enum {
    ASSET_BIG_0, ASSET_SMALL_0 = 10, ASSET_PPM = 20, ASSET_CO2 = 21,
    ASSET_WIFI = 22, ASSET_THERMO = 23, ASSET_WATER = 24,
    ASSET_GLYPH_O = 25, ASSET_GLYPH_C = 26, ASSET_GLYPH_PERCENT = 27,
    ASSET_GLYPH_DOT = 28, ASSET_GLYPH_MINUS = 29, ASSET_GLYPH_F = 30, ASSET_COUNT = 31
};
static display_asset_t display_assets[ASSET_COUNT];

// Zigbee stack has been started and can be accessed from the sensor task.
static volatile bool zigbee_stack_started = false;
static volatile bool zigbee_network_joined = false;
static volatile bool zigbee_network_joining = false;
static uint8_t zigbee_application_version = FW_APPLICATION_VERSION;

static const esp_partition_t *ota_update_partition;
static esp_ota_handle_t ota_update_handle;
static bool ota_update_in_progress;
static bool ota_update_image_element;
static bool ota_metadata_valid;
static uint32_t ota_stream_offset;
static uint32_t ota_expected_size;
static uint32_t ota_sub_element_remaining;
static uint8_t ota_sub_element_header[OTA_SUBELEMENT_HEADER_SIZE];
static uint8_t ota_sub_element_header_bytes;


// ============================================================
// Zigbee signal handler
// ============================================================

void esp_zb_app_signal_handler(
    esp_zb_app_signal_t *signal_struct
)
{
    uint32_t *p_sg_p =
        signal_struct->p_app_signal;

    esp_err_t err_status =
        signal_struct->esp_err_status;

    esp_zb_app_signal_type_t sig_type =
        (esp_zb_app_signal_type_t)(*p_sg_p);


    switch (sig_type) {

        // ----------------------------------------------------
        // Zigbee stack initialized
        // ----------------------------------------------------

        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:

            ESP_LOGI(
                ZIGBEE_TAG,
                "Zigbee stack initialized"
            );

            ESP_ERROR_CHECK(
                esp_zb_bdb_start_top_level_commissioning(
                    ESP_ZB_BDB_MODE_INITIALIZATION
                )
            );

            break;


        // ----------------------------------------------------
        // First start / reboot
        // ----------------------------------------------------

        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:

        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:

            if (err_status == ESP_OK) {

                ESP_LOGI(
                    ZIGBEE_TAG,
                    "Device started, factory_new=%s",

                    esp_zb_bdb_is_factory_new()
                        ? "YES"
                        : "NO"
                );


                /*
                 * Если устройство новое —
                 * запускаем network steering.
                 */

                if (esp_zb_bdb_is_factory_new()) {

                    zigbee_network_joining = true;

                    ESP_LOGI(
                        ZIGBEE_TAG,
                        "Starting network steering..."
                    );

                    ESP_ERROR_CHECK(
                        esp_zb_bdb_start_top_level_commissioning(
                            ESP_ZB_BDB_MODE_NETWORK_STEERING
                        )
                    );

                } else {
                    zigbee_network_joined = true;
                    zigbee_network_joining = false;

                    ESP_LOGI(
                        ZIGBEE_TAG,
                        "Device rebooted, existing network state retained"
                    );
                }

            } else {

                zigbee_network_joined = false;
                zigbee_network_joining = false;

                ESP_LOGW(
                    ZIGBEE_TAG,
                    "Device startup failed: %s",
                    esp_err_to_name(err_status)
                );
            }

            break;


        // ----------------------------------------------------
        // Network steering result
        // ----------------------------------------------------

        case ESP_ZB_BDB_SIGNAL_STEERING:

            zigbee_network_joining = false;

            if (err_status == ESP_OK) {

                zigbee_network_joined = true;
                esp_zb_ieee_addr_t extended_pan_id;

                esp_zb_get_extended_pan_id(
                    extended_pan_id
                );


                ESP_LOGI(
                    ZIGBEE_TAG,
                    "========================================"
                );

                ESP_LOGI(
                    ZIGBEE_TAG,
                    "JOINED Zigbee network!"
                );

                ESP_LOGI(
                    ZIGBEE_TAG,
                    "PAN ID: 0x%04hx",
                    esp_zb_get_pan_id()
                );

                ESP_LOGI(
                    ZIGBEE_TAG,
                    "Extended PAN ID: "
                    "%02x:%02x:%02x:%02x:"
                    "%02x:%02x:%02x:%02x",

                    extended_pan_id[7],
                    extended_pan_id[6],
                    extended_pan_id[5],
                    extended_pan_id[4],
                    extended_pan_id[3],
                    extended_pan_id[2],
                    extended_pan_id[1],
                    extended_pan_id[0]
                );

                ESP_LOGI(
                    ZIGBEE_TAG,
                    "Channel: %d",
                    esp_zb_get_current_channel()
                );

                ESP_LOGI(
                    ZIGBEE_TAG,
                    "Short address: 0x%04hx",
                    esp_zb_get_short_address()
                );

                ESP_LOGI(
                    ZIGBEE_TAG,
                    "========================================"
                );

            } else {

                zigbee_network_joined = false;
                ESP_LOGW(
                    ZIGBEE_TAG,
                    "Network steering failed: %s",
                    esp_err_to_name(err_status)
                );
            }

            break;


        // ----------------------------------------------------
        // Permit join status
        // ----------------------------------------------------

        case ESP_ZB_ZDO_SIGNAL_LEAVE:
            zigbee_network_joined = false;
            zigbee_network_joining = false;
            ESP_LOGW(ZIGBEE_TAG, "Left Zigbee network; reports paused");
            break;

        case ESP_ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
            /* This reports expired neighbor routes, not that this device left
             * the Zigbee network. Keep reporting enabled so the stack can
             * rediscover routes without requiring a device reboot. */
            ESP_LOGW(ZIGBEE_TAG,
                     "No active Zigbee neighbor routes; staying joined and retrying reports");
            break;

        case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:

            if (err_status == ESP_OK) {

                uint8_t permit_duration =
                    *(uint8_t *)
                    esp_zb_app_signal_get_params(
                        p_sg_p
                    );

                if (permit_duration) {

                    ESP_LOGI(
                        ZIGBEE_TAG,
                        "Network 0x%04hx open for %u seconds",

                        esp_zb_get_pan_id(),

                        permit_duration
                    );

                } else {

                    ESP_LOGI(
                        ZIGBEE_TAG,
                        "Network 0x%04hx closed",

                        esp_zb_get_pan_id()
                    );
                }
            }

            break;


        // ----------------------------------------------------
        // Other Zigbee signals
        // ----------------------------------------------------

        default:

            ESP_LOGI(
                ZIGBEE_TAG,
                "ZDO signal: %s (0x%x), status: %s",

                esp_zb_zdo_signal_to_string(sig_type),

                sig_type,

                esp_err_to_name(err_status)
            );

            break;
    }
}


// ============================================================
// Создание Zigbee endpoint
// ============================================================

/* The Zigbee stack removes the 56-byte OTA file header before delivering
 * RECEIVE payloads, but leaves the sub-element header in the byte stream.
 * START may have an empty ota_header on some SDK versions, so retry validation
 * on subsequent callbacks and require metadata by CHECK. */
static esp_err_t ota_validate_callback_header(const esp_zb_ota_file_header_t *header)
{
    if (header == NULL) return ESP_ERR_INVALID_ARG;

    if (header->manufacturer_code == 0 && header->image_type == 0 &&
        header->file_version == 0 && header->image_size == 0) {
        return ESP_OK; /* Metadata is not populated yet; try again later. */
    }

    const uint32_t image_size = header->image_size;
    const uint32_t max_ota_size = ota_update_partition->size +
                                  OTA_HEADER_SIZE + OTA_SUBELEMENT_HEADER_SIZE;
    if (header->manufacturer_code != OTA_MANUFACTURER_CODE ||
        header->image_type != OTA_IMAGE_TYPE ||
        header->file_version <= OTA_FILE_VERSION ||
        header->field_control != 0 ||
        image_size <= OTA_HEADER_SIZE + OTA_SUBELEMENT_HEADER_SIZE ||
        image_size > max_ota_size) {
        ESP_LOGE(ZIGBEE_TAG,
                 "Rejected OTA file header: manufacturer=0x%04x, image_type=0x%04x, "
                 "version=0x%08" PRIx32 ", image_size=%" PRIu32,
                 (unsigned)header->manufacturer_code, (unsigned)header->image_type,
                 header->file_version, image_size);
        return ESP_ERR_INVALID_VERSION;
    }

    /* The Zigbee stack has already removed the 56-byte OTA file header and
     * reports image_size as the remaining stream length in the callback. */
    ota_expected_size = image_size;
    ota_metadata_valid = true;
    ESP_LOGI(ZIGBEE_TAG,
             "OTA file header accepted: manufacturer=0x%04x, image_type=0x%04x, "
             "version=0x%08" PRIx32 ", size=%" PRIu32,
             (unsigned)header->manufacturer_code, (unsigned)header->image_type,
             header->file_version, image_size);
    return ESP_OK;
}

/* RECEIVE payload starts after the OTA file header. It still contains the
 * 6-byte sub-element header, followed by the ESP-IDF application image. */
static esp_err_t ota_write_stream(const uint8_t *data, size_t size)
{
    size_t pos = 0;

    while (pos < size) {
        if (ota_sub_element_remaining == 0) {
            while (pos < size && ota_sub_element_header_bytes < OTA_SUBELEMENT_HEADER_SIZE) {
                ota_sub_element_header[ota_sub_element_header_bytes++] = data[pos++];
                ota_stream_offset++;
            }
            if (ota_sub_element_header_bytes < OTA_SUBELEMENT_HEADER_SIZE) continue;

            uint16_t tag = (uint16_t)ota_sub_element_header[0] |
                           ((uint16_t)ota_sub_element_header[1] << 8);
            ota_sub_element_remaining = (uint32_t)ota_sub_element_header[2] |
                                        ((uint32_t)ota_sub_element_header[3] << 8) |
                                        ((uint32_t)ota_sub_element_header[4] << 16) |
                                        ((uint32_t)ota_sub_element_header[5] << 24);
            ota_update_image_element = tag == 0x0000;
            ota_sub_element_header_bytes = 0;
            if (ota_sub_element_remaining == 0) return ESP_ERR_INVALID_SIZE;
            if (!ota_update_image_element) return ESP_ERR_NOT_SUPPORTED;
            continue;
        }

        size_t count = ota_sub_element_remaining;
        if (count > size - pos) count = size - pos;
        if (ota_update_image_element) {
            esp_err_t err = esp_ota_write(ota_update_handle, data + pos, count);
            if (err != ESP_OK) return err;
        }
        pos += count;
        ota_stream_offset += count;
        ota_sub_element_remaining -= count;
    }

    return ESP_OK;
}

static esp_err_t zigbee_ota_action_handler(
    esp_zb_core_action_callback_id_t callback_id,
    const void *message
)
{
    if (callback_id != ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID || message == NULL) {
        return ESP_OK;
    }

    const esp_zb_zcl_ota_upgrade_value_message_t *ota = message;
    if (ota->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGE(ZIGBEE_TAG, "OTA callback status: 0x%x", ota->info.status);
        return ESP_FAIL;
    }

    switch (ota->upgrade_status) {
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
            ota_update_partition = esp_ota_get_next_update_partition(NULL);
            if (ota_update_partition == NULL) return ESP_ERR_NOT_FOUND;
            ota_stream_offset = 0;
            ota_expected_size = 0;
            ota_metadata_valid = false;
            ota_sub_element_remaining = 0;
            ota_sub_element_header_bytes = 0;
            ota_update_image_element = false;
            ESP_RETURN_ON_ERROR(ota_validate_callback_header(&ota->ota_header),
                                ZIGBEE_TAG, "Invalid OTA callback header");
            ESP_RETURN_ON_ERROR(
                esp_ota_begin(ota_update_partition, OTA_SIZE_UNKNOWN, &ota_update_handle),
                ZIGBEE_TAG, "esp_ota_begin failed"
            );
            ota_update_in_progress = true;
            ESP_LOGI(ZIGBEE_TAG,
                     "OTA started; validating metadata and OTA application stream");
            break;

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
            if (!ota_update_in_progress || ota->payload == NULL || ota->payload_size == 0) {
                return ESP_ERR_INVALID_STATE;
            }
            ESP_RETURN_ON_ERROR(ota_validate_callback_header(&ota->ota_header),
                                ZIGBEE_TAG, "Invalid OTA callback header");
            ESP_RETURN_ON_ERROR(ota_write_stream(ota->payload, ota->payload_size),
                                ZIGBEE_TAG, "Failed to process OTA data block");
            break;

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
            ESP_RETURN_ON_ERROR(ota_validate_callback_header(&ota->ota_header),
                                ZIGBEE_TAG, "Invalid OTA callback header");
            if (!ota_update_in_progress || !ota_metadata_valid ||
                ota_sub_element_header_bytes != 0 ||
                ota_sub_element_remaining != 0 || !ota_update_image_element ||
                ota_stream_offset != ota_expected_size) {
                ESP_LOGE(ZIGBEE_TAG,
                         "OTA image stream is incomplete: metadata=%d, offset=%" PRIu32
                         "/%" PRIu32 ", subelement_header_bytes=%u, "
                         "subelement_remaining=%" PRIu32 ", image_element=%d",
                         ota_metadata_valid, ota_stream_offset, ota_expected_size,
                         (unsigned)ota_sub_element_header_bytes,
                         ota_sub_element_remaining, ota_update_image_element);
                return ESP_ERR_INVALID_SIZE;
            }
            break;

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY: {
            if (!ota_update_in_progress) return ESP_ERR_INVALID_STATE;
            esp_err_t err = esp_ota_end(ota_update_handle);
            ota_update_in_progress = false;
            if (err != ESP_OK) return err;
            ESP_RETURN_ON_ERROR(esp_ota_set_boot_partition(ota_update_partition),
                                ZIGBEE_TAG, "Failed to select OTA boot partition");
            ESP_LOGI(ZIGBEE_TAG, "OTA image verified; reboot scheduled after Zigbee finishes");
            break;
        }

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
            ESP_LOGI(ZIGBEE_TAG, "OTA complete; restarting");
            esp_restart();
            break;

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ABORT:
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ERROR:
            if (ota_update_in_progress) {
                esp_ota_abort(ota_update_handle);
                ota_update_in_progress = false;
            }
            ESP_LOGW(ZIGBEE_TAG, "OTA cancelled or failed");
            break;

        default:
            break;
    }

    return ESP_OK;
}

/* Keep the display unit in sync with the standard Thermostat UI Config ZCL
 * attribute (cluster 0x0204, TemperatureDisplayMode attribute 0x0000). */
static volatile bool display_fahrenheit;
static volatile bool display_data_dirty;
static volatile int16_t temperature_calibration_tenths = 0;
static volatile int16_t humidity_calibration_percent = 0;

static void load_sensor_calibration(void)
{
    nvs_handle_t handle;
    int16_t value;
    if (nvs_open("calibration", NVS_READONLY, &handle) == ESP_OK) {
        if (nvs_get_i16(handle, "temp_tenths", &value) == ESP_OK && value >= -50 && value <= 50)
            temperature_calibration_tenths = value;
        if (nvs_get_i16(handle, "humidity", &value) == ESP_OK && value >= -30 && value <= 30)
            humidity_calibration_percent = value;
        nvs_close(handle);
    }
}

static void save_sensor_calibration(const char *key, int16_t value)
{
    nvs_handle_t handle;
    if (nvs_open("calibration", NVS_READWRITE, &handle) == ESP_OK) {
        esp_err_t err = nvs_set_i16(handle, key, value);
        if (err == ESP_OK) err = nvs_commit(handle);
        if (err != ESP_OK)
            ESP_LOGW(ZIGBEE_TAG, "Could not persist calibration %s: %s", key, esp_err_to_name(err));
        nvs_close(handle);
    }
}

static void load_temperature_display_mode(void)
{
    nvs_handle_t handle;
    uint8_t mode = 0;
    if (nvs_open("display", NVS_READONLY, &handle) == ESP_OK) {
        if (nvs_get_u8(handle, "temp_unit", &mode) == ESP_OK && mode <= 1U) {
            display_fahrenheit = (mode == 1U);
        }
        nvs_close(handle);
    }
}

static void save_temperature_display_mode(uint8_t mode)
{
    nvs_handle_t handle;
    if (nvs_open("display", NVS_READWRITE, &handle) == ESP_OK) {
        esp_err_t err = nvs_set_u8(handle, "temp_unit", mode);
        if (err == ESP_OK) err = nvs_commit(handle);
        if (err != ESP_OK) {
            ESP_LOGW(ZIGBEE_TAG, "Could not persist temperature unit: %s",
                     esp_err_to_name(err));
        }
        nvs_close(handle);
    }
}

static esp_err_t zigbee_action_handler(
    esp_zb_core_action_callback_id_t callback_id,
    const void *message
)
{
    if (callback_id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID && message != NULL) {
        const esp_zb_zcl_set_attr_value_message_t *attr_message = message;
        if (attr_message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS &&
            attr_message->info.dst_endpoint == ZIGBEE_ENDPOINT &&
            attr_message->info.cluster == SENSOR_CALIBRATION_CLUSTER_ID &&
            attr_message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_S16 &&
            attr_message->attribute.data.value != NULL &&
            attr_message->attribute.data.size >= sizeof(int16_t)) {
            const int16_t value = *(const int16_t *)attr_message->attribute.data.value;
            if (attr_message->attribute.id == SENSOR_CALIBRATION_TEMP_ATTR_ID && value >= -50 && value <= 50) {
                temperature_calibration_tenths = value;
                save_sensor_calibration("temp_tenths", value);
                display_data_dirty = true;
                ESP_LOGI(ZIGBEE_TAG, "Temperature calibration set to %.1f C", value / 10.0f);
            } else if (attr_message->attribute.id == SENSOR_CALIBRATION_HUM_ATTR_ID && value >= -30 && value <= 30) {
                humidity_calibration_percent = value;
                save_sensor_calibration("humidity", value);
                display_data_dirty = true;
                ESP_LOGI(ZIGBEE_TAG, "Humidity calibration set to %d %%", value);
            }
        }
        if (attr_message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS &&
            attr_message->info.dst_endpoint == ZIGBEE_ENDPOINT &&
            attr_message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT_UI_CONFIG &&
            attr_message->attribute.id ==
                ESP_ZB_ZCL_ATTR_THERMOSTAT_UI_CONFIG_TEMPERATURE_DISPLAY_MODE_ID &&
            attr_message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM &&
            attr_message->attribute.data.value != NULL &&
            attr_message->attribute.data.size >= sizeof(uint8_t)) {
            const uint8_t mode = *(const uint8_t *)attr_message->attribute.data.value;
            if (mode <= 1U) {
                display_fahrenheit = (mode == 1U);
                save_temperature_display_mode(mode);
                display_data_dirty = true;
                ESP_LOGI(ZIGBEE_TAG, "Temperature display unit set to %s",
                         display_fahrenheit ? "Fahrenheit" : "Celsius");
            }
        }
        return ESP_OK;
    }

    return zigbee_ota_action_handler(callback_id, message);
}

static esp_err_t zigbee_create_device(void)
{
    /*
     * --------------------------------------------------------
     * Basic cluster
     * --------------------------------------------------------
     */

    esp_zb_basic_cluster_cfg_t basic_cluster_cfg = {
        .zcl_version =
            ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,

        // Device is powered from a single-phase mains supply via USB.
        .power_source =
            ESP_ZB_ZCL_BASIC_POWER_SOURCE_MAINS_SINGLE_PHASE,
    };



    esp_zb_attribute_list_t *basic_cluster =
        esp_zb_basic_cluster_create(
            &basic_cluster_cfg
        );


    if (basic_cluster == NULL) {

        ESP_LOGE(
            ZIGBEE_TAG,
            "Failed to create Basic cluster"
        );

        return ESP_FAIL;
    }


    /*
     * Manufacturer name
     */

    ESP_ERROR_CHECK(
        esp_zb_basic_cluster_add_attr(
            basic_cluster,

            ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,

            (void *)ZIGBEE_MANUFACTURER_NAME
        )
    );

    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(
        basic_cluster,
        ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID,
        &zigbee_application_version
    ));


    /*
     * Model identifier
     */

    ESP_ERROR_CHECK(
        esp_zb_basic_cluster_add_attr(
            basic_cluster,

            ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,

            (void *)ZIGBEE_MODEL_IDENTIFIER
        )
    );


    /*
     * --------------------------------------------------------
     * Cluster list
     * --------------------------------------------------------
     */

    esp_zb_cluster_list_t *cluster_list =
        esp_zb_zcl_cluster_list_create();


    if (cluster_list == NULL) {

        ESP_LOGE(
            ZIGBEE_TAG,
            "Failed to create cluster list"
        );

        return ESP_FAIL;
    }


    ESP_ERROR_CHECK(
        esp_zb_cluster_list_add_basic_cluster(
            cluster_list,

            basic_cluster,

            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
        )
    );


    /*
     * --------------------------------------------------------
     * Temperature Measurement cluster 0x0402
     * --------------------------------------------------------
     * Zigbee stores temperature in 0.01 degC.
     */

    esp_zb_temperature_meas_cluster_cfg_t temperature_cluster_cfg = {
        .measured_value = 0,
        .min_value = -4000,
        .max_value = 8500,
    };

    esp_zb_attribute_list_t *temperature_cluster =
        esp_zb_temperature_meas_cluster_create(
            &temperature_cluster_cfg
        );

    if (temperature_cluster == NULL) {
        ESP_LOGE(
            ZIGBEE_TAG,
            "Failed to create Temperature Measurement cluster"
        );
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(
        esp_zb_cluster_list_add_temperature_meas_cluster(
            cluster_list,
            temperature_cluster,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
        )
    );


    /*
     * --------------------------------------------------------
     * Relative Humidity Measurement cluster 0x0405
     * --------------------------------------------------------
     * Zigbee stores humidity in 0.01 percent RH.
     */

    esp_zb_humidity_meas_cluster_cfg_t humidity_cluster_cfg = {
        .measured_value = 0,
        .min_value = 0,
        .max_value = 10000,
    };

    esp_zb_attribute_list_t *humidity_cluster =
        esp_zb_humidity_meas_cluster_create(
            &humidity_cluster_cfg
        );

    if (humidity_cluster == NULL) {
        ESP_LOGE(
            ZIGBEE_TAG,
            "Failed to create Relative Humidity Measurement cluster"
        );
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(
        esp_zb_cluster_list_add_humidity_meas_cluster(
            cluster_list,
            humidity_cluster,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
        )
    );

    /* Standard Zigbee UI setting, writable by Zigbee2MQTT. Default: Celsius. */
    esp_zb_thermostat_ui_config_cluster_cfg_t ui_config = {0};
    ui_config.temperature_display_mode = 0;
    esp_zb_attribute_list_t *ui_config_cluster =
        esp_zb_thermostat_ui_config_cluster_create(&ui_config);
    if (ui_config_cluster == NULL) {
        ESP_LOGE(ZIGBEE_TAG, "Failed to create Thermostat UI Config cluster");
        return ESP_FAIL;
    }
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_thermostat_ui_config_cluster(
        cluster_list, ui_config_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    /* Writable device-side calibration attributes. Temperature is in 0.1 C;
     * humidity is an absolute offset in percentage points. */
    esp_zb_attribute_list_t *calibration_cluster =
        esp_zb_zcl_attr_list_create(SENSOR_CALIBRATION_CLUSTER_ID);
    if (calibration_cluster == NULL) return ESP_FAIL;
    ESP_ERROR_CHECK(esp_zb_custom_cluster_add_custom_attr(
        calibration_cluster, SENSOR_CALIBRATION_TEMP_ATTR_ID, ESP_ZB_ZCL_ATTR_TYPE_S16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, (void *)&temperature_calibration_tenths));
    ESP_ERROR_CHECK(esp_zb_custom_cluster_add_custom_attr(
        calibration_cluster, SENSOR_CALIBRATION_HUM_ATTR_ID, ESP_ZB_ZCL_ATTR_TYPE_S16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, (void *)&humidity_calibration_percent));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_custom_cluster(
        cluster_list, calibration_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));


    /*
     * --------------------------------------------------------
     * Illuminance Measurement cluster 0x0400
     * The ZCL measured_value uses logarithmic lux encoding.
     */
    esp_zb_illuminance_meas_cluster_cfg_t illuminance_cluster_cfg = {
        .measured_value = 0,
        .min_value = 0,
        .max_value = 0xFFFE,
    };

    esp_zb_attribute_list_t *illuminance_cluster =
        esp_zb_illuminance_meas_cluster_create(&illuminance_cluster_cfg);

    if (illuminance_cluster == NULL) {
        ESP_LOGE(ZIGBEE_TAG, "Failed to create Illuminance Measurement cluster");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(
        esp_zb_cluster_list_add_illuminance_meas_cluster(
            cluster_list,
            illuminance_cluster,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
        )
    );


    /*
     * --------------------------------------------------------
     * Carbon Dioxide Measurement cluster 0x040D
     * --------------------------------------------------------
     * Zigbee CO2 measured value is a floating-point value in ppm.
     *
     * Keep MinMeasuredValue / MaxMeasuredValue valid because
     * esp_zb_zcl_set_attribute_val() validates the measured value
     * against these bounds for this cluster.
     */
    esp_zb_carbon_dioxide_measurement_cluster_cfg_t co2_cluster_cfg = {
        .measured_value = 0.0f,
        .min_measured_value = 0.0f,
        .max_measured_value = 0.01f,
    };

    esp_zb_attribute_list_t *co2_cluster =
        esp_zb_carbon_dioxide_measurement_cluster_create(
            &co2_cluster_cfg
        );

    if (co2_cluster == NULL) {
        ESP_LOGE(
            ZIGBEE_TAG,
            "Failed to create Carbon Dioxide Measurement cluster"
        );
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(
        esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(
            cluster_list,
            co2_cluster,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
        )
    );

    /* OTA Upgrade client cluster (ZCL cluster 0x0019). */
    esp_zb_ota_cluster_cfg_t ota_cluster_cfg = {
        .ota_upgrade_file_version = OTA_FILE_VERSION,
        .ota_upgrade_manufacturer = OTA_MANUFACTURER_CODE,
        .ota_upgrade_image_type = OTA_IMAGE_TYPE,
        .ota_min_block_reque = ESP_ZB_OTA_UPGRADE_MIN_BLOCK_PERIOD_DEF_VALUE,
        .ota_upgrade_file_offset = 0,
        .ota_upgrade_downloaded_file_ver =
            ESP_ZB_ZCL_OTA_UPGRADE_DOWNLOADED_FILE_VERSION_DEF_VALUE,
        .ota_upgrade_server_id = ESP_ZB_ZCL_OTA_UPGRADE_SERVER_DEF_VALUE,
        .ota_image_upgrade_status = ESP_ZB_ZCL_OTA_UPGRADE_IMAGE_STATUS_DEF_VALUE,
    };
    esp_zb_attribute_list_t *ota_cluster = esp_zb_ota_cluster_create(&ota_cluster_cfg);
    if (ota_cluster == NULL) {
        ESP_LOGE(ZIGBEE_TAG, "Failed to create OTA Upgrade cluster");
        return ESP_FAIL;
    }
    esp_zb_zcl_ota_upgrade_client_variable_t ota_client_cfg = {
        .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
        .hw_version = 1,
        .max_data_size = OTA_MAX_DATA_SIZE,
    };
    ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(
        ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, &ota_client_cfg));
    uint16_t ota_server_addr = ESP_ZB_ZCL_OTA_UPGRADE_SERVER_ADDR_DEF_VALUE;
    uint8_t ota_server_endpoint = ESP_ZB_ZCL_OTA_UPGRADE_SERVER_ENDPOINT_DEF_VALUE;
    ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(
        ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ADDR_ID, &ota_server_addr));
    ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(
        ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ENDPOINT_ID, &ota_server_endpoint));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_ota_cluster(
        cluster_list, ota_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));


    /*
     * --------------------------------------------------------
     * Endpoint
     * --------------------------------------------------------
     */

    esp_zb_ep_list_t *ep_list =
        esp_zb_ep_list_create();


    if (ep_list == NULL) {

        ESP_LOGE(
            ZIGBEE_TAG,
            "Failed to create endpoint list"
        );

        return ESP_FAIL;
    }


    esp_zb_endpoint_config_t endpoint_config = {

        .endpoint =
            ZIGBEE_ENDPOINT,

        .app_profile_id =
            ESP_ZB_AF_HA_PROFILE_ID,

        /*
         * Пока используем Generic HA device.
         *
         * Позже, когда добавим реальные измерительные
         * кластеры, здесь можно будет использовать
         * подходящий device ID.
         */

        .app_device_id =
            ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,

        .app_device_version =
            0,
    };


    ESP_ERROR_CHECK(
        esp_zb_ep_list_add_ep(
            ep_list,

            cluster_list,

            endpoint_config
        )
    );


    /*
     * Register endpoint
     */

    ESP_ERROR_CHECK(
        esp_zb_device_register(
            ep_list
        )
    );

    esp_zb_core_action_handler_register(zigbee_action_handler);


    ESP_LOGI(
        ZIGBEE_TAG,
        "Zigbee endpoint %d created",
        ZIGBEE_ENDPOINT
    );


    ESP_LOGI(
        ZIGBEE_TAG,
        "Manufacturer: Custom"
    );

    ESP_LOGI(
        ZIGBEE_TAG,
        "Model: ESP32H2_CO2"
    );


    return ESP_OK;
}


// ============================================================
// Zigbee task
// ============================================================

static void zigbee_task(void *arg)
{
    ESP_LOGI(
        ZIGBEE_TAG,
        "Starting Zigbee Router"
    );

    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,

        .install_code_policy = false,

        .nwk_cfg.zczr_cfg = {
            .max_children = ZIGBEE_MAX_CHILDREN,
        },
    };

    esp_zb_init(&zb_nwk_cfg);

    ESP_LOGI(
        ZIGBEE_TAG,
        "Zigbee stack initialized"
    );

    ESP_ERROR_CHECK(
        esp_zb_set_primary_network_channel_set(
            ZIGBEE_PRIMARY_CHANNEL_MASK
        )
    );

    ESP_LOGI(
        ZIGBEE_TAG,
        "Primary channel mask: 0x%08lx",
        (unsigned long)ZIGBEE_PRIMARY_CHANNEL_MASK
    );

    ESP_ERROR_CHECK(
        zigbee_create_device()
    );

    /* Keep the advertised ZCL setting aligned with the unit retained in NVS. */
    uint8_t temperature_display_mode = display_fahrenheit ? 1U : 0U;
    const esp_zb_zcl_status_t unit_status = esp_zb_zcl_set_attribute_val(
        ZIGBEE_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT_UI_CONFIG,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_THERMOSTAT_UI_CONFIG_TEMPERATURE_DISPLAY_MODE_ID,
        &temperature_display_mode,
        false
    );
    if (unit_status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(ZIGBEE_TAG, "Could not initialize temperature display mode: 0x%02x",
                 unit_status);
    }

    /*
     * Working join workaround:
     * the coordinator was seen with very low LQI during initial
     * network discovery, so allow join candidates down to LQI 0.
     */
    esp_zb_secur_network_min_join_lqi_set(0);

    ESP_LOGI(
        ZIGBEE_TAG,
        "Minimum join LQI: 0"
    );

    ESP_ERROR_CHECK(
        esp_zb_start(false)
    );

    zigbee_stack_started = true;

    ESP_LOGI(
        ZIGBEE_TAG,
        "Zigbee Router stack started"
    );

    esp_zb_stack_main_loop();

    vTaskDelete(NULL);
}


// ============================================================
// Button
// ============================================================

static void zigbee_button_init(void)
{
    gpio_config_t button_config = {

        .pin_bit_mask =
            1ULL << ZIGBEE_BUTTON_GPIO,

        .mode =
            GPIO_MODE_INPUT,

        // The button's bias resistor is installed on the PCB.
        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE,
    };


    ESP_ERROR_CHECK(
        gpio_config(
            &button_config
        )
    );
}


static bool zigbee_button_check_long_press(void)
{
    static bool was_pressed = false;
    static bool long_press_triggered = false;

    static TickType_t press_start = 0;


    bool pressed =
        (gpio_get_level(
            ZIGBEE_BUTTON_GPIO
        ) == 0);


    /*
     * New press
     */

    if (pressed && !was_pressed) {

        press_start =
            xTaskGetTickCount();

        was_pressed = true;

        long_press_triggered = false;
    }


    /*
     * Button held
     */

    if (
        pressed &&
        was_pressed &&
        !long_press_triggered
    ) {

        TickType_t elapsed =
            xTaskGetTickCount() -
            press_start;


        if (
            elapsed >=
            pdMS_TO_TICKS(
                ZIGBEE_LONG_PRESS_MS
            )
        ) {

            long_press_triggered = true;


            ESP_LOGI(
                ZIGBEE_TAG,
                "Long press detected - Zigbee factory reset"
            );

            /*
             * Erase Zigbee persistent data and restart the device.
             * After reboot the device becomes factory_new=YES
             * and the signal handler starts network steering.
             *
             * esp_zb_factory_reset() is a void API and performs
             * the reboot itself.
             */
            ESP_LOGW(
                ZIGBEE_TAG,
                "Erasing Zigbee NVRAM and restarting..."
            );

            esp_zb_factory_reset();

            /*
             * Normally execution never reaches here because
             * esp_zb_factory_reset() restarts the device.
             */
            return true;
        }
    }


    /*
     * Button released
     */

    if (!pressed && was_pressed) {

        was_pressed = false;

        long_press_triggered = false;
    }


    return false;
}


// ============================================================
// Button task
// ============================================================

static void zigbee_button_task(void *arg)
{
    while (1) {

        zigbee_button_check_long_press();

        vTaskDelay(
            pdMS_TO_TICKS(20)
        );
    }
}


// ============================================================
// Сглаженные растровые цифры Poetsen One для показания CO2.
// ============================================================

// ============================================================
// GC9A01 graphics: RGB565, rendered in small DMA stripes
// ============================================================

static int display_region_x;
static int display_region_width;
static int display_stripe_y;
static int display_stripe_height;
static uint16_t display_co2;
static float display_temperature;
static float display_humidity;
static bool display_measurement_valid;
static bool display_previous_fahrenheit;
static bool display_wifi_blink_on = true;
static uint16_t display_previous_co2;
static float display_previous_temperature;
static float display_previous_humidity;
static bool display_previous_co2_valid;
static bool display_previous_environment_valid;
static bool display_previous_wifi_valid;
static bool display_previous_wifi_visible;
static bool display_force_full_refresh = true;

#define DISPLAY_RETAINED_MAGIC 0xC0425EEDUL

typedef struct {
    uint32_t magic;
    uint16_t co2;
    float temperature;
    float humidity;
} display_retained_measurement_t;

static RTC_NOINIT_ATTR display_retained_measurement_t display_retained_measurement;

#define GAUGE_CENTER_X       120
#define GAUGE_CENTER_Y       120
#define GAUGE_RADIUS         100
#define GAUGE_MIN_CO2        400
#define GAUGE_MAX_CO2        2000
#define GAUGE_THICKNESS      12
#define DIGIT_SLOT_WIDTH     54
#define DIGIT_SLOT_GAP       2

#define COLOR_BLACK          0x0000
#define COLOR_WHITE          0xFFFF
#define COLOR_GREEN          0x07E0
#define COLOR_YELLOW         0xFFE0
#define COLOR_ORANGE         0xFD20
#define COLOR_RED            0xF800
#define COLOR_BLUE           0x001F

static void display_pixel(int x, int y, uint16_t color)
{
    if (x < display_region_x || x >= display_region_x + display_region_width ||
        y < display_stripe_y || y >= display_stripe_y + display_stripe_height ||
        x < 0 || x >= DISPLAY_WIDTH || y < 0 || y >= DISPLAY_HEIGHT) {
        return;
    }
    const size_t index = (size_t)(y - display_stripe_y) * display_region_width +
                         (size_t)(x - display_region_x);
    display_buffer[index] = color;
}

static void display_blend_pixel(int x, int y, uint16_t color, uint8_t coverage)
{
    if (coverage >= 15U) {
        display_pixel(x, y, color);
        return;
    }
    if (coverage == 0U || x < display_region_x || x >= display_region_x + display_region_width ||
        y < display_stripe_y || y >= display_stripe_y + display_stripe_height ||
        x < 0 || x >= DISPLAY_WIDTH || y < 0 || y >= DISPLAY_HEIGHT) return;

    const size_t index = (size_t)(y - display_stripe_y) * display_region_width +
                         (size_t)(x - display_region_x);
    const uint16_t background = display_buffer[index];
    const unsigned inverse = 15U - coverage;
    const uint16_t red = (uint16_t)((((background >> 11) & 31U) * inverse +
                                     ((color >> 11) & 31U) * coverage + 7U) / 15U);
    const uint16_t green = (uint16_t)((((background >> 5) & 63U) * inverse +
                                       ((color >> 5) & 63U) * coverage + 7U) / 15U);
    const uint16_t blue = (uint16_t)(((background & 31U) * inverse +
                                      (color & 31U) * coverage + 7U) / 15U);
    display_buffer[index] = (uint16_t)((red << 11) | (green << 5) | blue);
}

static uint16_t co2_color(uint16_t co2)
{
    if (co2 < 800) return COLOR_GREEN;
    if (co2 < 1000) return COLOR_YELLOW;
    if (co2 < 1500) return COLOR_ORANGE;
    return COLOR_RED;
}

static uint16_t temperature_color(float temperature, bool fahrenheit)
{
    const float cold_limit = fahrenheit ? 64.4f : 18.0f;
    const float warm_limit = fahrenheit ? 78.8f : 26.0f;
    if (temperature < cold_limit) return COLOR_BLUE;
    if (temperature <= warm_limit) return COLOR_GREEN;
    return COLOR_RED;
}

static uint16_t humidity_color(float humidity)
{
    if (humidity < 30.0f) return COLOR_RED;
    if (humidity < 40.0f) return COLOR_YELLOW;
    if (humidity <= 60.0f) return COLOR_GREEN;
    if (humidity <= 70.0f) return COLOR_YELLOW;
    return COLOR_RED;
}

static void draw_digit_colored(int x, int y, uint8_t digit, uint16_t color)
{
    const display_asset_t *glyph = &display_assets[ASSET_BIG_0 + digit];
    for (int row = 0; row < glyph->height; row++) {
        for (int col = 0; col < glyph->width; col++) {
            const size_t pixel_index = (size_t)row * glyph->width + (size_t)col;
            const uint8_t packed = glyph->coverage[pixel_index >> 1];
            const uint8_t level = (pixel_index & 1U) ? (packed & 0x0FU) : (packed >> 4);
            if (level == 0) continue;

            const uint8_t coverage = (uint8_t)(level * 17U);
            const uint16_t r = (uint16_t)((((color >> 11) & 0x1FU) * coverage + 127U) / 255U);
            const uint16_t g = (uint16_t)((((color >> 5) & 0x3FU) * coverage + 127U) / 255U);
            const uint16_t b = (uint16_t)(((color & 0x1FU) * coverage + 127U) / 255U);
            display_pixel(x + col, y + row, (uint16_t)((r << 11) | (g << 5) | b));
        }
    }
}

static void draw_ppm_label(void)
{
    const display_asset_t *asset = &display_assets[ASSET_PPM];
    const int x0 = (DISPLAY_WIDTH - asset->width) / 2;
    const int y0 = 170;
    for (int y = 0; y < asset->height; y++) {
        for (int x = 0; x < asset->width; x++) {
            const size_t pixel = (size_t)y * asset->width + (size_t)x;
            const uint8_t packed = asset->coverage[pixel >> 1];
            const uint8_t coverage = (pixel & 1U) ? (packed & 0x0FU) : (packed >> 4);
            if (coverage == 0) continue;
            const uint16_t r = (uint16_t)((31U * coverage + 7U) / 15U);
            const uint16_t g = (uint16_t)((63U * coverage + 7U) / 15U);
            const uint16_t b = (uint16_t)((31U * coverage + 7U) / 15U);
            display_pixel(x0 + x, y0 + y, (uint16_t)((r << 11) | (g << 5) | b));
        }
    }
}

static void draw_co2_label(void)
{
    const display_asset_t *asset = &display_assets[ASSET_CO2];
    for (int y = 0; y < asset->height; y++) {
        for (int x = 0; x < asset->width; x++) {
            const size_t pixel = (size_t)y * asset->width + (size_t)x;
            const uint8_t packed = asset->coverage[pixel >> 1];
            const uint8_t coverage = (pixel & 1U) ? (packed & 0x0FU) : (packed >> 4);
            if (coverage == 0) continue;
            const uint16_t r = (uint16_t)((31U * coverage + 7U) / 15U);
            const uint16_t g = (uint16_t)((63U * coverage + 7U) / 15U);
            const uint16_t b = (uint16_t)((31U * coverage + 7U) / 15U);
            display_pixel(104 + x, 49 + y, (uint16_t)((r << 11) | (g << 5) | b));
        }
    }
}

static void draw_gauge_arc(void)
{
    const float pi = 3.14159265358979323846f;
    const int samples_per_axis = 4;
    const int sample_count = samples_per_axis * samples_per_axis;
    const float half_thickness = (float)GAUGE_THICKNESS * 0.5f;
    const int arc_extent = GAUGE_RADIUS + (int)ceilf(half_thickness);
    const int min_x = display_region_x > GAUGE_CENTER_X - arc_extent ?
                      display_region_x : GAUGE_CENTER_X - arc_extent;
    const int max_x = display_region_x + display_region_width - 1 < GAUGE_CENTER_X + arc_extent ?
                      display_region_x + display_region_width - 1 : GAUGE_CENTER_X + arc_extent;
    const int min_y = display_stripe_y > GAUGE_CENTER_Y - arc_extent ?
                      display_stripe_y : GAUGE_CENTER_Y - arc_extent;
    const int max_y = display_stripe_y + display_stripe_height - 1 <
                      GAUGE_CENTER_Y + (int)ceilf(half_thickness) ?
                      display_stripe_y + display_stripe_height - 1 :
                      GAUGE_CENTER_Y + (int)ceilf(half_thickness);

    for (int y = min_y; y <= max_y; y++) {
        for (int x = min_x; x <= max_x; x++) {
            unsigned red_sum = 0;
            unsigned green_sum = 0;
            unsigned blue_sum = 0;

            for (int sy = 0; sy < samples_per_axis; sy++) {
                const float sample_y = (float)y + ((float)sy + 0.5f) / samples_per_axis;
                const float dy = (float)GAUGE_CENTER_Y - sample_y;
                for (int sx = 0; sx < samples_per_axis; sx++) {
                    const float sample_x = (float)x + ((float)sx + 0.5f) / samples_per_axis;
                    const float dx = sample_x - (float)GAUGE_CENTER_X;
                    float angle;
                    float distance;
                    const bool on_upper_half = dy >= 0.0f;

                    if (on_upper_half) {
                        const float radius = sqrtf(dx * dx + dy * dy);
                        distance = fabsf(radius - (float)GAUGE_RADIUS);
                    } else {
                        const float end_x = dx >= 0.0f ? (float)GAUGE_RADIUS : -(float)GAUGE_RADIUS;
                        const float end_dx = dx - end_x;
                        distance = sqrtf(end_dx * end_dx + dy * dy);
                    }

                    if (distance > half_thickness) continue;

                    angle = on_upper_half ? atan2f(dy, dx) : (dx >= 0.0f ? 0.0f : pi);

                    const float co2_position = (pi - angle) *
                        (float)(GAUGE_MAX_CO2 - GAUGE_MIN_CO2) / pi + (float)GAUGE_MIN_CO2;
                    const uint16_t color = co2_color((uint16_t)(co2_position + 0.5f));
                    red_sum += (color >> 11) & 0x1FU;
                    green_sum += (color >> 5) & 0x3FU;
                    blue_sum += color & 0x1FU;
                }
            }

            if (red_sum || green_sum || blue_sum) {
                const uint16_t red = (uint16_t)((red_sum + sample_count / 2U) / sample_count);
                const uint16_t green = (uint16_t)((green_sum + sample_count / 2U) / sample_count);
                const uint16_t blue = (uint16_t)((blue_sum + sample_count / 2U) / sample_count);
                display_pixel(x, y, (uint16_t)((red << 11) | (green << 5) | blue));
            }
        }
        // Yield during the wide, supersampled arc scan as well as between DMA
        // stripes. A single stripe can intersect enough of the arc to run for
        // several seconds on the ESP32-H2.
        if (((y - min_y) & 3) == 3) vTaskDelay(1);
    }
}

static void draw_co2_value(uint16_t co2)
{
    char value[8];
    snprintf(value, sizeof(value), "%u", (unsigned)co2);
    const size_t digit_count = strlen(value);
    const int total_width = (int)digit_count * DIGIT_SLOT_WIDTH +
                            ((int)digit_count - 1) * DIGIT_SLOT_GAP;
    const int start_x = (DISPLAY_WIDTH - total_width) / 2;
    const int start_y = 95;
    const uint16_t color = co2_color(co2);

    for (size_t i = 0; i < digit_count; i++) {
        const uint8_t digit = (uint8_t)(value[i] - '0');
        const int slot_x = start_x + (int)i * (DIGIT_SLOT_WIDTH + DIGIT_SLOT_GAP);
        const int digit_x = slot_x + (DIGIT_SLOT_WIDTH - display_assets[ASSET_BIG_0 + digit].width) / 2;
        draw_digit_colored(digit_x, start_y, digit, color);
    }
}

static void draw_measurement(uint16_t co2, float temperature, float humidity)
{
    display_co2 = co2;
    display_temperature = temperature;
    display_humidity = humidity;
    display_measurement_valid = true;
    display_data_dirty = true;

    display_retained_measurement.magic = 0;
    display_retained_measurement.co2 = co2;
    display_retained_measurement.temperature = temperature;
    display_retained_measurement.humidity = humidity;
    display_retained_measurement.magic = DISPLAY_RETAINED_MAGIC;
}

static int small_text_asset(char ch)
{
    if (ch >= '0' && ch <= '9') return ASSET_SMALL_0 + (ch - '0');
    if (ch == 'o') return ASSET_GLYPH_O;
    if (ch == 'C') return ASSET_GLYPH_C;
    if (ch == 'F') return ASSET_GLYPH_F;
    if (ch == '%') return ASSET_GLYPH_PERCENT;
    if (ch == '.') return ASSET_GLYPH_DOT;
    if (ch == '-') return ASSET_GLYPH_MINUS;
    return -1;
}

static bool small_glyph_bounds(const display_asset_t *glyph, int *left, int *right)
{
    *left = glyph->width;
    *right = -1;
    for (int row = 0; row < glyph->height; row++) {
        for (int col = 0; col < glyph->width; col++) {
            const size_t pixel = (size_t)row * glyph->width + (size_t)col;
            const uint8_t packed = glyph->coverage[pixel >> 1];
            const uint8_t level = (pixel & 1U) ? (packed & 0x0FU) : (packed >> 4);
            if (level == 0) continue;
            if (col < *left) *left = col;
            if (col > *right) *right = col;
        }
    }
    return *right >= *left;
}

static void draw_small_glyph(const display_asset_t *glyph, int x, int y,
                             int left, int right, uint16_t color)
{
    for (int row = 0; row < glyph->height; row++) {
        for (int col = left; col <= right; col++) {
            const size_t pixel = (size_t)row * glyph->width + (size_t)col;
            const uint8_t packed = glyph->coverage[pixel >> 1];
            const uint8_t level = (pixel & 1U) ? (packed & 0x0FU) : (packed >> 4);
            if (level == 0) continue;
            const uint8_t coverage = (uint8_t)(level * 17U);
            const uint16_t r = (uint16_t)((((color >> 11) & 0x1FU) * coverage + 127U) / 255U);
            const uint16_t g = (uint16_t)((((color >> 5) & 0x3FU) * coverage + 127U) / 255U);
            const uint16_t b = (uint16_t)(((color & 0x1FU) * coverage + 127U) / 255U);
            display_pixel(x + col - left, y + row, (uint16_t)((r << 11) | (g << 5) | b));
        }
    }
}

static void draw_small_text_centered(const char *text, int center_x, int y, uint16_t color)
{
    const size_t length = strlen(text);
    int widths[16];
    int bounds_left[16];
    size_t count = 0;
    int total_width = 0;
    const int gap = 1;

    for (size_t i = 0; i < length && count < 16; i++) {
        const int asset_id = small_text_asset(text[i]);
        if (asset_id < 0) continue;
        const display_asset_t *glyph = &display_assets[asset_id];
        int right;
        if (!small_glyph_bounds(glyph, &bounds_left[count], &right)) continue;
        widths[count] = right - bounds_left[count] + 1;
        total_width += widths[count] + (count ? gap : 0);
        count++;
    }

    int x = center_x - total_width / 2;
    size_t glyph_index = 0;
    for (size_t i = 0; i < length && glyph_index < count; i++) {
        const int asset_id = small_text_asset(text[i]);
        if (asset_id < 0) continue;
        const display_asset_t *glyph = &display_assets[asset_id];
        int left, right;
        if (!small_glyph_bounds(glyph, &left, &right)) continue;
        draw_small_glyph(glyph, x, y, left, right, color);
        x += widths[glyph_index] + gap;
        glyph_index++;
    }
}

static void draw_thermometer_icon(int x, int y, uint16_t color)
{
    const display_asset_t *asset = &display_assets[ASSET_THERMO];
    for (int py=0; py<asset->height; py++) for (int px=0; px<asset->width; px++) {
        const size_t i=(size_t)py*asset->width+px; const uint8_t p=asset->coverage[i>>1];
        const uint8_t a=(i&1U)?(p&15U):(p>>4); if (!a) continue;
        const uint16_t r=(uint16_t)((((color>>11)&31U)*a+7U)/15U);
        const uint16_t g=(uint16_t)((((color>>5)&63U)*a+7U)/15U);
        const uint16_t b=(uint16_t)(((color&31U)*a+7U)/15U);
        display_pixel(x+px,y+py,(uint16_t)((r<<11)|(g<<5)|b));
    }
}

static void draw_water_icon(int x, int y, uint16_t color)
{
    const display_asset_t *asset = &display_assets[ASSET_WATER];
    for (int py=0; py<asset->height; py++) for (int px=0; px<asset->width; px++) {
        const size_t i=(size_t)py*asset->width+px; const uint8_t p=asset->coverage[i>>1];
        const uint8_t a=(i&1U)?(p&15U):(p>>4); if (!a) continue;
        const uint16_t r=(uint16_t)((((color>>11)&31U)*a+7U)/15U);
        const uint16_t g=(uint16_t)((((color>>5)&63U)*a+7U)/15U);
        const uint16_t b=(uint16_t)(((color&31U)*a+7U)/15U);
        display_pixel(x+px,y+py,(uint16_t)((r<<11)|(g<<5)|b));
    }
}

static uint8_t water_mask_sample(const display_asset_t *mask, int x, int y)
{
    if (x < 0 || y < 0 || x >= mask->width || y >= mask->height) return 0;
    const size_t pixel = (size_t)y * mask->width + (size_t)x;
    const uint8_t packed = mask->coverage[pixel >> 1];
    return (pixel & 1U) ? (packed & 0x0FU) : (packed >> 4);
}

// Bilinear coverage sampling avoids angle-dependent jumps when the water-drop
// mask is rotated onto the LCD's pixel grid.
static uint8_t water_mask_coverage(const display_asset_t *mask, float x, float y)
{
    const float px = x - 0.5f;
    const float py = y - 0.5f;
    const int x0 = (int)floorf(px);
    const int y0 = (int)floorf(py);
    const unsigned fx = (unsigned)lroundf((px - (float)x0) * 16.0f);
    const unsigned fy = (unsigned)lroundf((py - (float)y0) * 16.0f);
    const unsigned ix = 16U - fx;
    const unsigned iy = 16U - fy;
    const unsigned coverage =
        water_mask_sample(mask, x0, y0) * ix * iy +
        water_mask_sample(mask, x0 + 1, y0) * fx * iy +
        water_mask_sample(mask, x0, y0 + 1) * ix * fy +
        water_mask_sample(mask, x0 + 1, y0 + 1) * fx * fy;
    return (uint8_t)((coverage + 128U) / 256U);
}

static void draw_rotated_water_marker(int tip_x, int tip_y, float ux, float uy,
                                     float scale, uint16_t color)
{
    const float tx = -uy;
    const float ty = ux;
    const display_asset_t *mask = &display_assets[ASSET_WATER];
    int min_x = tip_x - 32, max_x = tip_x + 32;
    int min_y = tip_y - 32, max_y = tip_y + 32;
    if (min_x < display_region_x) min_x = display_region_x;
    if (max_x >= display_region_x + display_region_width) max_x = display_region_x + display_region_width - 1;
    if (min_y < display_stripe_y) min_y = display_stripe_y;
    if (max_y >= display_stripe_y + display_stripe_height) max_y = display_stripe_y + display_stripe_height - 1;
    if (min_x > max_x || min_y > max_y) return;

    for (int y = min_y; y <= max_y; y++) {
        for (int x = min_x; x <= max_x; x++) {
            const float dx = (float)x + 0.5f - tip_x;
            const float dy = (float)y + 0.5f - tip_y;
            const float source_x = 12.0f + (dx * tx + dy * ty) / scale;
            const float source_y = 3.25f - (dx * ux + dy * uy) / scale;
            const uint8_t coverage = water_mask_coverage(mask, source_x, source_y);
            if (coverage == 0U) continue;
            display_blend_pixel(x, y, color, coverage);
        }
        if (((y - min_y) & 7) == 7) vTaskDelay(1);
    }
}

static void draw_gauge_pointer(uint16_t co2)
{
    const float pi = 3.14159265358979323846f;
    if (co2 < GAUGE_MIN_CO2) co2 = GAUGE_MIN_CO2;
    if (co2 > GAUGE_MAX_CO2) co2 = GAUGE_MAX_CO2;
    const float angle = (((float)(GAUGE_MAX_CO2 - co2) * 180.0f) /
                         (float)(GAUGE_MAX_CO2 - GAUGE_MIN_CO2)) * pi / 180.0f;
    const float ux = cosf(angle);
    const float uy = -sinf(angle);
    // Put the pointer tip on the gauge centerline, halfway through its stroke.
    const int outer_tip_x = GAUGE_CENTER_X + (int)lroundf(ux * GAUGE_RADIUS);
    const int outer_tip_y = GAUGE_CENTER_Y + (int)lroundf(uy * GAUGE_RADIUS);
    // Draw both layers from the same anchor so the outline stays centered and
    // has a consistent thickness as the pointer rotates.
    draw_rotated_water_marker(outer_tip_x, outer_tip_y,
                              ux, uy, 31.2f / 24.0f, COLOR_BLACK);
    draw_rotated_water_marker(outer_tip_x, outer_tip_y,
                              ux, uy, 25.2f / 24.0f, co2_color(co2));
}

static void draw_wifi_icon(int x, int y, uint16_t color)
{
    for (int py = 0; py < 16; py++) {
        for (int px = 0; px < 16; px++) {
            const size_t pixel = (size_t)py * 16U + (size_t)px;
            const uint8_t packed = display_assets[ASSET_WIFI].coverage[pixel >> 1];
            const uint8_t coverage = (pixel & 1U) ? (packed & 0x0FU) : (packed >> 4);
            if (coverage == 0) continue;
            const uint16_t r = (uint16_t)((((color >> 11) & 0x1FU) * coverage + 7U) / 15U);
            const uint16_t g = (uint16_t)((((color >> 5) & 0x3FU) * coverage + 7U) / 15U);
            const uint16_t b = (uint16_t)(((color & 0x1FU) * coverage + 7U) / 15U);
            display_pixel(x + px, y + py, (uint16_t)((r << 11) | (g << 5) | b));
        }
    }
}

static bool display_wifi_visible(void)
{
    return zigbee_network_joined || (zigbee_network_joining && display_wifi_blink_on);
}

static void draw_environment_values(void)
{
    if (!display_measurement_valid) return;
    char temperature_text[16];
    char humidity_text[8];
    const float display_value = display_fahrenheit ?
        (display_temperature * 9.0f / 5.0f + 32.0f) : display_temperature;
    snprintf(temperature_text, sizeof(temperature_text), "%.1fo%s",
             (double)display_value, display_fahrenheit ? "F" : "C");
    snprintf(humidity_text, sizeof(humidity_text), "%.0f%%", (double)display_humidity);
    const uint16_t temp_color = temperature_color(display_value, display_fahrenheit);
    const uint16_t hum_color = humidity_color(display_humidity);
    draw_thermometer_icon(60, 164, temp_color);
    draw_water_icon(156, 164, hum_color);
    draw_small_text_centered(temperature_text, 72, 193, temp_color);
    draw_small_text_centered(humidity_text, 168, 193, hum_color);
}

typedef struct {
    int x;
    int y;
    int width;
    int height;
} display_rect_t;

static display_rect_t co2_value_rect(uint16_t co2)
{
    char value[8];
    snprintf(value, sizeof(value), "%u", (unsigned)co2);
    const int count = (int)strlen(value);
    const int width = count * DIGIT_SLOT_WIDTH + (count - 1) * DIGIT_SLOT_GAP;
    return (display_rect_t) { (DISPLAY_WIDTH - width) / 2, 95, width, 68 };
}

static display_rect_t pointer_rect(uint16_t co2)
{
    const float pi = 3.14159265358979323846f;
    if (co2 < GAUGE_MIN_CO2) co2 = GAUGE_MIN_CO2;
    if (co2 > GAUGE_MAX_CO2) co2 = GAUGE_MAX_CO2;
    const float angle = (((float)(GAUGE_MAX_CO2 - co2) * 180.0f) /
                         (float)(GAUGE_MAX_CO2 - GAUGE_MIN_CO2)) * pi / 180.0f;
    const float ux = cosf(angle), uy = -sinf(angle);
    const float cx = GAUGE_CENTER_X + ux * (GAUGE_RADIUS - 10.9f);
    const float cy = GAUGE_CENTER_Y + uy * (GAUGE_RADIUS - 10.9f);
    const int half_x = (int)ceilf(fabsf(ux) * 12.9f + fabsf(uy) * 9.8f + 2.0f);
    const int half_y = (int)ceilf(fabsf(uy) * 12.9f + fabsf(ux) * 9.8f + 2.0f);
    return (display_rect_t) {
        (int)floorf(cx) - half_x, (int)floorf(cy) - half_y,
        half_x * 2 + 2, half_y * 2 + 2
    };
}

static esp_err_t display_render_rect(display_rect_t rect, uint16_t shown_co2)
{
    if (rect.x < 0) { rect.width += rect.x; rect.x = 0; }
    if (rect.y < 0) { rect.height += rect.y; rect.y = 0; }
    if (rect.x + rect.width > DISPLAY_WIDTH) rect.width = DISPLAY_WIDTH - rect.x;
    if (rect.y + rect.height > DISPLAY_HEIGHT) rect.height = DISPLAY_HEIGHT - rect.y;
    if (rect.width <= 0 || rect.height <= 0) return ESP_OK;

    for (int y = rect.y; y < rect.y + rect.height; y += DISPLAY_STRIPE_HEIGHT) {
        display_region_x = rect.x;
        display_region_width = rect.width;
        display_stripe_y = y;
        display_stripe_height = rect.y + rect.height - y;
        if (display_stripe_height > DISPLAY_STRIPE_HEIGHT) display_stripe_height = DISPLAY_STRIPE_HEIGHT;
        const size_t pixel_count = (size_t)display_region_width * display_stripe_height;
        memset(display_buffer, 0, pixel_count * sizeof(display_buffer[0]));
        draw_gauge_arc();
        draw_gauge_pointer(shown_co2);
        draw_co2_label();
        draw_co2_value(shown_co2);
        draw_ppm_label();
        draw_environment_values();
        if (display_wifi_visible()) draw_wifi_icon(112, 218, COLOR_WHITE);

        // RGB565 is sent high byte first by the panel.
        for (size_t i = 0; i < pixel_count; i++) {
            const uint16_t pixel = display_buffer[i];
            display_buffer[i] = (uint16_t)((pixel << 8) | (pixel >> 8));
        }
        esp_err_t err = esp_lcd_panel_draw_bitmap(
            display_panel, rect.x, y, rect.x + rect.width, y + display_stripe_height,
            display_buffer);
        if (err != ESP_OK) return err;
        if (xSemaphoreTake(display_transfer_done, portMAX_DELAY) != pdTRUE) return ESP_FAIL;
        // Full-frame software antialiasing is CPU intensive on ESP32-H2.
        // Give the single-core scheduler a tick between DMA stripes.
        vTaskDelay(1);
    }
    return ESP_OK;
}

static esp_err_t display_update(void)
{
    display_rect_t rects[8];
    size_t rect_count = 0;
    const uint16_t shown_co2 = display_co2 > GAUGE_MAX_CO2 ? GAUGE_MAX_CO2 :
                               (display_co2 < GAUGE_MIN_CO2 ? GAUGE_MIN_CO2 : display_co2);
    const bool wifi_visible = display_wifi_visible();

    if (display_force_full_refresh || !display_previous_co2_valid) {
        rects[rect_count++] = (display_rect_t) { 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT };
    } else {
        if (shown_co2 != display_previous_co2) {
            const display_rect_t old_value = co2_value_rect(display_previous_co2);
            const display_rect_t new_value = co2_value_rect(shown_co2);
            const int x0 = old_value.x < new_value.x ? old_value.x : new_value.x;
            const int x1 = old_value.x + old_value.width > new_value.x + new_value.width ?
                           old_value.x + old_value.width : new_value.x + new_value.width;
            rects[rect_count++] = (display_rect_t) { x0, 95, x1 - x0, 68 };
            rects[rect_count++] = pointer_rect(display_previous_co2);
            rects[rect_count++] = pointer_rect(shown_co2);
        }
        if (display_measurement_valid && (!display_previous_environment_valid ||
            display_fahrenheit != display_previous_fahrenheit ||
            temperature_color(display_fahrenheit ?
                                  display_temperature * 9.0f / 5.0f + 32.0f : display_temperature,
                              display_fahrenheit) !=
                temperature_color(display_previous_fahrenheit ?
                                      display_previous_temperature * 9.0f / 5.0f + 32.0f :
                                          display_previous_temperature,
                                  display_previous_fahrenheit) ||
            (int)lroundf(display_temperature * 10.0f) != (int)lroundf(display_previous_temperature * 10.0f))) {
            rects[rect_count++] = (display_rect_t) { 24, 160, 96, 55 };
        }
        if (display_measurement_valid && (!display_previous_environment_valid ||
            humidity_color(display_humidity) != humidity_color(display_previous_humidity) ||
            (int)lroundf(display_humidity) != (int)lroundf(display_previous_humidity))) {
            rects[rect_count++] = (display_rect_t) { 120, 160, 96, 55 };
        }
        if (!display_previous_wifi_valid || wifi_visible != display_previous_wifi_visible) {
            rects[rect_count++] = (display_rect_t) { 112, 218, 16, 16 };
        }
    }

    for (size_t i = 0; i < rect_count; i++) {
        esp_err_t err = display_render_rect(rects[i], shown_co2);
        if (err != ESP_OK) return err;
    }
    display_previous_co2 = shown_co2;
    display_previous_co2_valid = true;
    display_previous_temperature = display_temperature;
    display_previous_fahrenheit = display_fahrenheit;
    display_previous_humidity = display_humidity;
    display_previous_environment_valid = display_measurement_valid;
    display_previous_wifi_visible = wifi_visible;
    display_previous_wifi_valid = true;
    display_force_full_refresh = false;
    return ESP_OK;
}

static uint8_t light_raw_to_brightness(int raw)
{
    if (raw <= LIGHT_ADC_DARK_RAW) return BACKLIGHT_MIN_PERCENT;
    if (raw >= LIGHT_ADC_BRIGHT_RAW) return BACKLIGHT_MAX_PERCENT;

    const int raw_range = LIGHT_ADC_BRIGHT_RAW - LIGHT_ADC_DARK_RAW;
    const int brightness_range = BACKLIGHT_MAX_PERCENT - BACKLIGHT_MIN_PERCENT;
    return (uint8_t)(BACKLIGHT_MIN_PERCENT +
                     ((raw - LIGHT_ADC_DARK_RAW) * brightness_range) / raw_range);
}

/*
 * Convert ADC counts to ZCL Illuminance Measurement MeasuredValue.
 * ZCL stores 10000*log10(lux)+1; 0 means below 1 lx and 0xFFFF is invalid.
 * Calibration anchors are the two phone-meter readings supplied by the user.
 */
static uint16_t light_raw_to_zcl_illuminance(int raw)
{
    if (raw <= LIGHT_ADC_DARK_RAW) {
        return 0; // Below 1 lx
    }

    uint32_t encoded;
    if (raw <= LIGHT_CAL_49_LUX_RAW) {
        encoded = ((uint32_t)(raw - LIGHT_ADC_DARK_RAW) * LIGHT_CAL_49_LUX_ZCL) /
                  (LIGHT_CAL_49_LUX_RAW - LIGHT_ADC_DARK_RAW);
    } else {
        const uint32_t span_raw = LIGHT_CAL_304_LUX_RAW - LIGHT_CAL_49_LUX_RAW;
        const uint32_t span_zcl = LIGHT_CAL_304_LUX_ZCL - LIGHT_CAL_49_LUX_ZCL;
        encoded = LIGHT_CAL_49_LUX_ZCL +
                  ((uint32_t)(raw - LIGHT_CAL_49_LUX_RAW) * span_zcl) / span_raw;
    }

    if (encoded > 0xFFFEU) {
        encoded = 0xFFFEU;
    }
    return (uint16_t)encoded;
}

static void backlight_set_brightness(uint8_t brightness_percent)
{
    if (!backlight_pwm_ready) return;
    if (brightness_percent > BACKLIGHT_MAX_PERCENT) {
        brightness_percent = BACKLIGHT_MAX_PERCENT;
    }

    // The N-MOSFET is active-high, but it pulls BLK low (backlight OFF).
    // Invert PWM duty so 100% requested brightness means 0% MOSFET-on time.
    const uint32_t mosfet_on_percent = BACKLIGHT_MAX_PERCENT - brightness_percent;
    const uint32_t duty = (mosfet_on_percent * BACKLIGHT_PWM_DUTY_MAX + 50U) / 100U;
    esp_err_t err = ledc_set_duty(BACKLIGHT_LEDC_MODE, BACKLIGHT_LEDC_CHANNEL, duty);
    if (err == ESP_OK) {
        err = ledc_update_duty(BACKLIGHT_LEDC_MODE, BACKLIGHT_LEDC_CHANNEL);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Backlight PWM update failed: %s", esp_err_to_name(err));
        return;
    }

    backlight_brightness_percent = brightness_percent;
}

static esp_err_t display_backlight_init(void)
{
    const ledc_timer_config_t timer_config = {
        .speed_mode = BACKLIGHT_LEDC_MODE,
        .duty_resolution = BACKLIGHT_PWM_RESOLUTION,
        .timer_num = BACKLIGHT_LEDC_TIMER,
        .freq_hz = BACKLIGHT_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_config);
    if (err != ESP_OK) return err;

    const ledc_channel_config_t channel_config = {
        .gpio_num = BACKLIGHT_GPIO,
        .speed_mode = BACKLIGHT_LEDC_MODE,
        .channel = BACKLIGHT_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BACKLIGHT_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    err = ledc_channel_config(&channel_config);
    if (err != ESP_OK) return err;
    backlight_pwm_ready = true;
    backlight_set_brightness(BACKLIGHT_MAX_PERCENT);

    const adc_oneshot_unit_init_cfg_t adc_unit_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    err = adc_oneshot_new_unit(&adc_unit_config, &light_sensor_adc);
    if (err != ESP_OK) return err;

    const adc_oneshot_chan_cfg_t adc_channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(
        light_sensor_adc,
        LIGHT_SENSOR_ADC_CHANNEL,
        &adc_channel_config);
    if (err != ESP_OK) {
        adc_oneshot_del_unit(light_sensor_adc);
        light_sensor_adc = NULL;
        return err;
    }

    light_sensor_adc_ready = true;
    last_light_sample_tick = xTaskGetTickCount();
    last_light_log_tick = last_light_sample_tick;
    ESP_LOGI(TAG, "Ambient light ADC initialized on GPIO%d", LIGHT_SENSOR_GPIO);
    ESP_LOGI(TAG, "Backlight PWM initialized on GPIO%d at %d Hz", BACKLIGHT_GPIO,
             BACKLIGHT_PWM_FREQUENCY_HZ);
    return ESP_OK;
}

static void display_backlight_update_from_sensor(TickType_t now)
{
    if (light_sensor_adc_ready &&
        (now - last_light_sample_tick) >= pdMS_TO_TICKS(LIGHT_SAMPLE_INTERVAL_MS)) {
        last_light_sample_tick = now;

        int sample_sum = 0;
        bool sample_valid = true;
        for (int i = 0; i < 4; i++) {
            int sample = 0;
            if (adc_oneshot_read(light_sensor_adc, LIGHT_SENSOR_ADC_CHANNEL, &sample) != ESP_OK) {
                sample_valid = false;
                break;
            }
            sample_sum += sample;
        }

        if (sample_valid) {
            const int raw = sample_sum / 4;

            // Filter ADC fluctuations before converting ambient light to brightness.
            if (light_sensor_filtered_raw < 0) {
                light_sensor_filtered_raw = raw;
            } else {
                light_sensor_filtered_raw = (3 * light_sensor_filtered_raw + raw) / 4;
            }

            backlight_target_brightness_percent =
                light_raw_to_brightness(light_sensor_filtered_raw);
        }
    }

    // Move the PWM gradually instead of applying each sensor target at once.
    if (backlight_brightness_percent < backlight_target_brightness_percent) {
        const uint8_t next = (backlight_target_brightness_percent - backlight_brightness_percent >
                              BACKLIGHT_FADE_STEP_PERCENT)
                                 ? backlight_brightness_percent + BACKLIGHT_FADE_STEP_PERCENT
                                 : backlight_target_brightness_percent;
        backlight_set_brightness(next);
    } else if (backlight_brightness_percent > backlight_target_brightness_percent) {
        const uint8_t next = (backlight_brightness_percent - backlight_target_brightness_percent >
                              BACKLIGHT_FADE_STEP_PERCENT)
                                 ? backlight_brightness_percent - BACKLIGHT_FADE_STEP_PERCENT
                                 : backlight_target_brightness_percent;
        backlight_set_brightness(next);
    }

    if ((now - last_light_log_tick) >= pdMS_TO_TICKS(10000)) {
        ESP_LOGI(TAG, "Ambient ADC raw=%d, backlight=%u%% (target %u%%)",
                 light_sensor_filtered_raw, backlight_brightness_percent,
                 backlight_target_brightness_percent);
        last_light_log_tick = now;
    }
}

static void display_task(void *arg)
{
    (void)arg;

    bool state_known = false;
    bool previous_joined = false;
    bool previous_joining = false;
    TickType_t last_blink = xTaskGetTickCount();
    const TickType_t blink_interval = pdMS_TO_TICKS(500);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));

        const TickType_t now = xTaskGetTickCount();
        display_backlight_update_from_sensor(now);
        const bool joined = zigbee_network_joined;
        const bool joining = zigbee_network_joining;
        bool refresh = display_data_dirty;
        if (refresh) display_data_dirty = false;

        if (!state_known || joined != previous_joined || joining != previous_joining) {
            state_known = true;
            previous_joined = joined;
            previous_joining = joining;
            display_wifi_blink_on = true;
            last_blink = now;
            refresh = true;
        }

        if (joining && (now - last_blink) >= blink_interval) {
            display_wifi_blink_on = !display_wifi_blink_on;
            last_blink = now;
            refresh = true;
        } else if (!joining) {
            display_wifi_blink_on = true;
        }

        if (refresh) {
            const esp_err_t err = display_update();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "GC9A01 update failed: %s", esp_err_to_name(err));
            }
        }
    }
}

static esp_err_t display_assets_init(void)
{
    const esp_vfs_spiffs_conf_t conf = {
        .base_path = "/display",
        .partition_label = "display_fs",
        .max_files = 1,
        .format_if_mount_failed = false,
    };
    ESP_LOGI(TAG, "Mounting display SPIFFS partition...");
    vTaskDelay(1);
    esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "Display SPIFFS mounted; loading %d assets", ASSET_COUNT);
    vTaskDelay(1);

    for (int id = 0; id < ASSET_COUNT; id++) {
        char path[48];
        if (id < 10) snprintf(path, sizeof(path), "/display/large_%d.bin", id);
        else if (id < 20) snprintf(path, sizeof(path), "/display/small_%d.bin", id - 10);
        else {
            static const char *const names[] = {
                "ppm", "co2_icon", "wifi_icon", "thermometer", "water",
                "glyph_o", "glyph_C", "glyph_percent", "glyph_dot", "glyph_minus", "glyph_F"
            };
            snprintf(path, sizeof(path), "/display/%s.bin", names[id - 20]);
        }
        FILE *file = fopen(path, "rb");
        if (file == NULL) return ESP_ERR_NOT_FOUND;
        uint8_t header[7];
        if (fread(header, 1, sizeof(header), file) != sizeof(header) ||
            header[0] != 'D' || header[1] != '4' || header[2] != 1) {
            fclose(file);
            return ESP_ERR_INVALID_RESPONSE;
        }
        const uint16_t width = (uint16_t)(header[3] | ((uint16_t)header[4] << 8));
        const uint16_t height = (uint16_t)(header[5] | ((uint16_t)header[6] << 8));
        const size_t bytes = ((size_t)width * height + 1U) / 2U;
        uint8_t *coverage = malloc(bytes);
        if (coverage == NULL) { fclose(file); return ESP_ERR_NO_MEM; }
        if (fread(coverage, 1, bytes, file) != bytes) {
            free(coverage); fclose(file); return ESP_ERR_INVALID_SIZE;
        }
        fclose(file);
        display_assets[id] = (display_asset_t) { width, height, coverage };
        // Let IDLE run while the startup task reads the individual SPIFFS files.
        vTaskDelay(1);
    }
    ESP_LOGI(TAG, "Loaded %d antialiased display assets from SPIFFS", ASSET_COUNT);
    return ESP_OK;
}

static esp_err_t display_init(void)
{
    // The SuperMini blue LED is connected to GPIO13. Keep it off so it does not
    // illuminate the nearby photoresistor. LCD reset is routed to GPIO3 instead.
    const gpio_config_t blue_led_config = {
        .pin_bit_mask = 1ULL << BOARD_BLUE_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&blue_led_config), TAG, "Failed to configure blue LED GPIO");
    ESP_RETURN_ON_ERROR(gpio_set_level(BOARD_BLUE_LED_GPIO, 0), TAG, "Failed to turn off blue LED");

    esp_err_t backlight_err = display_backlight_init();
    if (backlight_err != ESP_OK) {
        ESP_LOGW(TAG, "Automatic backlight setup failed; using full brightness: %s",
                 esp_err_to_name(backlight_err));
    }

    esp_err_t assets_err = display_assets_init();
    if (assets_err != ESP_OK) {
        ESP_LOGE(TAG, "Display asset filesystem unavailable: %s", esp_err_to_name(assets_err));
        return assets_err;
    }
    if (display_retained_measurement.magic == DISPLAY_RETAINED_MAGIC &&
        display_retained_measurement.co2 >= 250 &&
        display_retained_measurement.co2 <= 10000 &&
        isfinite(display_retained_measurement.temperature) &&
        display_retained_measurement.temperature >= -40.0f &&
        display_retained_measurement.temperature <= 125.0f &&
        isfinite(display_retained_measurement.humidity) &&
        display_retained_measurement.humidity >= 0.0f &&
        display_retained_measurement.humidity <= 100.0f) {
        display_co2 = display_retained_measurement.co2;
        display_temperature = display_retained_measurement.temperature;
        display_humidity = display_retained_measurement.humidity;
        display_measurement_valid = true;
    }

    const spi_bus_config_t bus_config = {
        .sclk_io_num = DISPLAY_SPI_SCLK_GPIO,
        .mosi_io_num = DISPLAY_SPI_MOSI_GPIO,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DISPLAY_WIDTH * DISPLAY_STRIPE_HEIGHT * sizeof(uint16_t),
    };
    esp_err_t err = spi_bus_initialize(DISPLAY_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) return err;

    display_transfer_done = xSemaphoreCreateBinary();
    if (display_transfer_done == NULL) return ESP_ERR_NO_MEM;

    esp_lcd_panel_io_spi_config_t io_config =
        GC9A01_PANEL_IO_SPI_CONFIG(
            DISPLAY_CS_GPIO,
            DISPLAY_DC_GPIO,
            display_color_transfer_done,
            display_transfer_done);
    io_config.pclk_hz = DISPLAY_SPI_CLOCK_HZ;
    io_config.trans_queue_depth = 1;
    err = esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)DISPLAY_SPI_HOST, &io_config, &display_io);
    if (err != ESP_OK) return err;

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = DISPLAY_RST_GPIO,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    err = esp_lcd_new_panel_gc9a01(display_io, &panel_config, &display_panel);
    if (err != ESP_OK) return err;
    err = esp_lcd_panel_reset(display_panel);
    if (err != ESP_OK) return err;
    err = esp_lcd_panel_init(display_panel);
    if (err != ESP_OK) return err;
    err = esp_lcd_panel_invert_color(display_panel, true);
    if (err != ESP_OK) return err;
    // Correct the left-right mirrored image on this GC9A01 module.
    err = esp_lcd_panel_mirror(display_panel, true, false);
    if (err != ESP_OK) return err;
    err = esp_lcd_panel_disp_on_off(display_panel, true);
    if (err != ESP_OK) return err;

    return display_update();
}
// ============================================================
// CO2 status
// ============================================================

static const char *co2_status(
    uint16_t co2
)
{
    if (co2 < 800) {
        return "NORMAL";
    }


    if (co2 < 1200) {
        return "ELEVATED";
    }


    if (co2 < 2000) {
        return "HIGH";
    }


    return "VERY HIGH";
}


// ============================================================
// I2C / devices init
// ============================================================

static esp_err_t devices_init(void)
{
    esp_err_t ret;


    /*
     * --------------------------------------------------------
     * i2cdev
     * --------------------------------------------------------
     */

    ESP_ERROR_CHECK(
        i2cdev_init()
    );


    ESP_LOGI(
        TAG,
        "i2cdev initialized"
    );


    /*
     * --------------------------------------------------------
     * SCD40
     * --------------------------------------------------------
     */

    memset(
        &scd40,
        0,
        sizeof(scd40)
    );


    ESP_ERROR_CHECK(
        scd4x_init_desc(
            &scd40,
            I2C_PORT,
            I2C_SDA_GPIO,
            I2C_SCL_GPIO
        )
    );


    scd40.cfg.sda_pullup_en =
        GPIO_PULLUP_ENABLE;


    scd40.cfg.scl_pullup_en =
        GPIO_PULLUP_ENABLE;


    scd40.cfg.master.clk_speed =
        I2C_SPEED_HZ;


    ESP_LOGI(
        TAG,
        "SCD40 descriptor initialized at 0x%02X",
        scd40.addr
    );


    /*
     * Check SCD40.
     */

    ret =
        i2c_dev_check_present(
            &scd40
        );


    if (ret != ESP_OK) {

        ESP_LOGE(
            TAG,
            "SCD40 not found on I2C: %s",
            esp_err_to_name(ret)
        );

        return ret;
    }


    ESP_LOGI(
        TAG,
        "SCD40 detected on I2C"
    );


    /*
     * --------------------------------------------------------
     * Shared I2C bus
     * --------------------------------------------------------
     */

    ret =
        i2cdev_get_shared_handle(
            I2C_PORT,
            (void **)&i2c_bus
        );


    if (ret != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to get shared I2C bus handle: %s",
            esp_err_to_name(ret)
        );

        return ret;
    }


    ESP_LOGI(
        TAG,
        "Shared I2C bus handle obtained"
    );


    /*
     * --------------------------------------------------------
     * GC9A01 TFT
     * --------------------------------------------------------
     */

    ret =
        display_init();


    if (ret != ESP_OK) {

        ESP_LOGE(
            TAG,
            "GC9A01 initialization failed: %s",
            esp_err_to_name(ret)
        );

        return ret;
    }


    return ESP_OK;
}

static void zigbee_report_attr(uint16_t cluster_id, uint16_t attribute_id, const char *name)
{
    if (!zigbee_network_joined) {
        ESP_LOGD(ZIGBEE_TAG, "%s report skipped: not joined", name);
        return;
    }

    esp_zb_zcl_report_attr_cmd_t report_cmd = {0};
    report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    report_cmd.zcl_basic_cmd.src_endpoint = ZIGBEE_ENDPOINT;
    report_cmd.zcl_basic_cmd.dst_endpoint = 1;
    report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    report_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_cmd.clusterID = cluster_id;
    report_cmd.attributeID = attribute_id;

    /* Never let a stalled Zigbee stack block the sensor loop indefinitely. */
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(ZIGBEE_TAG, "%s report skipped: could not acquire Zigbee lock", name);
        return;
    }
    esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&report_cmd);
    esp_zb_lock_release();

    ESP_LOGI(ZIGBEE_TAG, "%s report: %s", name, esp_err_to_name(err));
}
// ============================================================
// Zigbee sensor attributes update
// ============================================================

static void zigbee_update_sensor_values(
    uint16_t co2,
    float temperature,
    float humidity
)
{
    if (!zigbee_stack_started) {
        return;
    }

    if (esp_zb_lock_acquire(pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(
            ZIGBEE_TAG,
            "Could not acquire Zigbee lock for sensor update"
        );
        return;
    }

    int16_t temperature_value =
        (int16_t)(temperature * 100.0f);

    if (temperature_value < -4000) {
        temperature_value = -4000;
    }

    if (temperature_value > 8500) {
        temperature_value = 8500;
    }

    uint16_t humidity_value =
        (uint16_t)(humidity * 100.0f + 0.5f);

    if (humidity_value > 10000) {
        humidity_value = 10000;
    }

    const int ambient_raw = light_sensor_filtered_raw;
    uint16_t illuminance_value = light_raw_to_zcl_illuminance(ambient_raw);

    /*
     * Carbon Dioxide Measurement cluster uses a float (SINGLE)
     * value in ppm. The cluster above accepts 0..10000 ppm.
     */
    float co2_zigbee = (float)co2 / 1000000.0f;

    if (co2_zigbee < 0.0f) {
        co2_zigbee = 0.0f;
    }

    if (co2_zigbee > 0.01f) {
        co2_zigbee = 0.01f;
    }

    esp_zb_zcl_status_t status =
        esp_zb_zcl_set_attribute_val(
            ZIGBEE_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
            &temperature_value,
            false
        );

    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(
            ZIGBEE_TAG,
            "Failed to update temperature attribute: 0x%02x",
            status
        );
    }

    status = esp_zb_zcl_set_attribute_val(
        ZIGBEE_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID,
        (void *)&illuminance_value,
        false
    );

    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(ZIGBEE_TAG, "Failed to update illuminance attribute: 0x%02x", status);
    } else {
        ESP_LOGI(ZIGBEE_TAG, "Illuminance attribute updated: ADC=%d, ZCL=%u",
                 ambient_raw, illuminance_value);
    }

    status =
        esp_zb_zcl_set_attribute_val(
            ZIGBEE_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
            &humidity_value,
            false
        );

    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(
            ZIGBEE_TAG,
            "Failed to update humidity attribute: 0x%02x",
            status
        );
    }

    esp_zb_zcl_attr_t *co2_attr =
    esp_zb_zcl_get_attribute(
        ZIGBEE_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID
    );

    if (co2_attr == NULL || co2_attr->data_p == NULL) {
        ESP_LOGE(
            ZIGBEE_TAG,
            "CO2 attribute not found!"
        );
    } else {
        *((float *)co2_attr->data_p) = co2_zigbee;

        ESP_LOGI(
            ZIGBEE_TAG,
            "CO2 Zigbee attribute: %.6f (%u ppm)",
            co2_zigbee,
            co2
        );
    }

    esp_zb_lock_release();

    /* Each standard measurement cluster needs its own report. */
    zigbee_report_attr(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, "Temperature");
    zigbee_report_attr(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, "Humidity");
    zigbee_report_attr(ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT, ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID, "Illuminance");
    zigbee_report_attr(ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID, "CO2");

    ESP_LOGD(
        ZIGBEE_TAG,
        "Zigbee values updated: CO2=%.0f ppm, T=%d, H=%u",
        co2_zigbee,
        temperature_value,
        humidity_value
    );
}


// ============================================================
// app_main
// ============================================================

void app_main(void)
{
    const TickType_t app_start_tick = xTaskGetTickCount();

    /*
     * --------------------------------------------------------
     * NVS
     * --------------------------------------------------------
     */

    ESP_ERROR_CHECK(
        nvs_flash_init()
    );

    load_temperature_display_mode();
    load_sensor_calibration();


    ESP_LOGI(
        TAG,
        "========================================"
    );


    ESP_LOGI(
        TAG,
        "ESP32-H2 CO2 monitor"
    );


    ESP_LOGI(
        TAG,
        "SCD40 + GC9A01"
    );


    ESP_LOGI(
        TAG,
        "Zigbee SDK 1.6.8"
    );


    ESP_LOGI(
        TAG,
        "========================================"
    );


    /*
     * --------------------------------------------------------
     * Zigbee platform configuration
     * --------------------------------------------------------
     *
     * Это обязательная часть старого API 1.x.
     */

    esp_zb_platform_config_t
        zigbee_platform_config = {

            .radio_config = {
                .radio_mode =
                    ZB_RADIO_MODE_NATIVE,
            },

            .host_config = {
                .host_connection_mode =
                    ZB_HOST_CONNECTION_MODE_NONE,
            },
        };


    ESP_ERROR_CHECK(
        esp_zb_platform_config(
            &zigbee_platform_config
        )
    );


    ESP_LOGI(
        ZIGBEE_TAG,
        "Zigbee platform configured"
    );


    /*
     * --------------------------------------------------------
     * Hardware
     * --------------------------------------------------------
     */

    ESP_ERROR_CHECK(
        devices_init()
    );

    if (xTaskCreate(display_task, "display", 4096, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create display task");
        return;
    }


    /*
     * --------------------------------------------------------
     * SCD40 periodic measurement
     * --------------------------------------------------------
     */

    ESP_LOGI(
        TAG,
        "Stopping SCD40 periodic measurement..."
    );


    esp_err_t ret =
        scd4x_stop_periodic_measurement(
            &scd40
        );


    if (ret != ESP_OK) {

        ESP_LOGW(
            TAG,
            "SCD40 stop returned: %s",
            esp_err_to_name(ret)
        );
    }


    vTaskDelay(
        pdMS_TO_TICKS(500)
    );


    ESP_LOGI(
        TAG,
        "Starting SCD40 periodic measurement..."
    );


    ret =
        scd4x_start_periodic_measurement(
            &scd40
        );


    if (ret != ESP_OK) {

        ESP_LOGE(
            TAG,
            "SCD40 start periodic measurement failed: %s",
            esp_err_to_name(ret)
        );

        return;
    }


    ESP_LOGI(
        TAG,
        "SCD40 periodic measurement started"
    );

    ESP_LOGI(TAG, "Waiting %u ms for SCD40 startup stabilization",
             (unsigned)SCD40_STARTUP_SETTLE_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(SCD40_STARTUP_SETTLE_DELAY_MS));


    /*
     * --------------------------------------------------------
     * Button
     * --------------------------------------------------------
     */

    zigbee_button_init();


    xTaskCreate(
        zigbee_button_task,
        "zigbee_button",
        2048,
        NULL,
        5,
        NULL
    );


    /*
     * --------------------------------------------------------
     * Zigbee task
     * --------------------------------------------------------
     */

    xTaskCreate(
        zigbee_task,
        "Zigbee_main",
        8192,
        NULL,
        5,
        NULL
    );

    /*
     * --------------------------------------------------------
     * Main CO2 loop
     * --------------------------------------------------------
     */
    TickType_t last_zigbee_update = 0;
    #define ZIGBEE_UPDATE_INTERVAL_MS 60000
    bool skip_first_valid_measurement = true;

    while (true) {

        bool data_ready =
            false;


        ret =
            scd4x_get_data_ready_status(
                &scd40,
                &data_ready
            );


        if (ret != ESP_OK) {

            if (ret == ESP_ERR_INVALID_CRC) {
                ESP_LOGW(TAG,
                         "SCD40 data-ready CRC-8 error; retrying readiness check");
            } else {
                ESP_LOGE(
                    TAG,
                    "Data ready check failed: %s",
                    esp_err_to_name(ret)
                );
            }


            vTaskDelay(
                pdMS_TO_TICKS(250)
            );


            continue;
        }


        if (!data_ready) {

            vTaskDelay(
                pdMS_TO_TICKS(250)
            );

            continue;
        }


        /*
         * ----------------------------------------------------
         * Read measurement
         * ----------------------------------------------------
         */

        uint16_t co2 =
            0;

        float temperature =
            0.0f;

        float humidity =
            0.0f;


        ret =
            scd4x_read_measurement(
                &scd40,
                &co2,
                &temperature,
                &humidity
            );


        if (ret != ESP_OK) {

            if (ret == ESP_ERR_INVALID_CRC) {
                ESP_LOGW(TAG,
                         "SCD40 CRC-8 mismatch; discarded the complete CO2/temperature/humidity sample; "
                         "will retry when the next sample is ready");
            } else {
                ESP_LOGE(
                    TAG,
                    "SCD40 read measurement failed: %s",
                    esp_err_to_name(ret)
                );
            }


            vTaskDelay(
                pdMS_TO_TICKS(250)
            );


            continue;
        }

        if (skip_first_valid_measurement) {
            skip_first_valid_measurement = false;
            ESP_LOGI(TAG,
                     "Discarding the first valid SCD40 measurement after startup");
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        const float corrected_temperature =
            temperature + temperature_calibration_tenths / 10.0f;
        float corrected_humidity = humidity + humidity_calibration_percent;
        if (corrected_humidity < 0.0f) corrected_humidity = 0.0f;
        if (corrected_humidity > 100.0f) corrected_humidity = 100.0f;


        /*
         * ----------------------------------------------------
         * Log
         * ----------------------------------------------------
         */

        ESP_LOGI(
            TAG,
            "CO2=%u ppm | TEMP=%.1f C | HUM=%.1f %% | %s",

            co2,

            corrected_temperature,

            corrected_humidity,

            co2_status(co2)
        );


        /*
         * ----------------------------------------------------
         * GC9A01 TFT
         * ----------------------------------------------------
         */

        draw_measurement(
            co2,
            corrected_temperature,
            corrected_humidity
        );


        /*
         * Update Zigbee Temperature, Humidity and CO2 attributes.
         */

        TickType_t now = xTaskGetTickCount();

        if (zigbee_network_joined &&
            (now - app_start_tick) >=
                pdMS_TO_TICKS(FIRST_ZIGBEE_REPORT_DELAY_MS) &&
            (last_zigbee_update == 0 ||
             (now - last_zigbee_update) >=
                 pdMS_TO_TICKS(ZIGBEE_UPDATE_INTERVAL_MS))) {
            ESP_LOGI(
                TAG,
                ">>> Zigbee update attempt: CO2=%u ppm, T=%.1f C, H=%.1f %%",
                co2,
                corrected_temperature,
                corrected_humidity
            );

            zigbee_update_sensor_values(
                co2,
                corrected_temperature,
                corrected_humidity
            );

            last_zigbee_update = now;

            ESP_LOGI(
                TAG,
                ">>> Zigbee update function returned"
            );
        }

        vTaskDelay(
            pdMS_TO_TICKS(250)
        );
    }
}
