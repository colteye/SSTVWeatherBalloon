#include "ax25.h"

#include "bit_ops.h"
#include "error_handling.h"

#define AX25_FLAG (0x7E)
#define AX25_CTRL_FIELD (0x03)
#define AX25_PROTOCOL_ID (0xF0)

// Calculated via: (1b / 1200 b/s) * 1000 ms/s
// Assume bitrate of 1200 bits/s !!
#define AX25_AFSK1200_BIT_MS (0.833333f)

#define AX25_AFSK1200_1200HZ (1200)
#define AX25_AFSK1200_2200HZ (2200)

#define AX25_AFSK1200_PREAMBLE_BYTES (25)
#define AX25_AFSK1200_REST_BYTES (5)

// Based on https://github.com/satyendrakumarsingh/ISO-IEC-3309-compliant-16-bit-CRC-in-C-Language/blob/master/iso_iec_3309_crc.c
esp_err_t ax25_iso_3099_fcs_checksum(uint8_t *data, uint8_t data_len, uint16_t *fcs)
{
    uint8_t x;
    *fcs = 0xFFFF;

    while (data_len--)
    {
        x = *fcs >> 8 ^ *data++;
        x ^= x >> 4;
        *fcs = (*fcs << 8) ^ ((unsigned short)(x << 12)) ^
               ((unsigned short)(x << 5)) ^
               ((unsigned short)x);
    }

    return ESP_OK;
}

// Based on https://www.cs.cmu.edu/afs/club/usr/jhutz/project/tmp/x/APRS101m.pdf (page 20)
esp_err_t ax25_byte(uint8_t byte, uint16_t *current_freq, radio_waveform_data_t *ax25_buf)
{
    // Init error.
    esp_err_t err = ESP_OK;

    // Iterate over bits of byte.
    for (uint8_t i = 0; i < BYTE_SIZE; ++i)
    {
        // If bit is 0, change frequency!
        if (!N_BIT(byte, i))
            *current_freq ^= AX25_AFSK1200_1200HZ ^ AX25_AFSK1200_2200HZ;

        float freq = (float)*current_freq;
        ESP_ERROR_VALIDATE("AX25 Bit",
                           err,
                           radio_transmitter_write_pulse(freq, AX25_AFSK1200_BIT_MS, ax25_buf))
    }
    return ESP_OK;
}

esp_err_t ax25_generate_waveform(uint8_t *data, uint8_t data_len, radio_waveform_data_t *ax25_buf)
{
    // Init error.
    esp_err_t err = ESP_OK;

    uint16_t ax25_current_freq = AX25_AFSK1200_1200HZ;

    // Flag bytes (preamble).
    for (uint8_t i = 0; i < AX25_AFSK1200_PREAMBLE_BYTES; ++i)
    {
        ESP_ERROR_VALIDATE("FLAG BYTE",
                           err,
                           ax25_byte(AX25_FLAG, &ax25_current_freq, ax25_buf))
    }

    // Generate destination address information.
    uint8_t callsign[HAM_CALLSIGN_LEN] = HAM_CALLSIGN;
    for (uint8_t i = 0; i < HAM_CALLSIGN_LEN; ++i)
    {
        ESP_ERROR_VALIDATE("DEST CALLSIGN",
                           err,
                           ax25_byte(callsign[i], &ax25_current_freq, ax25_buf))
    }

    ESP_ERROR_VALIDATE("DEST SSID",
                       err,
                       ax25_byte(HAM_SSID, &ax25_current_freq, ax25_buf))

    // Generate source address information.
    for (uint8_t i = 0; i < HAM_CALLSIGN_LEN; ++i)
    {
        ESP_ERROR_VALIDATE("SRC CALLSIGN",
                           err,
                           ax25_byte(callsign[i], &ax25_current_freq, ax25_buf))
    }

    ESP_ERROR_VALIDATE("SRC SSID",
                       err,
                       ax25_byte(HAM_SSID, &ax25_current_freq, ax25_buf))

    // This implementation does not use the digipeater!

    // Control field.
    ESP_ERROR_VALIDATE("CTRL FIELD",
                       err,
                       ax25_byte(AX25_CTRL_FIELD, &ax25_current_freq, ax25_buf))

    // Protocol ID.
    ESP_ERROR_VALIDATE("PROTOCOL ID",
                       err,
                       ax25_byte(AX25_PROTOCOL_ID, &ax25_current_freq, ax25_buf))

    // Write data.
    for (uint8_t i = 0; i < data_len; ++i)
    {
        ESP_ERROR_VALIDATE("DATA",
                           err,
                           ax25_byte(data[i], &ax25_current_freq, ax25_buf))
    }

    // FCS.
    uint16_t fcs_checksum = 0;
    ESP_ERROR_VALIDATE("FCS Generation",
                       err,
                       ax25_iso_3099_fcs_checksum(data, data_len, &fcs_checksum))

    uint8_t *fcs = (uint8_t *)&fcs_checksum;
    for (int i = 0; i < 2; ++i)
    {
        ESP_ERROR_VALIDATE("FCS BYTE",
                           err,
                           ax25_byte(fcs[i], &ax25_current_freq, ax25_buf))
    }

    // Flag bytes (rest).
    for (uint8_t i = 0; i < AX25_AFSK1200_REST_BYTES; ++i)
    {
        ESP_ERROR_VALIDATE("FLAG BYTE",
                           err,
                           ax25_byte(AX25_FLAG, &ax25_current_freq, ax25_buf))
    }

    return ESP_OK;
}
