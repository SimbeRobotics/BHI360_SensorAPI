/**
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file    bhi360_bsec_param.c
 * @brief   Example to set/get BSEC parameters
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "bhi360.h"
#include "bhi360_parse.h"
#include "common.h"
#include "bhi360_bsec_param.h"
#include "bhi360_param_defs.h"

#include "bhi360/Bosch_Shuttle3_BHI360_BMM350C_BME688_IAQ.fw.h"

static void print_api_error(int8_t rslt, struct bhi360_dev *dev);
static int8_t upload_firmware(struct bhi360_dev *dev);

enum bhi360_intf intf;

int main(void)
{
    uint8_t i;
    uint8_t chip_id = 0;
    uint16_t version = 0;
    int8_t rslt;
    struct bhi360_dev bhy;
    uint8_t hintr_ctrl, hif_ctrl, boot_status;
    bhi360_bsec_param_algo_state state;
    union bhi360_float_conv temp_offset;
    bhi360_bsec_param_sample_rate sample_rate;
    float sample_rate_act[3] = { 1.0, 0.33333, 0.0033333 };

#ifdef BHI360_USE_I2C
    intf = BHI360_I2C_INTERFACE;
#else
    intf = BHI360_SPI_INTERFACE;
#endif

    setup_interfaces(true, intf); /* Perform a power on reset */

#ifdef BHI360_USE_I2C
    rslt = bhi360_init(BHI360_I2C_INTERFACE,
                       bhi360_i2c_read,
                       bhi360_i2c_write,
                       bhi360_delay_us,
                       BHI360_RD_WR_LEN,
                       NULL,
                       &bhy);
#else
    rslt = bhi360_init(BHI360_SPI_INTERFACE,
                       bhi360_spi_read,
                       bhi360_spi_write,
                       bhi360_delay_us,
                       BHI360_RD_WR_LEN,
                       NULL,
                       &bhy);
#endif
    print_api_error(rslt, &bhy);

    rslt = bhi360_soft_reset(&bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi360_get_chip_id(&chip_id, &bhy);
    print_api_error(rslt, &bhy);

    /* Check for a valid Chip ID */
    if (chip_id == BHI360_CHIP_ID)
    {
        printf("Chip ID read 0x%X\r\n", chip_id);
    }
    else
    {
        printf("Device not found. Chip ID read 0x%X\r\n", chip_id);
    }

    /* Check the interrupt pin and FIFO configurations. Disable status and debug */
    hintr_ctrl = BHI360_ICTL_DISABLE_STATUS_FIFO | BHI360_ICTL_DISABLE_DEBUG;

    rslt = bhi360_set_host_interrupt_ctrl(hintr_ctrl, &bhy);
    print_api_error(rslt, &bhy);
    rslt = bhi360_get_host_interrupt_ctrl(&hintr_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    printf("Host interrupt control\r\n");
    printf("    Wake up FIFO %s.\r\n", (hintr_ctrl & BHI360_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
    printf("    Non wake up FIFO %s.\r\n", (hintr_ctrl & BHI360_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
    printf("    Status FIFO %s.\r\n", (hintr_ctrl & BHI360_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
    printf("    Debugging %s.\r\n", (hintr_ctrl & BHI360_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
    printf("    Fault %s.\r\n", (hintr_ctrl & BHI360_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
    printf("    Interrupt is %s.\r\n", (hintr_ctrl & BHI360_ICTL_ACTIVE_LOW) ? "active low" : "active high");
    printf("    Interrupt is %s triggered.\r\n", (hintr_ctrl & BHI360_ICTL_EDGE) ? "pulse" : "level");
    printf("    Interrupt pin drive is %s.\r\n", (hintr_ctrl & BHI360_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");

    /* Configure the host interface */
    hif_ctrl = 0;
    rslt = bhi360_set_host_intf_ctrl(hif_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    /* Check if the sensor is ready to load firmware */
    rslt = bhi360_get_boot_status(&boot_status, &bhy);
    print_api_error(rslt, &bhy);

    if (boot_status & BHI360_BST_HOST_INTERFACE_READY)
    {
        uint8_t sensor_error;
        int8_t temp_rslt;
        printf("Loading firmware.\r\n");

        rslt = upload_firmware(&bhy);
        temp_rslt = bhi360_get_error_value(&sensor_error, &bhy);
        if (sensor_error)
        {
            printf("%s\r\n", get_sensor_error_text(sensor_error));
        }

        print_api_error(rslt, &bhy);
        print_api_error(temp_rslt, &bhy);

        printf("Booting from RAM.\r\n");
        rslt = bhi360_boot_from_ram(&bhy);

        temp_rslt = bhi360_get_error_value(&sensor_error, &bhy);
        if (sensor_error)
        {
            printf("%s\r\n", get_sensor_error_text(sensor_error));
        }

        print_api_error(rslt, &bhy);
        print_api_error(temp_rslt, &bhy);

        rslt = bhi360_get_kernel_version(&version, &bhy);
        print_api_error(rslt, &bhy);
        if ((rslt == BHI360_OK) && (version != 0))
        {
            printf("Boot successful. Kernel version %u.\r\n", version);
        }

        printf("Get BSEC algorithm state.\r\n");
        rslt = bhi360_bsec_param_get_algo_state(&state, &bhy);
        print_api_error(rslt, &bhy);
        for (i = 0U; i < BHI360_BSEC_PARAM_BSEC_ALGO_STATE_LENGTH; i++)
        {
            printf("%u ", state.algo_state[i]);
        }

        printf("\r\n");
        printf("Change BSEC algorithm state.\r\n");
        state.algo_state[0] = 1;
        rslt = bhi360_bsec_param_set_algo_state(&state, &bhy);
        print_api_error(rslt, &bhy);
        printf("Get BSEC algorithm state again.\r\n");
        rslt = bhi360_bsec_param_get_algo_state(&state, &bhy);
        print_api_error(rslt, &bhy);
        for (i = 0U; i < BHI360_BSEC_PARAM_BSEC_ALGO_STATE_LENGTH; i++)
        {
            printf("%u ", state.algo_state[i]);
        }

        printf("\r\n");

        printf("Get temperature offset.\r\n");
        rslt = bhi360_bsec_param_get_temp_offset(&temp_offset, &bhy);
        print_api_error(rslt, &bhy);
        printf("    Temperature offset: %f\r\n", temp_offset.f_val);
        printf("Change temperature offset.\r\n");
        temp_offset.f_val = 1.0;
        rslt = bhi360_bsec_param_set_temp_offset(&temp_offset, &bhy);
        print_api_error(rslt, &bhy);
        printf("Get temperature offset again.\r\n");
        rslt = bhi360_bsec_param_get_temp_offset(&temp_offset, &bhy);
        print_api_error(rslt, &bhy);
        printf("    Temperature offset: %f\r\n", temp_offset.f_val);

        printf("Get sample rate.\r\n");
        rslt = bhi360_bsec_param_get_sample_rate(&sample_rate, &bhy);
        print_api_error(rslt, &bhy);
        printf("    Sample rate: %f\r\n", sample_rate_act[sample_rate.sample_rate_index]);
        printf("Change sample rate to 1.0.\r\n");
        sample_rate.sample_rate_index = BSEC_PARAM_SAMPLE_RATE_CONT;
        rslt = bhi360_bsec_param_set_sample_rate(&sample_rate, &bhy);
        print_api_error(rslt, &bhy);
        printf("Get sample rate again.\r\n");
        rslt = bhi360_bsec_param_get_sample_rate(&sample_rate, &bhy);
        print_api_error(rslt, &bhy);
        printf("    Sample rate: %f\r\n", sample_rate_act[sample_rate.sample_rate_index]);
    }
    else
    {
        printf("Host interface not ready. Exiting\r\n");

        close_interfaces(intf);

        return 0;
    }

    close_interfaces(intf);

    return rslt;
}

static void print_api_error(int8_t rslt, struct bhi360_dev *dev)
{
    if (rslt != BHI360_OK)
    {
        printf("%s\r\n", get_api_error(rslt));
        if ((rslt == BHI360_E_IO) && (dev != NULL))
        {
            printf("%s\r\n", get_coines_error(dev->hif.intf_rslt));
            dev->hif.intf_rslt = BHI360_INTF_RET_SUCCESS;
        }

        exit(0);
    }
}

static int8_t upload_firmware(struct bhi360_dev *dev)
{
    uint32_t incr = 256; /* Max command packet size */
    uint32_t len = sizeof(bhi360_firmware_image);
    int8_t rslt = BHI360_OK;

    if ((incr % 4) != 0) /* Round off to higher 4 bytes */
    {
        incr = ((incr >> 2) + 1) << 2;
    }

    for (uint32_t i = 0; (i < len) && (rslt == BHI360_OK); i += incr)
    {
        if (incr > (len - i)) /* If last payload */
        {
            incr = len - i;
            if ((incr % 4) != 0) /* Round off to higher 4 bytes */
            {
                incr = ((incr >> 2) + 1) << 2;
            }
        }

        rslt = bhi360_upload_firmware_to_ram_partly(&bhi360_firmware_image[i], len, i, incr, dev);

        printf("%.2f%% complete\r", (float)(i + incr) / (float)len * 100.0f);
    }

    printf("\n");

    return rslt;
}
