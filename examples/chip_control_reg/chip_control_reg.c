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
 * @file    chip_control_reg.c
 * @brief   Example usage of chip control register
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "bhi360.h"
#include "bhi360_parse.h"
#include "common.h"

#include "bhi360/Bosch_Shuttle3_BHI360_BMM350C_BMP580_BME688.fw.h"

#define WORK_BUFFER_SIZE  UINT16_C(2048)

/*! @brief Prints API error code.
 *
 *  @param[in] rslt      : API Error code.
 *  @param[in] dev       : Device reference.
 */
static void print_api_error(int8_t rslt, struct bhi360_dev *dev);

/*! @brief Loads BHy firmware image to BHy ram.
 *
 *  @param[in] boot_stat : Boot status.
 *  @param[in] dev       : Device reference.
 */
static void upload_firmware(uint8_t boot_stat, struct bhi360_dev *dev);

enum bhi360_intf intf;

int main(void)
{
    uint8_t chip_id = 0;
    uint16_t version = 0;
    int8_t rslt;
    struct bhi360_dev bhy;
    uint8_t hif_ctrl, boot_status, hintr_ctrl;
    uint8_t error_reg_value = 0, chip_ctrl_reg_value = 0;

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

    /* Configure the host interface */
    hif_ctrl = BHI360_HIF_CTRL_ASYNC_STATUS_CHANNEL;
    rslt = bhi360_set_host_intf_ctrl(hif_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    /* Check the interrupt pin and FIFO configurations. Disable status and debug */
    hintr_ctrl = BHI360_ICTL_DISABLE_STATUS_FIFO | BHI360_ICTL_DISABLE_DEBUG;

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

    /* Check if the sensor is ready to load firmware */
    rslt = bhi360_get_boot_status(&boot_status, &bhy);
    print_api_error(rslt, &bhy);

    if (boot_status & BHI360_BST_HOST_INTERFACE_READY)
    {
        printf("Uploading customized firmware to trigger sensor error.\r\n");
        upload_firmware(boot_status, &bhy);

        rslt = bhi360_get_kernel_version(&version, &bhy);
        print_api_error(rslt, &bhy);
        if ((rslt == BHI360_OK) && (version != 0))
        {
            printf("Boot successful. Kernel version %u.\r\n", version);
        }
    }
    else
    {
        printf("Host interface not ready. Exiting\r\n");

        close_interfaces(intf);

        return 0;
    }

    /*! Check the Error Status */
    printf("Read Error Value Register(0x2E) \r\n");
    rslt = bhi360_get_regs(BHI360_REG_ERROR_VALUE, &error_reg_value, 1, &bhy);
    print_api_error(rslt, &bhy);

    printf("Error Register Value : %x\r\n", error_reg_value);
    if (error_reg_value == BHI360_ERR_NO_RESPONSE_FROM_DEVICE)
    {
        printf("Sensor Init Failed: No Response from Device\r\n");
    }

    /*! Read the Chip Control Register */
    rslt = bhi360_get_regs(BHI360_REG_CHIP_CTRL, &chip_ctrl_reg_value, 1, &bhy);

    /*! Configure Chip Register to Clear Error and Debug Registers */
    printf("Clearing Error Register\r\n");
    uint8_t clr_err = chip_ctrl_reg_value | BHI360_CHIP_CTRL_CLR_ERR_REG;
    rslt = bhi360_set_regs(BHI360_REG_CHIP_CTRL, &clr_err, 1, &bhy); /* When written with a 1, the error and debug
                                                                   * registers are cleared */

    /*! Check the Error Status */
    rslt = bhi360_get_regs(BHI360_REG_ERROR_VALUE, &error_reg_value, 1, &bhy);
    print_api_error(rslt, &bhy);

    printf("Error Register Value : %x\r\n", error_reg_value);
    if (error_reg_value == BHI360_ERR_NO_ERROR)
    {
        printf("Error Value Register Reported Error Cleared\r\n");
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

static void upload_firmware(uint8_t boot_stat, struct bhi360_dev *dev)
{
    uint8_t sensor_error;
    int8_t temp_rslt;
    int8_t rslt = BHI360_OK;

    printf("Loading firmware into RAM.\r\n");
    rslt = bhi360_upload_firmware_to_ram(bhi360_firmware_image, sizeof(bhi360_firmware_image), dev);

    temp_rslt = bhi360_get_error_value(&sensor_error, dev);
    if (sensor_error)
    {
        printf("%s\r\n", get_sensor_error_text(sensor_error));
    }

    print_api_error(rslt, dev);
    print_api_error(temp_rslt, dev);

    printf("Booting from RAM.\r\n");
    rslt = bhi360_boot_from_ram(dev);

    temp_rslt = bhi360_get_error_value(&sensor_error, dev);
    if (sensor_error)
    {
        printf("%s\r\n", get_sensor_error_text(sensor_error));
    }

    print_api_error(rslt, dev);
    print_api_error(temp_rslt, dev);
}
