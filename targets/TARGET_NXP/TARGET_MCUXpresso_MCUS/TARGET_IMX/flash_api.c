/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "flash_api.h"

#if DEVICE_FLASH

#include "fsl_flexspi.h"
#include "flash_defines.h"

static bool flash_inited = false;

void flexspi_update_lut(void)
{
    flexspi_config_t config;

    __asm("cpsid i");
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    /*Get FLEXSPI default settings and configure the flexspi. */
    FLEXSPI_GetDefaultConfig(&config);

    /*Set AHB buffer size for reading data through AHB bus. */
    config.ahbConfig.enableAHBPrefetch = true;
    /*Allow AHB read start address do not follow the alignment requirement. */
    config.ahbConfig.enableReadAddressOpt = true;
    config.ahbConfig.enableAHBBufferable  = true;
    config.ahbConfig.enableAHBCachable    = true;
    /* enable diff clock and DQS */
    config.enableSckBDiffOpt = true;
    config.rxSampleClock     = kFLEXSPI_ReadSampleClkExternalInputFromDqsPad;
    config.enableCombination = true;
    config.enableDoze = false;

    FLEXSPI_Init(FLEXSPI, &config);

    /* Configure flash settings according to serial flash feature. */
    FLEXSPI_SetFlashConfig(FLEXSPI, &deviceconfig, kFLEXSPI_PortA1);

    /* Update LUT table. */
    FLEXSPI_UpdateLUT(FLEXSPI, 0, customLUT, CUSTOM_LUT_LENGTH);

    FLEXSPI_SoftwareReset(FLEXSPI);

    __asm("cpsie i");
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

status_t flexspi_nor_write_enable(uint32_t baseAddr)
{
    flexspi_transfer_t flashXfer;
    status_t status = kStatus_Success;

    /* Write enable */
    flashXfer.deviceAddress = baseAddr;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Command;
    flashXfer.SeqNumber     = 2;
    flashXfer.seqIndex      = HYPERFLASH_CMD_LUT_SEQ_IDX_WRITEENABLE;

    status = FLEXSPI_TransferBlocking(FLEXSPI, &flashXfer);

    return status;
}

status_t flexspi_nor_wait_bus_busy(void)
{
    /* Wait status ready. */
    bool isBusy = false;
    uint32_t readValue = 0;
    status_t status = kStatus_Success;
    flexspi_transfer_t flashXfer;

    flashXfer.deviceAddress = 0;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Read;
    flashXfer.SeqNumber     = 2;
    flashXfer.seqIndex      = HYPERFLASH_CMD_LUT_SEQ_IDX_READSTATUS;
    flashXfer.data          = &readValue;
    flashXfer.dataSize      = 2;

    do {
        status = FLEXSPI_TransferBlocking(FLEXSPI, &flashXfer);

        if (status != kStatus_Success) {
            return status;
        }

        if (readValue & 0x8000) {
            isBusy = false;
        } else {
            isBusy = true;
        }

        if (readValue & 0x3200) {
            status = kStatus_Fail;
            break;
        }
    } while (isBusy);

    return status;

}

status_t flexspi_nor_flash_erase_sector(uint32_t address)
{
    status_t status = kStatus_Success;
    flexspi_transfer_t flashXfer;

    __asm("cpsid i");
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    /* Write enable */
    status = flexspi_nor_write_enable(address);
    if (status != kStatus_Success) {
        goto exit_erase;
    }

    flashXfer.deviceAddress = address;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Command;
    flashXfer.SeqNumber     = 4;
    flashXfer.seqIndex      = HYPERFLASH_CMD_LUT_SEQ_IDX_ERASESECTOR;

    status = FLEXSPI_TransferBlocking(FLEXSPI, &flashXfer);
    if (status != kStatus_Success) {
        goto exit_erase;
    }

    status = flexspi_nor_wait_bus_busy();

    if (status != kStatus_Success) {
        goto exit_erase;
    }

    /* Do software reset. */
    FLEXSPI_SoftwareReset(FLEXSPI);

    SCB_InvalidateDCache_by_Addr((void *)(address + FlexSPI_AMBA_BASE), BOARD_FLASH_SECTOR_SIZE);

exit_erase:
    __asm("cpsie i");
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    /* Flush pipeline to allow pending interrupts take place
     * before starting next loop */
    __ISB();

    return status;
}


status_t flexspi_nor_flash_page_program(uint32_t address, const uint32_t *src)
{
    status_t status = kStatus_Success;
    flexspi_transfer_t flashXfer;

    __asm("cpsid i");
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    FLEXSPI_Enable(FLEXSPI, false);
    CCM->CCGR6 &= ~CCM_CCGR6_CG5_MASK;

    /* The clock should be max 50MHz during programming */
    /* Backup of CCM_ANALOG_PFD_480 register */
    uint32_t pfd480;
    pfd480 = CCM_ANALOG->PFD_480;
    /* Disable the clock output first */
    CCM_ANALOG->PFD_480 |= CCM_ANALOG_PFD_480_PFD0_CLKGATE_MASK;
    /* Set value of PFD0_FRAC to 26 - clock 332MHz */
    CCM_ANALOG->PFD_480 &= ~CCM_ANALOG_PFD_480_PFD0_FRAC_MASK;
    CCM_ANALOG->PFD_480 |= CCM_ANALOG_PFD_480_PFD0_FRAC(26);
    /* Enable output */
    CCM_ANALOG->PFD_480 &= ~CCM_ANALOG_PFD_480_PFD0_CLKGATE_MASK;

    /* Backup of CCM_CSCMR1 register */
    uint32_t cscmr1;
    cscmr1 = CCM->CSCMR1;
    /* Set value of FLEXSPI_CLK_SEL to 3 - derive clock from PLL3 PFD0 */
    CCM->CSCMR1 |= CCM_CSCMR1_FLEXSPI_CLK_SEL(3);
    /* Set value of FLEXSPI_PODF to 3 - divide by 4, flexspi clock 83MHz, in DDR mode is half clock frequency on SCK - 42MHz */
    CCM->CSCMR1 &= ~CCM_CSCMR1_FLEXSPI_PODF_MASK;
    CCM->CSCMR1 |= CCM_CSCMR1_FLEXSPI_PODF(3);

    CCM->CCGR6 |= CCM_CCGR6_CG5_MASK;

    FLEXSPI_Enable(FLEXSPI, true);

    /* Do software reset. */
    FLEXSPI_SoftwareReset(FLEXSPI);

    /* Write enable */
    status = flexspi_nor_write_enable(address);

    if (status != kStatus_Success) {
        goto exit_program;
    }

    /* Prepare page program command */
    flashXfer.deviceAddress = address;
    flashXfer.port          = kFLEXSPI_PortA1;
    flashXfer.cmdType       = kFLEXSPI_Write;
    flashXfer.SeqNumber     = 2;
    flashXfer.seqIndex      = HYPERFLASH_CMD_LUT_SEQ_IDX_PAGEPROGRAM;
    flashXfer.data          = (uint32_t *)(src);
    flashXfer.dataSize      = BOARD_FLASH_PAGE_SIZE;

    status = FLEXSPI_TransferBlocking(FLEXSPI, &flashXfer);

    if (status != kStatus_Success) {
        goto exit_program;
    }

    status = flexspi_nor_wait_bus_busy();

    if (status != kStatus_Success) {
        goto exit_program;
    }

    FLEXSPI_Enable(FLEXSPI, false);
    CCM->CCGR6 &= ~CCM_CCGR6_CG5_MASK;

    /* Return back the changes in clocks */
    CCM_ANALOG->PFD_480 = pfd480;
    CCM->CSCMR1 = cscmr1;

    CCM->CCGR6 |= CCM_CCGR6_CG5_MASK;

    FLEXSPI_Enable(FLEXSPI, true);

    /* Do software reset. */
    FLEXSPI_SoftwareReset(FLEXSPI);

    SCB_InvalidateDCache_by_Addr((void *)(address + FlexSPI_AMBA_BASE), BOARD_FLASH_PAGE_SIZE);

exit_program:
    __asm("cpsie i");
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    /* Flush pipeline to allow pending interrupts take place
     * before starting next loop */
    __ISB();

    return status;
}

int32_t flash_init(flash_t *obj)
{
    if (!flash_inited) {
        flexspi_update_lut();
    }

    flash_inited = true;
    return 0;
}

int32_t flash_erase_sector(flash_t *obj, uint32_t address)
{
    status_t status = kStatus_Success;
    int32_t ret = 0;

    status = flexspi_nor_flash_erase_sector(address - FlexSPI_AMBA_BASE);

    if (status != kStatus_Success) {
        ret = -1;
    }

    return ret;
}

int32_t flash_program_page(flash_t *obj, uint32_t address, const uint8_t *data, uint32_t size)
{
    status_t status = kStatus_Success;
    uint32_t offset = 0;
    int32_t ret = 0;

    while (size > 0) {
        status = flexspi_nor_flash_page_program(address + offset - FlexSPI_AMBA_BASE,
                                                    (uint32_t *)(data + offset));

        if (status != kStatus_Success) {
            ret = -1;
            break;
        }

        size -= BOARD_FLASH_PAGE_SIZE;
        offset += BOARD_FLASH_PAGE_SIZE;
    }

    return ret;
}

int32_t flash_free(flash_t *obj)
{
    return 0;
}

uint32_t flash_get_sector_size(const flash_t *obj, uint32_t address)
{
    uint32_t sectorsize = MBED_FLASH_INVALID_SIZE;
    uint32_t devicesize = BOARD_FLASH_SIZE;
    uint32_t startaddr = BOARD_FLASH_START_ADDR;

    if ((address >= startaddr) && (address < (startaddr + devicesize))) {
        sectorsize = BOARD_FLASH_SECTOR_SIZE;
    }

    return sectorsize;
}

uint32_t flash_get_page_size(const flash_t *obj)
{
    return BOARD_FLASH_PAGE_SIZE;
}

uint32_t flash_get_start_address(const flash_t *obj)
{
    return BOARD_FLASH_START_ADDR;
}

uint32_t flash_get_size(const flash_t *obj)
{
    return BOARD_FLASH_SIZE;
}

uint8_t flash_get_erase_value(const flash_t *obj)
{
    (void)obj;

    return 0xFF;
}

#endif //DEVICE_FLASH

