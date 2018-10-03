/*
 * Copyright (c) 2018 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file
 * This is the source file for the real time clock sensor PCF2123 driver.
 */

#include "pcf2123.h"
#include <string.h>
#include <assert.h>

/* REGISTERS */
#define PCF2123_REG_CTRL1       (0x00)    /* Control Register 1 */
#define PCF2123_REG_CTRL2       (0x01)    /* Control Register 2 */
#define PCF2123_REG_SC          (0x02)    /* datetime */
#define PCF2123_REG_MN          (0x03)
#define PCF2123_REG_HR          (0x04)
#define PCF2123_REG_DM          (0x05)
#define PCF2123_REG_DW          (0x06)
#define PCF2123_REG_MO          (0x07)
#define PCF2123_REG_YR          (0x08)
#define PCF2123_REG_ALRM_MN     (0x09)    /* Alarm Registers */
#define PCF2123_REG_ALRM_HR     (0x0a)
#define PCF2123_REG_ALRM_DM     (0x0b)
#define PCF2123_REG_ALRM_DW     (0x0c)
#define PCF2123_REG_OFFSET      (0x0d)    /* Clock Rate Offset Register */
#define PCF2123_REG_TMR_CLKOUT  (0x0e)    /* Timer Registers */
#define PCF2123_REG_CTDWN_TMR   (0x0f)

/* PCF2123_REG_CTRL1 BITS */
#define CTRL1_CLEAR             (0)                         /* Clear */
#define CTRL1_CORR_INT          (1<<1)                      /* Correction irq enable */
#define CTRL1_12_HOUR           (1<<2)                      /* 12 hour time */
#define CTRL1_SW_RESET          (1<<6) | (1<<4) | (1<<3)    /* Software reset */
#define CTRL1_STOP              (1<<5)                      /* Stop the clock */
#define CTRL1_EXT_TEST          (1<<7)                      /* External clock test mode */

/* PCF2123_REG_CTRL2 BITS */
#define CTRL2_TIE       (1)         /* Countdown timer irq enable */
#define CTRL2_AIE       (1<<1)      /* Alarm irq enable */
#define CTRL2_TF        (1<<2)      /* Countdown timer flag */
#define CTRL2_AF        (1<<3)      /* Alarm flag */
#define CTRL2_TI_TP     (1<<4)      /* Irq pin generates pulse */
#define CTRL2_MSF       (1<<5)      /* Minute or second irq flag */
#define CTRL2_SI        (1<<6)      /* Second irq enable */
#define CTRL2_MI        (1<<7)      /* Minute irq enable */

/* PCF2123_REG_SC BITS */
#define OSC_HAS_STOPPED      (1<<7)    /* Clock has been stopped */

/* PCF2123_REG_ALRM_XX BITS */
#define ALARM_ENABLE         (1<<7)    /* MN, HR, DM, or DW alarm enable */

/* PCF2123_REG_TMR_CLKOUT BITS */
#define CD_TMR_TE            (1<<3)    /* Countdown timer enable */

/* PCF2123_REG_OFFSET BITS */
#define OFFSET_SIGN_BIT       6         /* 2's complement sign bit */
#define OFFSET_COARSE         (1<<7)    /* Coarse mode offset */
#define OFFSET_STEP           (2170)    /* Offset step in parts per billion */

/* PCF2123 MASKS */
#define COF_MASK              0x70      // bits 6 to 4
#define CTD_MASK              0x03      // bits 1 to 0

/* PCF2123 SHIFTS */
#define COF_SHIFT             4
#define ALARM_EN_SHIFT        7

/*****************************************************************************
 * Variables
 ****************************************************************************/
static pcf2123_IoFunc_t sIoFunc;
static settingsPCF_t settings;
static bool initDriverDone = false;
static bool initHwDone = false;

/*****************************************************************************
 * Private functions
 ****************************************************************************/
static uint8_t PCF_SoftReset();

// software reset
uint8_t PCF_SoftReset()
{
    uint8_t wBuf[2] = {PCF2123_WRITE | PCF2123_REG_CTRL1, CTRL1_SW_RESET};

    // Reset all registers to POR value
    // 32.768 kHz on pin CLKOUT active, 24 hour mode is selected, Offset register is set to 0
    // No alarms set, Timer disabled, No interrupts enabled
    if (sIoFunc.SPI_Write(wBuf, 2) != 0) return PCF2123_I2C_ERROR;
    else return PCF2123_SUCCESS;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

// wrap the low level function (I2C write, I2C read, WaitMsec) required by PCF2123 driver
// this function does not initialize the HW
void PCF2123_Init_Driver(pcf2123_IoFunc_t* pIoFunc){
    assert((pIoFunc != NULL) &&
            (pIoFunc->SPI_Read != NULL) &&
            (pIoFunc->SPI_Write != NULL) &&
            (pIoFunc->WaitMsec != NULL));
    sIoFunc = *pIoFunc;
    initDriverDone = true;
}

void PCF2123_Deinit_Driver(){
    if (initHwDone)
    {
        /* Deinit HW */
        // nothing to do as already very low consumption
    }
    /* Deinit driver */
    initDriverDone = false;
    initHwDone = false;
}

// Initialize the sensor
pcf2123_status_t PCF2123_Init_Hw(const settingsPCF_t* pcfSettings)
{
    pcf2123_status_t status = PCF2123_SUCCESS;

    if (!initDriverDone) return PCF2123_INIT_ERROR;
    assert(pcfSettings != NULL);

    /* trick to allow calling internal public functions */
    initHwDone = true;

    // initialize internal structures which will be used from now on
    memcpy( (void*)&settings, (void*)pcfSettings, sizeof(settings) );

    if (settings.Softreset == true)
    {
        // Reset all registers to POR values
        status = PCF_SoftReset();
        if (status != PCF2123_SUCCESS) goto return_status;
    }

    // Set Date and Time
    status = PCF2123_SetDateTime(&settings);
    if (status != PCF2123_SUCCESS) goto return_status;

    // Set Clock output pin frequency
    status = PCF2123_SetClockOutputFreq(settings.clockOutputFreq);
    if (status != PCF2123_SUCCESS) goto return_status;

    // Set/reset Minute & second interrupt
    status = PCF2123_SetMinSecInterrupt(settings.MinInterrupt, settings.SecInterrupt, settings.PulseInterrupt);
    if (status != PCF2123_SUCCESS) goto return_status;

    return_status:
    if (status != PCF2123_SUCCESS) initHwDone = false;
    return status;
}

// Read all the chip registers
pcf2123_status_t PCF2123_Dump(uint8_t *pcfReg)
{
    uint8_t wBuf[17];

    if (!initHwDone) return PCF2123_INIT_ERROR;
    assert(pcfReg != NULL);

    wBuf[0] = PCF2123_READ | PCF2123_REG_CTRL1;    // read + sub address;    // Start reading at register address 0
    // read all registers
    if (sIoFunc.SPI_Read(wBuf, pcfReg, 16) != 0) return PCF2123_I2C_ERROR; // Read all 16 registers

    return PCF2123_SUCCESS;
}

// Read Date and Time
pcf2123_status_t PCF2123_GetDateTime(settingsPCF_t* pcfDateTime)
{
    uint8_t wBuf[8] = {0};
    uint8_t rBuf[8] = {0};

    if (!initHwDone) return PCF2123_INIT_ERROR;
    assert(pcfDateTime != NULL);

    wBuf[0] = PCF2123_READ | (PCF2123_REG_SC);    // read + sub address;    // Start reading at register address 0
    // read all registers
    if (sIoFunc.SPI_Read(wBuf, rBuf, 7) != 0) return PCF2123_I2C_ERROR; // Read all 16 registers
    pcfDateTime->seconds = rBuf[1];
    pcfDateTime->minutes = rBuf[2];
    pcfDateTime->hours = rBuf[3];
    pcfDateTime->days =rBuf[4];
    pcfDateTime->weekdays = rBuf[5];
    pcfDateTime->months = rBuf[6];
    pcfDateTime->years =rBuf[7];

    return PCF2123_SUCCESS;
}

// Set Date and Time
pcf2123_status_t PCF2123_SetDateTime(settingsPCF_t* pcfDateTime)
{
    uint8_t wBuf[8] = {0};
    uint8_t rBuf[8] = {0};

    if (!initHwDone) return PCF2123_INIT_ERROR;
    assert(pcfDateTime != NULL);

    // set STOP bit <=> stop RTC clock before updating date and time
    wBuf[0] = PCF2123_READ | PCF2123_REG_CTRL1;
    if (sIoFunc.SPI_Read(wBuf, rBuf, 1) != 0) return PCF2123_I2C_ERROR;    // read REG_CTRL1

    wBuf[0] = PCF2123_WRITE | PCF2123_REG_CTRL1;
    wBuf[1] = rBuf[1] | CTRL1_STOP;                                        // Set STOP bit
    wBuf[1] = (pcfDateTime->mode12_24 == CTRL1_12_HOUR) ? wBuf[1]|CTRL1_12_HOUR : wBuf[1]&~CTRL1_12_HOUR ;    // Set 12_24H bit
    if (sIoFunc.SPI_Write(wBuf, 2) != 0) return PCF2123_I2C_ERROR;        // Write it back

    // set the date and time
    wBuf[0] = PCF2123_WRITE | PCF2123_REG_SC;
    wBuf[1] = pcfDateTime->seconds;
    wBuf[2] = pcfDateTime->minutes;
    wBuf[3] = pcfDateTime->hours;
    wBuf[4] = pcfDateTime->days;
    wBuf[5] = pcfDateTime->weekdays;
    wBuf[6] = pcfDateTime->months;
    wBuf[7] = pcfDateTime->years;
    if (sIoFunc.SPI_Write(wBuf, 8) != 0) return PCF2123_I2C_ERROR;        // Write it back

    // reset STOP bit <=> restart RTC clock
    wBuf[0] = PCF2123_READ | PCF2123_REG_CTRL1;
    if (sIoFunc.SPI_Read(wBuf, rBuf, 1) != 0) return PCF2123_I2C_ERROR;    // read REG_CTRL1

    wBuf[0] = PCF2123_WRITE | PCF2123_REG_CTRL1;
    wBuf[1] = rBuf[1]  &~CTRL1_STOP ;            // reset STOP bit
    if (sIoFunc.SPI_Write(wBuf, 2) != 0) return PCF2123_I2C_ERROR;        // Write it back

    return PCF2123_SUCCESS;
}

// Set Clock output pin frequency from 1Hz up to 32768Hz, or disabled = High Z
pcf2123_status_t PCF2123_SetClockOutputFreq(clockOutputPCF_t pcfClockOutputFreq)
{
    uint8_t wBuf[2] = {0};
    uint8_t rBuf[2] = {0};

    if (!initHwDone) return PCF2123_INIT_ERROR;

    // read register Timer_clkout and set COF[2:0] field = clock output frequency
    wBuf[0] = PCF2123_READ | PCF2123_REG_TMR_CLKOUT;
    if (sIoFunc.SPI_Read(wBuf, rBuf, 1) != 0) return PCF2123_I2C_ERROR;

    wBuf[0] = PCF2123_WRITE | PCF2123_REG_TMR_CLKOUT;
    wBuf[1] = (rBuf[1] & ~COF_MASK) | (pcfClockOutputFreq << COF_SHIFT);    // Set COF[2:0]
    if (sIoFunc.SPI_Write(wBuf, 2) != 0) return PCF2123_I2C_ERROR;

    return PCF2123_SUCCESS;
}

// Set Countdown interrupt
// interrupt generated when countdown flag set
pcf2123_status_t PCF2123_SetCountdownInterrupt(bool Countdown_Int)
{
    uint8_t wBuf[2] = {0};
    uint8_t rBuf[2] = {0};

    if (!initHwDone) return PCF2123_INIT_ERROR;

    // read register CTRL_2 and reset TF bit
    wBuf[0] = PCF2123_READ | PCF2123_REG_CTRL2;
    if (sIoFunc.SPI_Read(wBuf, rBuf, 1) != 0) return PCF2123_I2C_ERROR;

    wBuf[0] = PCF2123_WRITE | PCF2123_REG_CTRL2;
    wBuf[1] = (Countdown_Int == true) ? wBuf[1] | CTRL2_TIE : wBuf[1] & ~CTRL2_TIE;        // Set/Reset countdown interrupt
    if (sIoFunc.SPI_Write(wBuf, 2) != 0) return PCF2123_I2C_ERROR;

    return PCF2123_SUCCESS;
}

// Set Countdown timer source clock.
pcf2123_status_t PCF2123_SetCountdownMode(bool CountDownEn, cdmodePCF_t CountDownMode)
{
    uint8_t wBuf[2] = {0};
    uint8_t rBuf[2] = {0};

    if (!initHwDone) return PCF2123_INIT_ERROR;

    // read register Timer_clkout. Set CTD[1:0] and TE
    wBuf[0] = PCF2123_READ | PCF2123_REG_TMR_CLKOUT;
    if (sIoFunc.SPI_Read(wBuf, rBuf, 1) != 0) return PCF2123_I2C_ERROR;

    wBuf[0] = PCF2123_WRITE | PCF2123_REG_TMR_CLKOUT;
    wBuf[1] = (CountDownEn == true) ? rBuf[1] | CD_TMR_TE : rBuf[1] &~CD_TMR_TE;    // Set/reset TE bit
    wBuf[1] = (wBuf[1] & ~CTD_MASK) | (CountDownMode);    // Set CTD[1:0]
    if (sIoFunc.SPI_Write(wBuf, 2) != 0) return PCF2123_I2C_ERROR;

    return PCF2123_SUCCESS;
}

// Set Countdown timer value
pcf2123_status_t PCF2123_SetCountdownTimer(uint8_t CDT_Value)
{
    uint8_t wBuf[2] = {PCF2123_WRITE | PCF2123_REG_CTDWN_TMR, CDT_Value};

    if (!initHwDone) return PCF2123_INIT_ERROR;

    if (sIoFunc.SPI_Write(wBuf, 2) != 0) return PCF2123_I2C_ERROR;

    return PCF2123_SUCCESS;
}

// Set Offset register
pcf2123_status_t PCF2123_SetOffset(bool mode, uint8_t Offset)
{
    uint8_t wBuf[2] = {PCF2123_WRITE | PCF2123_REG_OFFSET, (mode << 7) | (Offset & 0x7f)};

    if (!initHwDone) return PCF2123_INIT_ERROR;

    if (sIoFunc.SPI_Write(wBuf, 2) != 0) return PCF2123_I2C_ERROR;

    return PCF2123_SUCCESS;
}

// Set Minute and second interrupt
pcf2123_status_t PCF2123_SetMinSecInterrupt(bool Minute_Int, bool Second_Int, bool Pulse_Int)
{
    uint8_t wBuf[2] = {0};
    uint8_t rBuf[2] = {0};

    if (!initHwDone) return PCF2123_INIT_ERROR;

    // read register CTRL_2 and set MI/SI bits
    wBuf[0] = PCF2123_READ | PCF2123_REG_CTRL2;
    if (sIoFunc.SPI_Read(wBuf, rBuf, 1) != 0) return PCF2123_I2C_ERROR;

    wBuf[0] = PCF2123_WRITE | PCF2123_REG_CTRL2;
    wBuf[1] = (Minute_Int == true) ? wBuf[1] | CTRL2_MI : wBuf[1] & ~CTRL2_MI;        // Set/Reset minute interrupt
    wBuf[1] = (Second_Int == true) ? wBuf[1] | CTRL2_SI : wBuf[1] & ~CTRL2_SI;        // Set/Reset minute interrupt
    wBuf[1] = (Pulse_Int == true) ? wBuf[1] | CTRL2_TI_TP : wBuf[1] & ~CTRL2_TI_TP;    // Set/Reset interrupt pulse mode
    if (sIoFunc.SPI_Write(wBuf, 2) != 0) return PCF2123_I2C_ERROR;

    return PCF2123_SUCCESS;
}

// Set Alarm interrupt
pcf2123_status_t PCF2123_SetAlarmInterrupt(bool Alarm_Int)
{
    uint8_t wBuf[2] = {0};
    uint8_t rBuf[2] = {0};

    if (!initHwDone) return PCF2123_INIT_ERROR;

    // read register CTRL_2 and reset TF bit
    wBuf[0] = PCF2123_READ | PCF2123_REG_CTRL2;
    if (sIoFunc.SPI_Read(wBuf, rBuf, 1) != 0) return PCF2123_I2C_ERROR;

    wBuf[0] = PCF2123_WRITE | PCF2123_REG_CTRL2;
    wBuf[1] = (Alarm_Int == true) ? wBuf[1] | CTRL2_AIE : wBuf[1] & ~CTRL2_AIE;        // Set/Reset alarm interrupt
    if (sIoFunc.SPI_Write(wBuf, 2) != 0) return PCF2123_I2C_ERROR;

    return PCF2123_SUCCESS;
}

// Clear minute or second interrupt flag MSF
pcf2123_status_t PCF2123_ClearMinSecInterruptFlag()
{
    uint8_t wBuf[2] = {0};
    uint8_t rBuf[2] = {0};

    if (!initHwDone) return PCF2123_INIT_ERROR;

    // read register CTRL_2 and reset MSF bit
    wBuf[0] = PCF2123_READ | PCF2123_REG_CTRL2;
    if (sIoFunc.SPI_Read(wBuf, rBuf, 1) != 0) return PCF2123_I2C_ERROR;

    wBuf[0] = PCF2123_WRITE | PCF2123_REG_CTRL2;
    wBuf[1] &= ~CTRL2_MSF;                                                // Reset MSF bit
    if (sIoFunc.SPI_Write(wBuf, 2) != 0) return PCF2123_I2C_ERROR;

    return PCF2123_SUCCESS;
}

// Clear Timer countdown flag TF
pcf2123_status_t PCF2123_ClearCountdownInterruptFlag()
{
    uint8_t wBuf[2] = {0};
    uint8_t rBuf[2] = {0};

    if (!initHwDone) return PCF2123_INIT_ERROR;

    // read register CTRL_2 and reset TF bit
    wBuf[0] = PCF2123_READ | PCF2123_REG_CTRL2;
    if (sIoFunc.SPI_Read(wBuf, rBuf, 1) != 0) return PCF2123_I2C_ERROR;

    wBuf[0] = PCF2123_WRITE | PCF2123_REG_CTRL2;
    wBuf[1] &= ~CTRL2_TF;                                                // Reset TF bit
    if (sIoFunc.SPI_Write(wBuf, 2) != 0) return PCF2123_I2C_ERROR;

    return PCF2123_SUCCESS;
}

// Clear Alarm flag AF
pcf2123_status_t PCF2123_ClearAlarmInterruptFlag()
{
    uint8_t wBuf[2] = {0};
    uint8_t rBuf[2] = {0};

    if (!initHwDone) return PCF2123_INIT_ERROR;

    // read register CTRL_2 and reset TF bit
    wBuf[0] = PCF2123_READ | PCF2123_REG_CTRL2;
    if (sIoFunc.SPI_Read(wBuf, rBuf, 1) != 0) return PCF2123_I2C_ERROR;

    wBuf[0] = PCF2123_WRITE | PCF2123_REG_CTRL2;
    wBuf[1] &= ~CTRL2_AF;                                                // Reset AF bit
    if (sIoFunc.SPI_Write(wBuf, 2) != 0) return PCF2123_I2C_ERROR;

    return PCF2123_SUCCESS;
}

// Set Minute alarm
pcf2123_status_t PCF2123_SetMinuteAlarm(bool AlarmEn, uint8_t Minute)
{
    uint8_t wBuf[2] = {0};

    if (!initHwDone) return PCF2123_INIT_ERROR;

    wBuf[0] = PCF2123_WRITE | PCF2123_REG_ALRM_MN;
    wBuf[1] = (~AlarmEn << ALARM_EN_SHIFT) | (Minute & 0x7F);    // Alarm AE_M: minute alarm enabled when 0
    if (sIoFunc.SPI_Write(wBuf, 2) != 0) return PCF2123_I2C_ERROR;

    return PCF2123_SUCCESS;
}

// Set Hour alarm
pcf2123_status_t PCF2123_SetHourAlarm(bool AlarmEn, bool AMPM, uint8_t Hour)
{
    uint8_t wBuf[2] = {0};
    uint8_t rBuf[2] = {0};

    if (!initHwDone) return PCF2123_INIT_ERROR;

    // read register CTRL_1 to check if 12H or 24H mode is selected
    wBuf[0] = PCF2123_READ | PCF2123_REG_CTRL1;
    if (sIoFunc.SPI_Read(wBuf, rBuf, 1) != 0) return PCF2123_I2C_ERROR;

    wBuf[0] = PCF2123_WRITE | PCF2123_REG_ALRM_HR;
    wBuf[1] = (rBuf[1] & CTRL1_12_HOUR) ? (~AlarmEn << ALARM_EN_SHIFT) | (Hour & 0x7F) : (~AlarmEn << ALARM_EN_SHIFT) | (AMPM << 5) | (Hour & 0x3F);

    if (sIoFunc.SPI_Write(wBuf, 2) != 0) return PCF2123_I2C_ERROR;

    return PCF2123_SUCCESS;
}

// Set Day alarm
pcf2123_status_t PCF2123_SetDayAlarm(bool AlarmEn, uint8_t Day)
{
    uint8_t wBuf[2] = {0};

    if (!initHwDone) return PCF2123_INIT_ERROR;

    wBuf[0] = PCF2123_WRITE | PCF2123_REG_ALRM_DM;
    wBuf[1] = (~AlarmEn << ALARM_EN_SHIFT) | (Day & 0x3F);
    if (sIoFunc.SPI_Write(wBuf, 2) != 0) return PCF2123_I2C_ERROR;

    return PCF2123_SUCCESS;
}

// Set Weekday alarm
pcf2123_status_t PCF2123_SetWeekdayAlarm(bool AlarmEn, uint8_t Weekday)
{
    uint8_t wBuf[2] = {0};

    if (!initHwDone) return PCF2123_INIT_ERROR;

    wBuf[0] = PCF2123_WRITE | PCF2123_REG_ALRM_DW;
    wBuf[1] = (~AlarmEn << ALARM_EN_SHIFT) | (Weekday & 0x03);
    if (sIoFunc.SPI_Write(wBuf, 2) != 0) return PCF2123_I2C_ERROR;

    return PCF2123_SUCCESS;
}
