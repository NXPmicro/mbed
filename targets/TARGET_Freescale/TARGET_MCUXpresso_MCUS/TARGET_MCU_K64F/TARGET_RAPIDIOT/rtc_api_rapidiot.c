/* mbed Microcontroller Library
 * Copyright (c) 2006-2018 ARM Limited
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
#include "rtc_api.h"

#if DEVICE_RTC

#include "pinmap.h"
#include "spi_api.h"
#include "pcf2123.h"
#include "mbed_wait_api.h"
#include "mbed_mktime.h"
#include "PeripheralPins.h"

static bool rtc_time_set = false;
static bool rtc_inited = false;
/* RTC SPI */
spi_t rtc_spi;

static uint8_t rtc_spi_write(uint8_t *buf, uint32_t size)
{
    // write the data
    for (uint32_t i = 0; i < size; i++) {
        spi_master_write(&rtc_spi, buf[i]);
    }

    return 0;
}

static uint8_t rtc_spi_read(uint8_t *writebuf, uint8_t *readbuf, uint32_t size)
{
    // read the data
    for (uint32_t i = 0; i < size; i++) {
        readbuf[i] = spi_master_write(&rtc_spi, 0x0);
    }

    return 0;
}

static void rtc_spi_DelayMilliseconds(uint32_t milliseconds)
{
    wait(milliseconds);
}


void rtc_init(void)
{
    /* Initialize SPI for RTC */
    spi_init(&rtc_spi, PTD6, PTD7, PTD5, PTB9);
    spi_format(&rtc_spi, 8, 0, 0);
    /* Set SPI Freq */
    spi_frequency(&rtc_spi, 1000000);

    pcf2123_IoFunc_t io;
    io.SPI_Write = rtc_spi_write;
    io.SPI_Read = rtc_spi_read;
    io.WaitMsec = rtc_spi_DelayMilliseconds;

    // Initialize RTC
    PCF2123_Init_Driver(&io);

    if (!rtc_inited) {
        settingsPCF_t settings;

        settings.MinInterrupt = false;
        settings.PulseInterrupt = false;
        settings.SecInterrupt = false;
        settings.Softreset = true;
        settings.clockOutputFreq = clkoutFreq_32768;
        settings.days = 1;
        settings.hours = 2;
        settings.minutes = 3;
        settings.mode12_24 = PCF2123_MODE_24HOURS;
        settings.months = April;
        settings.seconds = 14;
        settings.weekdays = Tuesday;
        settings.years = 18;

        if (PCF2123_Init_Hw(&settings) != PCF2123_SUCCESS) {
            return;
        }

        rtc_inited = true;
    }
}

void rtc_free(void)
{

}

/*
 * Little check routine to see if the RTC has been initialized and time has been set
 * 0 = Disabled, 1 = Enabled
 */
int rtc_isenabled(void)
{
    return (rtc_inited & rtc_time_set);
}

time_t rtc_read(void)
{
    settingsPCF_t pcfSettings;
    struct tm timeinfo;

    if (PCF2123_GetDateTime(&pcfSettings) != PCF2123_SUCCESS) {
        return 0;
    }

    timeinfo.tm_mday = (((pcfSettings.days >> 4) & 0x3) * 10) + (pcfSettings.days & 0xF);
    timeinfo.tm_mon  = ((((pcfSettings.months >> 4) & 0x1) * 10) + (pcfSettings.months & 0xF)) - 1;
    timeinfo.tm_year = (((pcfSettings.years >> 4) & 0xF) * 10) + (pcfSettings.years & 0xF);
    timeinfo.tm_hour = (((pcfSettings.hours >> 4) & 0x3) * 10) + (pcfSettings.hours & 0xF);
    timeinfo.tm_min  = (((pcfSettings.minutes >> 4) & 0x7) * 10) + (pcfSettings.minutes & 0xF);
    timeinfo.tm_sec  = (((pcfSettings.seconds >> 4) & 0x7) * 10) + (pcfSettings.seconds & 0xF);

    // Convert to timestamp
    time_t t;
    if (_rtc_maketime(&timeinfo, &t, RTC_4_YEAR_LEAP_YEAR_SUPPORT) == false) {
        return 0;
    }

    return t;
}

void rtc_write(time_t t)
{
    settingsPCF_t pcfSettings;
    struct tm timeinfo;

    // Convert the time into a tm
    if (_rtc_localtime(t, &timeinfo, RTC_4_YEAR_LEAP_YEAR_SUPPORT) == false) {
        return;
    }

    pcfSettings.days = (timeinfo.tm_mday / 10) << 4 | (timeinfo.tm_mday % 10);
    timeinfo.tm_mon += 1;
    pcfSettings.months = (monthsPCF_t)((timeinfo.tm_mon / 10) << 4 | (timeinfo.tm_mon % 10));
    pcfSettings.years = (timeinfo.tm_year / 10) << 4 | (timeinfo.tm_year % 10);
    pcfSettings.hours = (timeinfo.tm_hour / 10) << 4 | (timeinfo.tm_hour % 10);
    pcfSettings.minutes = (timeinfo.tm_min / 10) << 4 | (timeinfo.tm_min % 10);
    pcfSettings.seconds = (timeinfo.tm_sec / 10) << 4 | (timeinfo.tm_sec % 10);
    pcfSettings.weekdays = (weekdaysPCF_t)timeinfo.tm_wday;

    PCF2123_SetDateTime(&pcfSettings);

    rtc_time_set = true;
}

#endif
