#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_iocon.h"
#include "wifi_shield_silex2401.h"

#if MBED_CONF_QCA400X_CONNECTOR_TYPE == ARDUINO

#define IOCON_PIO_DIGITAL_EN 0x0100u  /*!<@brief Enables digital function */
#define IOCON_PIO_ASW_DI 0x0400u      /*!<@brief Analog switch is disabled */
#define IOCON_PIO_FUNC0 0x00u         /*!<@brief Selects pin function 0 */
#define IOCON_PIO_FUNC1 0x01u         /*!<@brief Selects pin function 1 */
#define IOCON_PIO_FUNC5 0x05u         /*!<@brief Selects pin function 5 */
#define IOCON_PIO_FUNC6 0x06u         /*!<@brief Selects pin function 6 */
#define IOCON_PIO_FUNC9 0x09u         /*!<@brief Selects pin function 9 */
#define IOCON_PIO_INV_DI 0x00u        /*!<@brief Input function is not inverted */
#define IOCON_PIO_MODE_INACT 0x00u    /*!<@brief No addition pin function */
#define IOCON_PIO_MODE_PULLDOWN 0x10u /*!<@brief Selects pull-down function */
#define IOCON_PIO_MODE_PULLUP 0x20u   /*!<@brief Selects pull-up function */
#define IOCON_PIO_OPENDRAIN_DI 0x00u  /*!<@brief Open drain is disabled */
#define IOCON_PIO_SLEW_FAST 0x40u     /*!<@brief Fast mode, slew rate control is disabled */
#define IOCON_PIO_SLEW_STANDARD 0x00u /*!<@brief Standard mode, output slew rate control is enabled */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitSilex2401Shield
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M33 (Core #0) */
void BOARD_InitSilex2401Shield(void)
{
    /* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

    /* Enables the clock for the GPIO0 module */
    CLOCK_EnableClock(kCLOCK_Gpio0);

    /* Enables the clock for the GPIO1 module */
    CLOCK_EnableClock(kCLOCK_Gpio1);

    gpio_pin_config_t gpio0_pin22_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO0_15 (pin 22)  */
    GPIO_PinInit(GPIO, 0U, 15U, &gpio0_pin22_config);

    gpio_pin_config_t gpio1_pin1_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_4 (pin 1)  */
    GPIO_PinInit(GPIO, 1U, 4U, &gpio1_pin1_config);

    gpio_pin_config_t gpio1_pin9_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_7 (pin 9)  */
    GPIO_PinInit(GPIO, 1U, 7U, &gpio1_pin9_config);

    const uint32_t port0_pin15_config = (/* Pin is configured as PIO0_15 */
                                         IOCON_PIO_FUNC0 |
                                         /* Selects pull-up function */
                                         IOCON_PIO_MODE_PULLUP |
                                         /* Standard mode, output slew rate control is enabled */
                                         IOCON_PIO_SLEW_STANDARD |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI |
                                         /* Analog switch is disabled */
                                         IOCON_PIO_ASW_DI);
    /* PORT0 PIN15 (coords: 22) is configured as PIO0_15 */
    IOCON_PinMuxSet(IOCON, 0U, 15U, port0_pin15_config);

    const uint32_t port0_pin26_config = (/* Pin is configured as HS_SPI_MOSI */
                                         IOCON_PIO_FUNC9 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Fast mode, slew rate control is disabled */
                                         IOCON_PIO_SLEW_FAST |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN26 (coords: 60) is configured as HS_SPI_MOSI */
    IOCON_PinMuxSet(IOCON, 0U, 26U, port0_pin26_config);

    const uint32_t port1_pin1_config = (/* Pin is configured as HS_SPI_SSEL1 */
                                        IOCON_PIO_FUNC5 |
                                        /* No addition pin function */
                                        IOCON_PIO_MODE_INACT |
                                        /* Fast mode, slew rate control is disabled */
                                        IOCON_PIO_SLEW_FAST |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI);
    /* PORT1 PIN1 (coords: 59) is configured as HS_SPI_SSEL1 */
    IOCON_PinMuxSet(IOCON, 1U, 1U, port1_pin1_config);

    const uint32_t port1_pin2_config = (/* Pin is configured as HS_SPI_SCK */
                                        IOCON_PIO_FUNC6 |
                                        /* No addition pin function */
                                        IOCON_PIO_MODE_INACT |
                                        /* Fast mode, slew rate control is disabled */
                                        IOCON_PIO_SLEW_FAST |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI);
    /* PORT1 PIN2 (coords: 61) is configured as HS_SPI_SCK */
    IOCON_PinMuxSet(IOCON, 1U, 2U, port1_pin2_config);

    const uint32_t port1_pin3_config = (/* Pin is configured as HS_SPI_MISO */
                                        IOCON_PIO_FUNC6 |
                                        /* No addition pin function */
                                        IOCON_PIO_MODE_INACT |
                                        /* Fast mode, slew rate control is disabled */
                                        IOCON_PIO_SLEW_FAST |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI);
    /* PORT1 PIN3 (coords: 62) is configured as HS_SPI_MISO */
    IOCON_PinMuxSet(IOCON, 1U, 3U, port1_pin3_config);

    const uint32_t port1_pin4_config = (/* Pin is configured as PIO1_4 */
                                        IOCON_PIO_FUNC0 |
                                        /* Selects pull-down function */
                                        IOCON_PIO_MODE_PULLDOWN |
                                        /* Standard mode, output slew rate control is enabled */
                                        IOCON_PIO_SLEW_STANDARD |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI);
    /* PORT1 PIN4 (coords: 1) is configured as PIO1_4 */
    IOCON_PinMuxSet(IOCON, 1U, 4U, port1_pin4_config);

    const uint32_t port1_pin7_config = (/* Pin is configured as PIO1_7 */
                                        IOCON_PIO_FUNC0 |
                                        /* Selects pull-down function */
                                        IOCON_PIO_MODE_PULLDOWN |
                                        /* Standard mode, output slew rate control is enabled */
                                        IOCON_PIO_SLEW_STANDARD |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI);
    /* PORT1 PIN7 (coords: 9) is configured as PIO1_7 */
    IOCON_PinMuxSet(IOCON, 1U, 7U, port1_pin7_config);
}

#else // Configuration for mikroe wifi board
/*!
 * @brief Enables digital function */
#define IOCON_PIO_DIGITAL_EN 0x0100u
/*!
 * @brief Selects pin function 5 */
#define IOCON_PIO_FUNC5 0x05u
/*!
 * @brief Selects pin function 6 */
#define IOCON_PIO_FUNC6 0x06u
/*!
 * @brief Selects pin function 9 */
#define IOCON_PIO_FUNC9 0x09u
/*!
 * @brief Input function is not inverted */
#define IOCON_PIO_INV_DI 0x00u
/*!
 * @brief No addition pin function */
#define IOCON_PIO_MODE_INACT 0x00u
/*!
 * @brief Open drain is disabled */
#define IOCON_PIO_OPENDRAIN_DI 0x00u
/*!
 * @brief Fast mode, slew rate control is disabled */
#define IOCON_PIO_SLEW_FAST 0x40u
/*!
 * @brief Select Digital mode.: Digital mode, digital input is enabled. */
#define PIO0_16_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO0_16_FUNC_ALT0 0x00u
/*!
 * @brief
 * Selects function mode (on-chip pull-up/pull-down resistor control).
 * : Pull-down.
 * Pull-down resistor enabled.
 */
#define PIO0_16_MODE_PULL_DOWN 0x01u
/*!
 * @brief Select Digital mode.: Digital mode, digital input is enabled. */
#define PIO1_18_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO1_18_FUNC_ALT0 0x00u
/*!
 * @brief Selects function mode (on-chip pull-up/pull-down resistor control).: Pull-up. Pull-up resistor enabled. */
#define PIO1_18_MODE_PULL_UP 0x02u
/*!
 * @brief Select Digital mode.: Digital mode, digital input is enabled. */
#define PIO1_5_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO1_5_FUNC_ALT0 0x00u
/*!
 * @brief
 * Selects function mode (on-chip pull-up/pull-down resistor control).
 * : Pull-down.
 * Pull-down resistor enabled.
 */
#define PIO1_5_MODE_PULL_DOWN 0x01u

/*! @name PIO0_16 (number 14), KFETON
  @{ */
#define BOARD_INITSILEX2401SHIELD_KFETON_ID_GPIO GPIO /*!<@brief GPIO device name: GPIO */
#define BOARD_INITSILEX2401SHIELD_KFETON_ID_PORT 0U   /*!<@brief PORT device name: 0U */
#define BOARD_INITSILEX2401SHIELD_KFETON_ID_PIN 16U   /*!<@brief 0U pin index: 16 */
                                                      /* @} */

/*! @name PIO1_5 (number 31), PWRON
  @{ */
#define BOARD_INITSILEX2401SHIELD_PWRON_ID_GPIO GPIO /*!<@brief GPIO device name: GPIO */
#define BOARD_INITSILEX2401SHIELD_PWRON_ID_PORT 1U   /*!<@brief PORT device name: 1U */
#define BOARD_INITSILEX2401SHIELD_PWRON_ID_PIN 5U    /*!<@brief 1U pin index: 5 */
                                                     /* @} */

/*! @name PIO1_18 (number 64), IRQ
  @{ */
#define BOARD_INITSILEX2401SHIELD_IRQ_ID_GPIO GPIO /*!<@brief GPIO device name: GPIO */
#define BOARD_INITSILEX2401SHIELD_IRQ_ID_PORT 1U   /*!<@brief PORT device name: 1U */
#define BOARD_INITSILEX2401SHIELD_IRQ_ID_PIN 18U   /*!<@brief 1U pin index: 18 */

void BOARD_InitSilex2401Shield(void)
{
    /* Enables the clock for the I/O controller.: Enable Clock. */
    CLOCK_EnableClock(kCLOCK_Iocon);

    /* Enables the clock for the GPIO0 module */
    CLOCK_EnableClock(kCLOCK_Gpio0);

    /* Enables the clock for the GPIO1 module */
    CLOCK_EnableClock(kCLOCK_Gpio1);

    gpio_pin_config_t KFETON_ID_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO0_16 (pin 14)  */
    GPIO_PinInit(BOARD_INITSILEX2401SHIELD_KFETON_ID_GPIO, BOARD_INITSILEX2401SHIELD_KFETON_ID_PORT, BOARD_INITSILEX2401SHIELD_KFETON_ID_PIN, &KFETON_ID_config);

    gpio_pin_config_t PWRON_ID_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_5 (pin 31)  */
    GPIO_PinInit(BOARD_INITSILEX2401SHIELD_PWRON_ID_GPIO, BOARD_INITSILEX2401SHIELD_PWRON_ID_PORT, BOARD_INITSILEX2401SHIELD_PWRON_ID_PIN, &PWRON_ID_config);

    gpio_pin_config_t IRQ_ID_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_18 (pin 64)  */
    GPIO_PinInit(BOARD_INITSILEX2401SHIELD_IRQ_ID_GPIO, BOARD_INITSILEX2401SHIELD_IRQ_ID_PORT, BOARD_INITSILEX2401SHIELD_IRQ_ID_PIN, &IRQ_ID_config);

    IOCON->PIO[0][16] = ((IOCON->PIO[0][16] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_MODE_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT016 (pin 14) is configured as PIO0_16. */
                         | IOCON_PIO_FUNC(PIO0_16_FUNC_ALT0)

                         /* Selects function mode (on-chip pull-up/pull-down resistor control).
                          * : Pull-down.
                          * Pull-down resistor enabled. */
                         | IOCON_PIO_MODE(PIO0_16_MODE_PULL_DOWN)

                         /* Select Digital mode.
                          * : Digital mode, digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO0_16_DIGIMODE_DIGITAL));

    const uint32_t port0_pin26_config = (/* Pin is configured as HS_SPI_MOSI */
                                         IOCON_PIO_FUNC9 |
                                         /* No addition pin function */
                                         IOCON_PIO_MODE_INACT |
                                         /* Fast mode, slew rate control is disabled */
                                         IOCON_PIO_SLEW_FAST |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI);
    /* PORT0 PIN26 (coords: 60) is configured as HS_SPI_MOSI */
    IOCON_PinMuxSet(IOCON, 0U, 26U, port0_pin26_config);

    const uint32_t port1_pin1_config = (/* Pin is configured as HS_SPI_SSEL1 */
                                        IOCON_PIO_FUNC5 |
                                        /* No addition pin function */
                                        IOCON_PIO_MODE_INACT |
                                        /* Fast mode, slew rate control is disabled */
                                        IOCON_PIO_SLEW_FAST |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI);
    /* PORT1 PIN1 (coords: 59) is configured as HS_SPI_SSEL1 */
    IOCON_PinMuxSet(IOCON, 1U, 1U, port1_pin1_config);

    IOCON->PIO[1][18] = ((IOCON->PIO[1][18] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_MODE_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT118 (pin 64) is configured as PIO1_18. */
                         | IOCON_PIO_FUNC(PIO1_18_FUNC_ALT0)

                         /* Selects function mode (on-chip pull-up/pull-down resistor control).
                          * : Pull-up.
                          * Pull-up resistor enabled. */
                         | IOCON_PIO_MODE(PIO1_18_MODE_PULL_UP)

                         /* Select Digital mode.
                          * : Digital mode, digital input is enabled. */
                         | IOCON_PIO_DIGIMODE(PIO1_18_DIGIMODE_DIGITAL));

    const uint32_t port1_pin2_config = (/* Pin is configured as HS_SPI_SCK */
                                        IOCON_PIO_FUNC6 |
                                        /* No addition pin function */
                                        IOCON_PIO_MODE_INACT |
                                        /* Fast mode, slew rate control is disabled */
                                        IOCON_PIO_SLEW_FAST |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI);
    /* PORT1 PIN2 (coords: 61) is configured as HS_SPI_SCK */
    IOCON_PinMuxSet(IOCON, 1U, 2U, port1_pin2_config);

    const uint32_t port1_pin3_config = (/* Pin is configured as HS_SPI_MISO */
                                        IOCON_PIO_FUNC6 |
                                        /* No addition pin function */
                                        IOCON_PIO_MODE_INACT |
                                        /* Fast mode, slew rate control is disabled */
                                        IOCON_PIO_SLEW_FAST |
                                        /* Input function is not inverted */
                                        IOCON_PIO_INV_DI |
                                        /* Enables digital function */
                                        IOCON_PIO_DIGITAL_EN |
                                        /* Open drain is disabled */
                                        IOCON_PIO_OPENDRAIN_DI);
    /* PORT1 PIN3 (coords: 62) is configured as HS_SPI_MISO */
    IOCON_PinMuxSet(IOCON, 1U, 3U, port1_pin3_config);

    IOCON->PIO[1][5] = ((IOCON->PIO[1][5] &
                         /* Mask bits to zero which are setting */
                         (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_MODE_MASK | IOCON_PIO_DIGIMODE_MASK)))

                        /* Selects pin function.
                         * : PORT15 (pin 31) is configured as PIO1_5. */
                        | IOCON_PIO_FUNC(PIO1_5_FUNC_ALT0)

                        /* Selects function mode (on-chip pull-up/pull-down resistor control).
                         * : Pull-down.
                         * Pull-down resistor enabled. */
                        | IOCON_PIO_MODE(PIO1_5_MODE_PULL_DOWN)

                        /* Select Digital mode.
                         * : Digital mode, digital input is enabled. */
                        | IOCON_PIO_DIGIMODE(PIO1_5_DIGIMODE_DIGITAL));
}

#endif  // MBED_CONF_QCA400X_CONNECTOR_TYPE

