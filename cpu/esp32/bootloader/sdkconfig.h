/*
 * Copyright (C) 2021 iosabi
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     pkg_esp32_sdk
 * @{
 *
 * @file
 * @brief       RIOT-OS modification of the bootloader SDK configuration
 *
 * The bootloader build of the ESP32 SDK needs some settings from the SDK
 * configuration. These are normally generated by the menuconfig in the vendor
 * SDK.
 *
 * Some of these parameters are configurable by the application. For example,
 * the UART baudrate used by the console and the verbose level of the
 * bootloader.
 *
 * @author      iosabi <iosabi@protonmail.com>
 */

#ifndef SDKCONFIG_H
#define SDKCONFIG_H

#ifndef DOXYGEN

#include "esp_idf_ver.h"
#include "sdkconfig_default.h"

#ifdef __cplusplus
extern "C" {
#endif

#if MODULE_ESP_LOG_COLORED
#define CONFIG_LOG_COLORS   1
#endif

#ifndef CONFIG_BOOTLOADER_LOG_LEVEL
/*
 * SDK Log levels:
 *
 *  0 = NONE
 *  1 = ERROR
 *  2 = WARN
 *  3 = INFO
 *  4 = DEBUG
 *  5 = VERBOSE
 */
#if MODULE_ESP_LOG_STARTUP
#define CONFIG_BOOTLOADER_LOG_LEVEL     3   /* INFO */
#else
#define CONFIG_BOOTLOADER_LOG_LEVEL     0   /* NONE */
#endif
#endif

#if FLASH_MODE_QIO
#define CONFIG_FLASHMODE_QIO            1
#define CONFIG_ESPTOOLPY_FLASHMODE_QIO  1
#elif FLASH_MODE_QOUT
#define CONFIG_FLASHMODE_QOUT           1
#define CONFIG_ESPTOOLPY_FLASHMODE_QOUT 1
#elif FLASH_MODE_DIO
#define CONFIG_FLASHMODE_DIO            1
#define CONFIG_ESPTOOLPY_FLASHMODE_DIO  1
#elif FLASH_MODE_DOUT
#define CONFIG_FLASHMODE_DOUT           1
#define CONFIG_ESPTOOLPY_FLASHMODE_DOUT 1
#else
#error "Unknown flash mode selected."
#endif

/*
 * Bootloader output baudrate, defined by the app settings as BAUD or
 * BOOTLOADER_BAUD.
 */
#define CONFIG_ESP_CONSOLE_UART_BAUDRATE (RIOT_BOOTLOADER_BAUD)

#ifdef __cplusplus
}
#endif

#endif /* DOXYGEN */
/** @} */

#endif /* SDKCONFIG_H */