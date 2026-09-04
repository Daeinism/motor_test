/*|SCARA-LP MK II Wi-Fi Manager| --------------------------------------------
#
# Project: Summer Project 2026
# Program: wifiManager.c
#
# Description:
#   This program initializes and tracks the Wi-Fi system for the SCARA.
#
# Author: Dain Kim
# Date Created: 2026-09-04
# Last Modified: 2026-09-04
# -----------------------------------------------------------------------------*/

/*|Includes|-------------------------------------------------------------------*/
#include "wifiManager.h"

#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

/*|CONSTANTS|------------------------------------------------------------------*/
#define WIFI_MANAGER_IP_ADDRESS_LENGTH 16
#define WIFI_MANAGER_MAX_DETECTED_NETWORKS 20

/*|Function Prototype|---------------------------------------------------------*/
static void wifiEventHandler(void *arg, esp_event_base_t eventBase, int32_t eventId, void *eventData);
static bool checkEspResult(esp_err_t result, const char *operation);
static const char *wifiAuthModeToString(wifi_auth_mode_t authMode);

/*|Variable Declaration|-------------------------------------------------------*/
static volatile WIFI_MANAGER_STATUS wifiStatus = WIFI_MANAGER_UNINITIALIZED;
static char wifiIpAddress[WIFI_MANAGER_IP_ADDRESS_LENGTH] = "0.0.0.0";
static bool wifiInitialized = false;
static wifi_ap_record_t detectedNetworks[WIFI_MANAGER_MAX_DETECTED_NETWORKS];
static uint16_t detectedNetworkCount = 0;

/*|Function Definitions|-------------------------------------------------------*/
bool wifiManagerInit(void)
{
    if (wifiInitialized) {
        return true;
    }

    // NVS (Non-Volatile Storage) stores data required internally by the ESP-IDF Wi-Fi driver.
    if (!checkEspResult(nvs_flash_init(), "NVS initialization")) {
        return false;
    }

    // Initialize the TCP/IP stack, default event loop, and Station network interface.
    if (!checkEspResult(esp_netif_init(), "network interface initialization") ||
        !checkEspResult(esp_event_loop_create_default(), "default event loop creation")) {
        return false;
    }

    if (esp_netif_create_default_wifi_sta() == NULL) {
        printf("Wi-Fi initialization failed: Station interface creation failed\n");
        return false;
    }

    wifi_init_config_t wifiConfiguration = WIFI_INIT_CONFIG_DEFAULT();
    if (!checkEspResult(esp_wifi_init(&wifiConfiguration), "Wi-Fi driver initialization") ||
        !checkEspResult(esp_event_handler_register(WIFI_EVENT,
                                                   ESP_EVENT_ANY_ID,
                                                   &wifiEventHandler,
                                                   NULL),
                        "Wi-Fi event handler registration") ||
        !checkEspResult(esp_event_handler_register(IP_EVENT,
                                                   IP_EVENT_STA_GOT_IP,
                                                   &wifiEventHandler,
                                                   NULL),
                        "IP event handler registration") ||
        !checkEspResult(esp_wifi_set_mode(WIFI_MODE_STA), "Wi-Fi Station mode selection") ||
        !checkEspResult(esp_wifi_start(), "Wi-Fi driver start")) {
        return false;
    }

    // Starting the driver prepares Wi-Fi, but it does not connect to an access point.
    wifiStatus = WIFI_MANAGER_DISCONNECTED;
    wifiInitialized = true;
    printf("Wi-Fi manager initialized. Status: disconnected\n");
    return true;
}

bool wifiManagerDetectNetworks(void)
{
    if (!wifiInitialized) {
        printf("Wi-Fi scan failed: Wi-Fi manager is not initialized\n");
        return false;
    }

    if (wifiStatus != WIFI_MANAGER_DISCONNECTED) {
        printf("Wi-Fi scan rejected: disconnect from the current network first\n");
        return false;
    }

    detectedNetworkCount = 0;
    printf("Scanning for Wi-Fi networks...\n");

    // A blocking scan keeps this command in sequence with the other queued commands.
    esp_err_t scanResult = esp_wifi_scan_start(NULL, true);
    if (scanResult != ESP_OK) {
        printf("Wi-Fi scan failed: %s\n", esp_err_to_name(scanResult));
        return false;
    }

    uint16_t totalNetworkCount = 0;
    esp_err_t countResult = esp_wifi_scan_get_ap_num(&totalNetworkCount);
    if (countResult != ESP_OK) {
        printf("Wi-Fi scan result count failed: %s\n", esp_err_to_name(countResult));
        esp_wifi_clear_ap_list();
        return false;
    }

    if (totalNetworkCount == 0) {
        printf("Wi-Fi scan complete: no networks found\n");
        esp_wifi_clear_ap_list();
        return true;
    }

    detectedNetworkCount = totalNetworkCount;
    if (detectedNetworkCount > WIFI_MANAGER_MAX_DETECTED_NETWORKS) {
        detectedNetworkCount = WIFI_MANAGER_MAX_DETECTED_NETWORKS;
    }

    esp_err_t recordsResult = esp_wifi_scan_get_ap_records(&detectedNetworkCount, detectedNetworks);
    if (recordsResult != ESP_OK) {
        detectedNetworkCount = 0;
        printf("Wi-Fi scan result retrieval failed: %s\n", esp_err_to_name(recordsResult));
        esp_wifi_clear_ap_list();
        return false;
    }

    printf("Wi-Fi scan complete: %u network(s) found",
           (unsigned int)totalNetworkCount);
    if (totalNetworkCount > detectedNetworkCount) {
        printf(" (showing strongest %u)", (unsigned int)detectedNetworkCount);
    }
    printf("\n");

    // Keep these records so wifiConnect can select one by number in the next stage.
    for (uint16_t i = 0; i < detectedNetworkCount; i++) {
        const char *ssid = detectedNetworks[i].ssid[0] == '\0'
                               ? "<hidden>"
                               : (const char *)detectedNetworks[i].ssid;

        printf("[%u] SSID: %s | RSSI: %d dBm | Channel: %u | Security: %s\n",
               (unsigned int)i,
               ssid,
               detectedNetworks[i].rssi,
               (unsigned int)detectedNetworks[i].primary,
               wifiAuthModeToString(detectedNetworks[i].authmode));
    }

    return true;
}

WIFI_MANAGER_STATUS wifiManagerGetStatus(void)
{
    return wifiStatus;
}

void wifiManagerPrintStatus(void)
{
    switch (wifiManagerGetStatus()) {
        case WIFI_MANAGER_UNINITIALIZED:
            printf("Wi-Fi status: uninitialized\n");
            break;

        case WIFI_MANAGER_DISCONNECTED:
            printf("Wi-Fi status: disconnected\n");
            break;

        case WIFI_MANAGER_CONNECTING:
            printf("Wi-Fi status: connecting\n");
            break;

        case WIFI_MANAGER_CONNECTED:
            printf("Wi-Fi status: connected\n");
            printf("IP address: %s\n", wifiIpAddress);
            break;
    }
}

static void wifiEventHandler(void *arg, esp_event_base_t eventBase, int32_t eventId, void *eventData)
{
    (void)arg;

    if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_DISCONNECTED) {
        wifiStatus = WIFI_MANAGER_DISCONNECTED;
        snprintf(wifiIpAddress, sizeof(wifiIpAddress), "0.0.0.0");
        return;
    }

    if (eventBase == IP_EVENT && eventId == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *gotIpEvent = (ip_event_got_ip_t *)eventData;
        snprintf(wifiIpAddress,
                 sizeof(wifiIpAddress),
                 IPSTR,
                 IP2STR(&gotIpEvent->ip_info.ip));
        wifiStatus = WIFI_MANAGER_CONNECTED;
    }
}

static bool checkEspResult(esp_err_t result, const char *operation)
{
    if (result == ESP_OK) {
        return true;
    }

    printf("Wi-Fi initialization failed during %s: %s\n",
           operation,
           esp_err_to_name(result));
    return false;
}

static const char *wifiAuthModeToString(wifi_auth_mode_t authMode)
{
    switch (authMode) {
        case WIFI_AUTH_OPEN:
            return "OPEN";
        case WIFI_AUTH_WEP:
            return "WEP";
        case WIFI_AUTH_WPA_PSK:
            return "WPA";
        case WIFI_AUTH_WPA2_PSK:
            return "WPA2";
        case WIFI_AUTH_WPA_WPA2_PSK:
            return "WPA/WPA2";
        case WIFI_AUTH_WPA3_PSK:
        case WIFI_AUTH_WPA3_EXT_PSK:
            return "WPA3";
        case WIFI_AUTH_WPA2_WPA3_PSK:
        case WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE:
            return "WPA2/WPA3";
        case WIFI_AUTH_OWE:
            return "OWE";
        case WIFI_AUTH_WAPI_PSK:
            return "WAPI";
        case WIFI_AUTH_ENTERPRISE:
        case WIFI_AUTH_WPA3_ENT_192:
        case WIFI_AUTH_WPA3_ENTERPRISE:
        case WIFI_AUTH_WPA2_WPA3_ENTERPRISE:
        case WIFI_AUTH_WPA_ENTERPRISE:
            return "ENTERPRISE";
        case WIFI_AUTH_DPP:
            return "DPP";
        default:
            return "UNKNOWN";
    }
}
