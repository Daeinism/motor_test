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
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"

/*|CONSTANTS|------------------------------------------------------------------*/
#define WIFI_MANAGER_IP_ADDRESS_LENGTH 16
#define WIFI_MANAGER_MAX_DETECTED_NETWORKS 20
#define WIFI_MANAGER_SSID_LENGTH 33
#define WIFI_MANAGER_CONNECT_TIMEOUT_MS 10000
#define WIFI_MANAGER_CONNECTED_BIT BIT0
#define WIFI_MANAGER_CONNECT_FAILED_BIT BIT1

/*|Function Prototype|---------------------------------------------------------*/
static void wifiEventHandler(void *arg, esp_event_base_t eventBase, int32_t eventId, void *eventData);
static bool checkEspResult(esp_err_t result, const char *operation);
static const char *wifiAuthModeToString(wifi_auth_mode_t authMode);
static bool wifiAuthModeUsesPersonalPassword(wifi_auth_mode_t authMode);

/*|Variable Declaration|-------------------------------------------------------*/
static volatile WIFI_MANAGER_STATUS wifiStatus = WIFI_MANAGER_UNINITIALIZED;
static char wifiIpAddress[WIFI_MANAGER_IP_ADDRESS_LENGTH] = "0.0.0.0";
static bool wifiInitialized = false;
static wifi_ap_record_t detectedNetworks[WIFI_MANAGER_MAX_DETECTED_NETWORKS];
static uint16_t detectedNetworkCount = 0;
static EventGroupHandle_t wifiEventGroup = NULL;
static bool connectionRequested = false;
static char connectedSsid[WIFI_MANAGER_SSID_LENGTH] = "";

/*|Function Definitions|-------------------------------------------------------*/
bool wifiManagerInit(void)
{
    if (wifiInitialized) {
        return true;
    }

    wifiEventGroup = xEventGroupCreate();
    if (wifiEventGroup == NULL) {
        printf("Wi-Fi initialization failed: event group creation failed\n");
        return false;
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
        !checkEspResult(esp_wifi_set_storage(WIFI_STORAGE_RAM), "Wi-Fi configuration storage selection") ||
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

bool wifiManagerConnect(int networkIndex, const char *password)
{
    if (!wifiInitialized) {
        printf("Wi-Fi connection failed: Wi-Fi manager is not initialized\n");
        return false;
    }

    if (wifiStatus != WIFI_MANAGER_DISCONNECTED || connectionRequested) {
        printf("Wi-Fi connection rejected: Wi-Fi is already connecting or connected\n");
        return false;
    }

    if (networkIndex < 0 || networkIndex >= detectedNetworkCount) {
        printf("Wi-Fi connection failed: network number is not in the latest scan results\n");
        return false;
    }

    if (password == NULL || detectedNetworks[networkIndex].ssid[0] == '\0') {
        printf("Wi-Fi connection failed: the selected network cannot be configured\n");
        return false;
    }

    wifi_auth_mode_t authMode = detectedNetworks[networkIndex].authmode;
    const char *configuredPassword = password;
    size_t passwordLength = strlen(password);

    if (authMode == WIFI_AUTH_OPEN) {
        if (strcmp(password, "-") != 0) {
            printf("Wi-Fi connection failed: use - as the password for an open network\n");
            return false;
        }
        configuredPassword = "";
    } else if (!wifiAuthModeUsesPersonalPassword(authMode)) {
        printf("Wi-Fi connection failed: the selected security mode is not supported\n");
        return false;
    } else if (passwordLength < 8 || passwordLength > 63) {
        printf("Wi-Fi connection failed: password must contain 8 to 63 characters\n");
        return false;
    }

    wifi_config_t wifiConfiguration = {0};
    memcpy(wifiConfiguration.sta.ssid,
           detectedNetworks[networkIndex].ssid,
           sizeof(wifiConfiguration.sta.ssid));
    memcpy(wifiConfiguration.sta.bssid,
           detectedNetworks[networkIndex].bssid,
           sizeof(wifiConfiguration.sta.bssid));
    wifiConfiguration.sta.bssid_set = true;
    wifiConfiguration.sta.channel = detectedNetworks[networkIndex].primary;
    wifiConfiguration.sta.threshold.authmode = authMode;
    snprintf((char *)wifiConfiguration.sta.password,
             sizeof(wifiConfiguration.sta.password),
             "%s",
             configuredPassword);

    xEventGroupClearBits(wifiEventGroup,
                         WIFI_MANAGER_CONNECTED_BIT | WIFI_MANAGER_CONNECT_FAILED_BIT);

    connectionRequested = true;
    wifiStatus = WIFI_MANAGER_CONNECTING;
    snprintf(connectedSsid,
             sizeof(connectedSsid),
             "%s",
             (const char *)detectedNetworks[networkIndex].ssid);

    printf("Connecting to %s...\n", connectedSsid);

    esp_err_t configurationResult = esp_wifi_set_config(WIFI_IF_STA, &wifiConfiguration);
    if (configurationResult != ESP_OK) {
        printf("Wi-Fi connection failed while setting configuration: %s\n",
               esp_err_to_name(configurationResult));
        connectionRequested = false;
        wifiStatus = WIFI_MANAGER_DISCONNECTED;
        connectedSsid[0] = '\0';
        return false;
    }

    esp_err_t connectionResult = esp_wifi_connect();
    if (connectionResult != ESP_OK) {
        printf("Wi-Fi connection request failed: %s\n", esp_err_to_name(connectionResult));
        connectionRequested = false;
        wifiStatus = WIFI_MANAGER_DISCONNECTED;
        connectedSsid[0] = '\0';
        return false;
    }

    EventBits_t connectionBits = xEventGroupWaitBits(
        wifiEventGroup,
        WIFI_MANAGER_CONNECTED_BIT | WIFI_MANAGER_CONNECT_FAILED_BIT,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(WIFI_MANAGER_CONNECT_TIMEOUT_MS)
    );

    if ((connectionBits & WIFI_MANAGER_CONNECTED_BIT) != 0) {
        printf("Wi-Fi connected\n");
        printf("SSID: %s\n", connectedSsid);
        printf("IP address: %s\n", wifiIpAddress);
        return true;
    }

    if ((connectionBits & WIFI_MANAGER_CONNECT_FAILED_BIT) != 0) {
        printf("Wi-Fi connection failed\n");
    } else {
        printf("Wi-Fi connection failed: IP address acquisition timed out\n");
    }

    connectionRequested = false;
    wifiStatus = WIFI_MANAGER_DISCONNECTED;
    connectedSsid[0] = '\0';
    snprintf(wifiIpAddress, sizeof(wifiIpAddress), "0.0.0.0");
    esp_wifi_disconnect();
    return false;
}

bool wifiManagerDisconnect(void)
{
    if (!wifiInitialized) {
        printf("Wi-Fi disconnect failed: Wi-Fi manager is not initialized\n");
        return false;
    }

    if (wifiStatus == WIFI_MANAGER_DISCONNECTED && !connectionRequested) {
        printf("Wi-Fi is already disconnected\n");
        return true;
    }

    // Clear this first so a deliberate disconnect cannot trigger automatic reconnection later.
    connectionRequested = false;
    esp_err_t disconnectResult = esp_wifi_disconnect();
    if (disconnectResult != ESP_OK && disconnectResult != ESP_ERR_WIFI_NOT_CONNECT) {
        printf("Wi-Fi disconnect failed: %s\n", esp_err_to_name(disconnectResult));
        return false;
    }

    wifiStatus = WIFI_MANAGER_DISCONNECTED;
    connectedSsid[0] = '\0';
    snprintf(wifiIpAddress, sizeof(wifiIpAddress), "0.0.0.0");
    xEventGroupClearBits(wifiEventGroup,
                         WIFI_MANAGER_CONNECTED_BIT | WIFI_MANAGER_CONNECT_FAILED_BIT);
    printf("Wi-Fi disconnected\n");
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
            printf("SSID: %s\n", connectedSsid);
            printf("IP address: %s\n", wifiIpAddress);
            break;
    }
}

static void wifiEventHandler(void *arg, esp_event_base_t eventBase, int32_t eventId, void *eventData)
{
    (void)arg;

    if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_DISCONNECTED) {
        bool connectionFailed = connectionRequested && wifiStatus == WIFI_MANAGER_CONNECTING;
        wifiStatus = WIFI_MANAGER_DISCONNECTED;
        snprintf(wifiIpAddress, sizeof(wifiIpAddress), "0.0.0.0");

        if (connectionFailed && wifiEventGroup != NULL) {
            xEventGroupSetBits(wifiEventGroup, WIFI_MANAGER_CONNECT_FAILED_BIT);
        }
        return;
    }

    if (eventBase == IP_EVENT && eventId == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *gotIpEvent = (ip_event_got_ip_t *)eventData;
        snprintf(wifiIpAddress,
                 sizeof(wifiIpAddress),
                 IPSTR,
                 IP2STR(&gotIpEvent->ip_info.ip));
        wifiStatus = WIFI_MANAGER_CONNECTED;

        if (wifiEventGroup != NULL) {
            xEventGroupSetBits(wifiEventGroup, WIFI_MANAGER_CONNECTED_BIT);
        }
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

static bool wifiAuthModeUsesPersonalPassword(wifi_auth_mode_t authMode)
{
    switch (authMode) {
        case WIFI_AUTH_WPA_PSK:
        case WIFI_AUTH_WPA2_PSK:
        case WIFI_AUTH_WPA_WPA2_PSK:
        case WIFI_AUTH_WPA3_PSK:
        case WIFI_AUTH_WPA2_WPA3_PSK:
        case WIFI_AUTH_WPA3_EXT_PSK:
        case WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE:
            return true;

        default:
            return false;
    }
}
