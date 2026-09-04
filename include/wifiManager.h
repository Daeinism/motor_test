/*|SCARA-LP MK II Wi-Fi Manager| --------------------------------------------
#
# Project: Summer Project 2026
# Program: wifiManager.h
#
# Description:
#   This program contains the Wi-Fi manager interface for the SCARA.
#
# Author: Dain Kim
# Date Created: 2026-09-04
# Last Modified: 2026-09-04
# -----------------------------------------------------------------------------*/
#ifndef WIFI_MANAGER_H_
#define WIFI_MANAGER_H_

#include <stdbool.h>

typedef enum WIFI_MANAGER_STATUS {
    WIFI_MANAGER_UNINITIALIZED,
    WIFI_MANAGER_DISCONNECTED,
    WIFI_MANAGER_CONNECTING,
    WIFI_MANAGER_CONNECTED
} WIFI_MANAGER_STATUS;

bool wifiManagerInit(void);
WIFI_MANAGER_STATUS wifiManagerGetStatus(void);
void wifiManagerPrintStatus(void);

#endif /* WIFI_MANAGER_H_ */
