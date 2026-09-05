/*|SCARA-LP MK II TCP Command Server| ----------------------------------------
#
# Project: Summer Project 2026
# Program: tcpCommandServer.c
#
# Description:
#   This program contains the TCP server for receiving SCARA commands.
#
# Author: Dain Kim
# Date Created: 2026-09-04
# Last Modified: 2026-09-04
# -----------------------------------------------------------------------------*/

/*|Includes|-------------------------------------------------------------------*/
#include "tcpCommandServer.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "scaraCommandQueue.h"
#include "scaraConsole.h"
#include "wifiManager.h"

/*|CONSTANTS|------------------------------------------------------------------*/
#define TCP_COMMAND_SERVER_PORT 5000
#define TCP_COMMAND_SERVER_BACKLOG 1
#define TCP_COMMAND_SERVER_RECEIVE_BUFFER_SIZE 128
#define TCP_COMMAND_SERVER_TASK_STACK_SIZE 4096
#define TCP_COMMAND_SERVER_TASK_PRIORITY 1
#define TCP_COMMAND_SERVER_POLL_TIME_MS 500

/*|Function Prototype|---------------------------------------------------------*/
static void tcpCommandServerTask(void *arg);
static int createListeningSocket(void);
static int waitForClient(int listeningSocket);
static void serveCommandClient(int clientSocket);
static bool sendPendingCommandResults(int clientSocket);
static bool sendAll(int socket, const char *data, size_t length);
static void closeSocket(int socket);

/*|Function Definitions|-------------------------------------------------------*/
bool tcpCommandServerInit(void)
{
    return xTaskCreate(tcpCommandServerTask,
                       "tcpCommandServerTask",
                       TCP_COMMAND_SERVER_TASK_STACK_SIZE,
                       NULL,
                       TCP_COMMAND_SERVER_TASK_PRIORITY,
                       NULL) == pdPASS;
}

static void tcpCommandServerTask(void *arg)
{
    (void)arg;

    while (1) {
        // The task exists at boot, but a TCP server is opened only after Wi-Fi gets an IP.
        while (wifiManagerGetStatus() != WIFI_MANAGER_CONNECTED) {
            vTaskDelay(pdMS_TO_TICKS(TCP_COMMAND_SERVER_POLL_TIME_MS));
        }

        int listeningSocket = createListeningSocket();
        if (listeningSocket < 0) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        printf("TCP command server listening on port %d\n", TCP_COMMAND_SERVER_PORT);

        while (wifiManagerGetStatus() == WIFI_MANAGER_CONNECTED) {
            int clientSocket = waitForClient(listeningSocket);
            if (clientSocket < 0) {
                continue;
            }

            printf("TCP client connected\n");
            serveCommandClient(clientSocket);
            closeSocket(clientSocket);
            printf("TCP client disconnected\n");
        }

        closeSocket(listeningSocket);
        printf("TCP command server stopped: Wi-Fi is disconnected\n");
    }
}

static int createListeningSocket(void)
{
    int listeningSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listeningSocket < 0) {
        printf("TCP socket creation failed: errno %d\n", errno);
        return -1;
    }

    int reuseAddress = 1;
    if (setsockopt(listeningSocket,
                   SOL_SOCKET,
                   SO_REUSEADDR,
                   &reuseAddress,
                   sizeof(reuseAddress)) < 0) {
        printf("TCP socket option failed: errno %d\n", errno);
        closeSocket(listeningSocket);
        return -1;
    }

    struct sockaddr_in serverAddress = {
        .sin_family = AF_INET,
        .sin_port = htons(TCP_COMMAND_SERVER_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };

    if (bind(listeningSocket,
             (struct sockaddr *)&serverAddress,
             sizeof(serverAddress)) < 0) {
        printf("TCP bind failed on port %d: errno %d\n",
               TCP_COMMAND_SERVER_PORT,
               errno);
        closeSocket(listeningSocket);
        return -1;
    }

    if (listen(listeningSocket, TCP_COMMAND_SERVER_BACKLOG) < 0) {
        printf("TCP listen failed: errno %d\n", errno);
        closeSocket(listeningSocket);
        return -1;
    }

    return listeningSocket;
}

static int waitForClient(int listeningSocket)
{
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(listeningSocket, &readSet);

    struct timeval timeout = {
        .tv_sec = 0,
        .tv_usec = TCP_COMMAND_SERVER_POLL_TIME_MS * 1000
    };

    int selectResult = select(listeningSocket + 1, &readSet, NULL, NULL, &timeout);
    if (selectResult < 0) {
        printf("TCP accept wait failed: errno %d\n", errno);
        return -1;
    }

    if (selectResult == 0 || !FD_ISSET(listeningSocket, &readSet)) {
        return -1;
    }

    struct sockaddr_in clientAddress;
    socklen_t clientAddressLength = sizeof(clientAddress);
    return accept(listeningSocket,
                  (struct sockaddr *)&clientAddress,
                  &clientAddressLength);
}

static void serveCommandClient(int clientSocket)
{
    static const char welcomeMessage[] =
        "SCARA TCP command server connected\n";
    static const char queuedMessage[] = "QUEUED\n";
    static const char queueFullMessage[] = "ERROR QUEUE_FULL\n";
    static const char commandTooLongMessage[] =
        "ERROR COMMAND_TOO_LONG\n";
    char receiveBuffer[TCP_COMMAND_SERVER_RECEIVE_BUFFER_SIZE];
    char commandBuffer[MAX_SCARA_STRING];
    size_t commandLength = 0;
    bool discardingLongCommand = false;

    if (!sendAll(clientSocket, welcomeMessage, sizeof(welcomeMessage) - 1)) {
        return;
    }

    while (wifiManagerGetStatus() == WIFI_MANAGER_CONNECTED) {
        if (!sendPendingCommandResults(clientSocket)) {
            return;
        }

        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(clientSocket, &readSet);

        struct timeval timeout = {
            .tv_sec = 0,
            .tv_usec = TCP_COMMAND_SERVER_POLL_TIME_MS * 1000
        };

        int selectResult = select(clientSocket + 1, &readSet, NULL, NULL, &timeout);
        if (selectResult < 0) {
            printf("TCP client wait failed: errno %d\n", errno);
            return;
        }

        if (selectResult == 0) {
            continue;
        }

        int receivedLength = recv(clientSocket, receiveBuffer, sizeof(receiveBuffer), 0);
        if (receivedLength == 0) {
            return;
        }
        if (receivedLength < 0) {
            printf("TCP receive failed: errno %d\n", errno);
            return;
        }

        for (int i = 0; i < receivedLength; i++) {
            char receivedCharacter = receiveBuffer[i];

            // Ignore carriage returns so both "\n" and "\r\n" can terminate a command.
            if (receivedCharacter == '\r') {
                continue;
            }

            if (discardingLongCommand) {
                if (receivedCharacter == '\n') {
                    discardingLongCommand = false;
                }
                continue;
            }

            if (receivedCharacter == '\n') {
                if (commandLength == 0) {
                    continue;
                }

                commandBuffer[commandLength] = '\0';
                const char *response;
                size_t responseLength;

                if (scaraCommandQueueSend(commandBuffer, SCARA_COMMAND_SOURCE_TCP)) {
                    response = queuedMessage;
                    responseLength = sizeof(queuedMessage) - 1;
                } else {
                    response = queueFullMessage;
                    responseLength = sizeof(queueFullMessage) - 1;
                }

                if (!sendAll(clientSocket, response, responseLength)) {
                    return;
                }

                commandLength = 0;
                continue;
            }

            if (commandLength >= sizeof(commandBuffer) - 1) {
                if (!sendAll(clientSocket,
                             commandTooLongMessage,
                             sizeof(commandTooLongMessage) - 1)) {
                    return;
                }

                commandLength = 0;
                discardingLongCommand = true;
                continue;
            }

            commandBuffer[commandLength++] = receivedCharacter;
        }
    }
}

static bool sendPendingCommandResults(int clientSocket)
{
    static const char doneMessage[] = "DONE\n";
    static const char errorMessage[] = "ERROR\n";
    SCARA_COMMAND_RESULT result;

    while (scaraCommandResultReceive(&result)) {
        const char *message = result == SCARA_COMMAND_RESULT_DONE
            ? doneMessage
            : errorMessage;
        size_t messageLength = result == SCARA_COMMAND_RESULT_DONE
            ? sizeof(doneMessage) - 1
            : sizeof(errorMessage) - 1;

        if (!sendAll(clientSocket, message, messageLength)) {
            return false;
        }
    }

    return true;
}

static bool sendAll(int socket, const char *data, size_t length)
{
    size_t totalSent = 0;

    while (totalSent < length) {
        int sentLength = send(socket, data + totalSent, length - totalSent, 0);
        if (sentLength <= 0) {
            printf("TCP send failed: errno %d\n", errno);
            return false;
        }

        totalSent += (size_t)sentLength;
    }

    return true;
}

static void closeSocket(int socket)
{
    if (socket >= 0) {
        shutdown(socket, SHUT_RDWR);
        close(socket);
    }
}
