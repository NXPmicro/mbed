
/* QCA400x Class
 * Copyright (c) 2019 ARM Limited
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

#ifndef QCA400x_H
#define QCA400x_H

#include "rtos/Mutex.h"
#include "features/netsocket/nsapi_types.h"
#include "features/netsocket/WiFiAccessPoint.h"
#include "platform/Callback.h"
#include "qcom_api.h"
#include "wlan_qcom.h"

// Various timeouts for different QCA400x operations
#ifndef QCA400x_CONNECT_TIMEOUT
#define QCA400x_CONNECT_TIMEOUT 15000
#endif
#ifndef QCA400x_SEND_TIMEOUT
#define QCA400x_SEND_TIMEOUT    2000
#endif
#ifndef QCA400x_RECV_TIMEOUT
#define QCA400x_RECV_TIMEOUT    2000
#endif
#ifndef QCA400x_MISC_TIMEOUT
#define QCA400x_MISC_TIMEOUT    2000
#endif

#define QCA400X_SCAN_TIME_MIN 0     // [ms]
#define QCA400X_SCAN_TIME_MAX 1500  // [ms]
#define QCA400X_SCAN_TIME_MIN_DEFAULT 120 // [ms]
#define QCA400X_SCAN_TIME_MAX_DEFAULT 360 // [ms]

/**
 * @brief Max SSID length.
 */
#define wificonfigMAX_SSID_LEN                ( 32 )

/**
 * @brief Max BSSID length.
 */
#define wificonfigMAX_BSSID_LEN               ( 6 )

/**
 * This is the interface class to wifi_qca
 */
class QCA400x
{
public:
    QCA400x();

    /**
    * Initialize QCA400x to default state
    */
    nsapi_error_t initialize();

    /**
    * Connect QCA400x to AP
    *
    * @param ssid the name of the AP
    * @param passPhrase the password of AP
    * @param security security type
    * @param channel channel on which the connection is to be made
    * @return NSAPI_ERROR_OK only if QCA400x is connected successfully
    */
    nsapi_error_t connect(const char *ssid, const char *passPhrase, nsapi_security_t security, uint8_t channel);

    /**
    * Disconnect QCA400x from AP
    *
    * @return true only if QCA400x is disconnected successfully
    */
    bool disconnect(void);

    /**
    * Set the channel on which the connection is to be made
    *
    * @param channel channel on which the connection is to be made
    * @return NSAPI_ERROR_OK only if channel set successfully
    */
    bool set_channel(uint8_t channel);

    /**
    * Get the IP address of QCA400x
    *
    * @return null-teriminated IP address or null if no IP address is assigned
    */
    const char *getIPAddress(void);

    /**
    * Get the MAC address of QCA400x
    *
    * @return null-terminated MAC address or null if no MAC address is assigned
    */
    const char *getMACAddress(void);

     /** Get the local gateway
     *
     *  @return         Null-terminated representation of the local gateway
     *                  or null if no network mask has been received
     */
    const char *getGateway();

    /** Get the local network mask
     *
     *  @return         Null-terminated representation of the local network mask
     *                  or null if no network mask has been recieved
     */
    const char *getNetmask();

    /** Scan for available networks
     *
     * @param  ap    Pointer to allocated array to store discovered AP
     * @param  limit Size of allocated @a res array, or 0 to only count available AP
     * @return       Number of entries in @a res, or if @a count was 0 number of available networks, negative on error
     *               see @a nsapi_error
     */
    int scan(WiFiAccessPoint *res, unsigned count);

    /** Set channel
     *
     * @param  channel    channel to set
     * @return            NSAPI error code
     */
    nsapi_error_t set_channel(int channel);

    /**Perform a dns query
    *
    * @param name Hostname to resolve
    * @param ip   Buffer to store IP address
    * @return     NSAPI error code
    */
    nsapi_error_t dns_lookup(const char *name, char *ip, uint32_t ip_size, nsapi_version_t version);

    /* Return RSSI for active connection
     *
     * @return      Measured RSSI
     */
    int8_t rssi();

    /**
    * Attach a function to call whenever network state has changed.
    *
    * @param func A pointer to a void function, or 0 to set as none
    */
    void attach(mbed::Callback<void()> status_cb);

    template <typename T, typename M>
    void attach(T *obj, M method)
    {
        attach(mbed::Callback<void()>(obj, method));
    }

    /**
    * Open a socketed connection
    *
    * @param id id to give the new socket, valid 0-4
    * @param type the type of socket to open "UDP" or "TCP"
    * @return NSAPI_ERROR_OK in success, negative error code in failure
    */
    nsapi_error_t open_socket(int id, nsapi_protocol_t proto);

    /**
    * Bind a specific address to a socket
    *
    * @param id id to give the new socket, valid 0-4
    * @param address local IP address to bind
    * @param port local port number
    * @return NSAPI_ERROR_OK in success, negative error code in failure
    */
    nsapi_error_t bind_socket(int id, const uint8_t *addr, int port);

    /**
    * Close a socket
    *
    * @param id id to give the new socket, valid 0-4
    * @return true if socket was closed successfully, false otherwise
    */
    bool close_socket(int id);

    /**
    * Initiate a connection on a socket (blocking)
    *
    * @param id id to give the new socket, valid 0-4
    * @param address IP address of the destination
    * @param port port on the destination
    *
    * @return NSAPI_ERROR_OK in success, negative error code in failure
    */
    int connect_socket(int id, const uint8_t *addr, int port);

    /**
    * Sends data to an open socket (nonblocking)
    *
    * @param id id to give the new socket, valid 0-4
    * @param data data to be sent
    * @param size number of bytes to be sent - max 1024
    * @return NSAPI_ERROR_OK in success, negative error code in failure
    */
    int32_t send(int id, const void *data, uint32_t size);

    /**
    * Receives stream data from an open TCP socket (nonblocking)
    *
    * @param id id to give the new socket, valid 0-4
    * @param data placeholder for returned information
    * @param size number of bytes to be received
    * @return the number of bytes received
    */
    int32_t recv(int id, void *data, uint32_t size);

    /** Get the connection status
     *
     *  @return         The connection status according to ConnectionStatusType
     */
    nsapi_connection_status_t connection_status() const;

    static const int8_t SOCKET_COUNT = 5;

private:

    rtos::Mutex _smutex; // Protect accesses

    // Add private members
    int _initialized;

    // Address info
    char _ip_buffer[16];
    char _gateway_buffer[16];
    char _netmask_buffer[16];
    char _mac_buffer[ATH_MAC_LEN];

    // Socket info
    struct _sock_info {
        bool open;
        nsapi_protocol_t proto;
        int32_t xSocket;
    };

    struct _sock_info _sock_i[SOCKET_COUNT];
};
#endif /* QCA400x_H_ */

