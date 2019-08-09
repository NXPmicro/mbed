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

#include <stdlib.h>
#include "QCA400x.h"
#include "ip4string.h"

// Convert comma separated bytes to a uint32_t IP address
#define CSV_BYTES_TO_UINT32_IPADDR(a0, a1, a2, a3) \
    (((uint32_t)(a0)&0xFF) << 24) | (((uint32_t)(a1)&0xFF) << 16) | (((uint32_t)(a2)&0xFF) << 8) | ((uint32_t)(a3)&0xFF)

// Connection state reporting
mbed::Callback<void()> _conn_stat_cb; // QCA400XInterface registered
nsapi_connection_status_t _conn_status;
static osSemaphoreId_t conn_event;
static mbed_rtos_storage_semaphore_t event_sem_storage;

using namespace mbed;

QCA400x::QCA400x()
    : _initialized(false)
{
    _conn_status = NSAPI_STATUS_DISCONNECTED;

    for (int i = 0; i < SOCKET_COUNT; i++) {
        _sock_i[i].open = false;
        _sock_i[i].proto = NSAPI_UDP;
        _sock_i[i].xSocket = 0;
    }
    osSemaphoreAttr_t event_sem_attr = { 0 };
    event_sem_attr.cb_mem = &event_sem_storage;
    event_sem_attr.cb_size = sizeof(event_sem_storage);

    conn_event = osSemaphoreNew(1, 0, &event_sem_attr);
}

nsapi_error_t QCA400x::initialize()
{
    int32_t result = 0;

    if (_initialized) {
        return NSAPI_ERROR_OK;
    }

    _smutex.lock();

    /* Initialize WIFI shield */
    result = WIFISHIELD_Init();
    if (result != A_OK) {
        _smutex.unlock();
        return NSAPI_ERROR_DEVICE_ERROR;
    }

    /* Initialize the WIFI driver (thus starting the driver task) */
    result = wlan_driver_start();
    if (result != A_OK) {
        _smutex.unlock();
        return NSAPI_ERROR_DEVICE_ERROR;
    }

    for (int i = 0; i < SOCKET_COUNT; i++) {
        _sock_i[i].open = false;
        _sock_i[i].proto = NSAPI_UDP;
        _sock_i[i].xSocket = 0;
    }

    _smutex.unlock();

    _initialized = true;

    return NSAPI_ERROR_OK;
}

static void connect_cb(uint8_t event, uint8_t devId, char *bssid, uint8_t bssConn)
{
    bool release_sem = false;

    switch (event)
    {
        case QCOM_ONCONNECT_EVENT_SUCCESS:
            /* Do nothing, wait for 4-way handshake */
            // NOTE that station is not fully connected until PEER_FIRST_NODE_JOIN_EVENT
            break;
        case INVALID_PROFILE:
            // this event is used to indicate RSNA failure
            _conn_status = NSAPI_STATUS_DISCONNECTED;
            release_sem = true;
            break;
        case PEER_FIRST_NODE_JOIN_EVENT:
            // this event is used to RSNA success
            _conn_status = NSAPI_STATUS_CONNECTING;
            release_sem = true;
            break;
        case QCOM_ONCONNECT_EVENT_DISCONNECT:
            _conn_status = NSAPI_STATUS_DISCONNECTED;
            release_sem = true;
            break;
        default:
            //printf("Code = %d\r\n", event);
            release_sem = true;
            break;
    }

    if (release_sem) {
        MBED_ASSERT(_conn_stat_cb);
        _conn_stat_cb();

        osSemaphoreRelease(conn_event);
    }
}

nsapi_error_t QCA400x::connect(const char *ssid, const char *passPhrase, nsapi_security_t security,
                                  uint8_t channel)
{
    WLAN_AUTH_MODE g_auth;
    WLAN_CRYPT_TYPE g_cipher;
    QCOM_SSID g_ssid;
    QCOM_PASSPHRASE g_passphrase;

    switch (security) {
        case NSAPI_SECURITY_NONE:
            g_auth = WLAN_AUTH_NONE;
            g_cipher = WLAN_CRYPT_NONE;
            break;
        case NSAPI_SECURITY_WEP:
            g_auth = WLAN_AUTH_NONE;
            g_cipher = WLAN_CRYPT_WEP_CRYPT;
            break;
        case NSAPI_SECURITY_WPA:
            g_auth = WLAN_AUTH_WPA_PSK;
            g_cipher = WLAN_CRYPT_TKIP_CRYPT;
            break;
        case NSAPI_SECURITY_WPA2:
        case NSAPI_SECURITY_WPA_WPA2:
            g_auth = WLAN_AUTH_WPA2_PSK;
            g_cipher = WLAN_CRYPT_AES_CRYPT;
            break;
        case NSAPI_SECURITY_PAP:
        case NSAPI_SECURITY_CHAP:
        default:
            return NSAPI_ERROR_UNSUPPORTED;
    }

    _smutex.lock();

    /* Set Wi-Fi to device mode */
    if (A_OK != (A_STATUS)qcom_op_set_mode(0, QCOM_WLAN_DEV_MODE_STATION)) {
        _smutex.unlock();
        return NSAPI_ERROR_NO_CONNECTION;
    }

    /* Set SSID, must be done before auth, cipher and passphrase */
    strncpy(g_ssid.ssid, ssid, sizeof(g_ssid.ssid));
    if (A_OK != (A_STATUS)qcom_set_ssid(0, &g_ssid)) {
        _smutex.unlock();
        return NSAPI_ERROR_NO_CONNECTION;
    }

    /* Set encyption mode */
    if (A_OK != (A_STATUS)qcom_sec_set_encrypt_mode(0, g_cipher)) {
        _smutex.unlock();
        return NSAPI_ERROR_NO_CONNECTION;
    }

    /* Set auth mode */
    if (A_OK != qcom_sec_set_auth_mode(0, g_auth)) {
        _smutex.unlock();
        return NSAPI_ERROR_NO_CONNECTION;
    }

    /* Set passphrase */
    strncpy(g_passphrase.passphrase, passPhrase, sizeof(g_passphrase.passphrase));
    if (A_OK != qcom_sec_set_passphrase(0, &g_passphrase)) {
        _smutex.unlock();
        return NSAPI_ERROR_NO_CONNECTION;
    }

    /* Set channel */
    if (0 != channel) {
        if (A_OK != qcom_set_channel(0, channel)) {
            _smutex.unlock();
            return NSAPI_ERROR_NO_CONNECTION;
        }
    }

    /* Set connect_callback */
    if (A_OK != qcom_set_connect_callback(0, (void *)connect_cb)) {
        _smutex.unlock();
        return NSAPI_ERROR_NO_CONNECTION;
    }

    /* Commit settings to Wi-Fi module */
    if (A_OK != qcom_commit(0)) {
        _smutex.unlock();
        return NSAPI_ERROR_NO_CONNECTION;
    }

    /* Wait for callback to indicate connection is complete */
    /* semaphore is released in callback fn */
    if (osOK != osSemaphoreAcquire(conn_event, QCA400x_CONNECT_TIMEOUT)) {
        _smutex.unlock();
        return NSAPI_ERROR_NO_CONNECTION;
    }

    if (A_OK != getDhcp()) {
        _smutex.unlock();
        return NSAPI_ERROR_DHCP_FAILURE;
    }

    _smutex.unlock();

    return NSAPI_ERROR_OK;
}

bool QCA400x::disconnect()
{
    int32_t result = 0;

    _smutex.lock();

    result = apDisconnect();

    _smutex.unlock();

    if (result != A_OK) {
        return false;
    }

    return true;
}

bool QCA400x::set_channel(uint8_t channel)
{
    _smutex.lock();

    /* Set channel */
    if (0 != channel) {
        if (A_OK != qcom_set_channel(0, channel)) {
            _smutex.unlock();
            return false;
        }
    }

    _smutex.unlock();

    return true;
}

const char *QCA400x::getIPAddress()
{
    uint32_t address;
    uint8_t bytes[4];

    _smutex.lock();

    if (A_OK != qcom_ipconfig(0, QCOM_IPCONFIG_QUERY, &address, NULL, NULL)) {
        _smutex.unlock();
        return 0;
    }

    _smutex.unlock();

    for (int i = 0; i < 4; i++) {
        bytes[i] = (address >> (8 * (3 - i))) & 0xFF;
    }

    ip4tos(bytes, _ip_buffer);

    return _ip_buffer;
}

const char *QCA400x::getMACAddress()
{
    _smutex.lock();

    if (A_OK != qcom_get_bssid(0, (uint8_t *)_mac_buffer)) {
        _smutex.unlock();
        return 0;
    }

    _smutex.unlock();

    return _mac_buffer;
}

const char *QCA400x::getGateway()
{
    uint32_t address;
    uint8_t bytes[4];

    _smutex.lock();

    if (A_OK != qcom_ipconfig(0, QCOM_IPCONFIG_QUERY, NULL, NULL, &address)) {
        _smutex.unlock();
        return 0;
    }

    _smutex.unlock();

    for (int i = 0; i < 4; i++) {
        bytes[i] = (address >> (8 * (3 - i))) & 0xFF;
    }

    ip4tos(bytes, _gateway_buffer);

    return _gateway_buffer;
}

const char *QCA400x::getNetmask()
{
    uint32_t address;
    uint8_t bytes[4];

    _smutex.lock();

    if (A_OK != qcom_ipconfig(0, QCOM_IPCONFIG_QUERY, NULL, &address, NULL)) {
        _smutex.unlock();
        return 0;
    }

    _smutex.unlock();

    for (int i = 0; i < 4; i++) {
        bytes[i] = (address >> (8 * (3 - i))) & 0xFF;
    }

    ip4tos(bytes, _netmask_buffer);

    return _netmask_buffer;
}

int QCA400x::scan(WiFiAccessPoint *res, unsigned count)
{
    QCOM_BSS_SCAN_INFO *scan_result = NULL;
    unsigned int scan_result_num = 0;
    nsapi_wifi_ap_t ap;
    unsigned int i;

    _smutex.lock();

    /* Note: won't work if SSID is set already */
    if (A_OK != qcom_set_scan(0, NULL)) {
        _smutex.unlock();
        return NSAPI_ERROR_DEVICE_ERROR;
    }

    if (qcom_get_scan(0, &scan_result, (int16_t *)&scan_result_num)) {
        _smutex.unlock();
        return NSAPI_ERROR_DEVICE_ERROR;
    }

    for (i = 0; i < scan_result_num; i++) {
        strncpy((char*)ap.ssid, (char const*)scan_result[i].ssid, wificonfigMAX_SSID_LEN);
        strncpy((char*)ap.bssid, (char const*)scan_result[i].bssid, wificonfigMAX_BSSID_LEN);
        ap.rssi = scan_result[i].rssi;
        ap.channel = scan_result[i].channel;
        if (!scan_result[i].security_enabled) {
            ap.security = NSAPI_SECURITY_NONE;
        } else if ((0 == scan_result[i].rsn_cipher) ||
                   (ATH_CIPHER_TYPE_WEP & scan_result[i].rsn_cipher)) {
            ap.security = NSAPI_SECURITY_WEP;
        } else if (ATH_CIPHER_TYPE_CCMP & scan_result[i].rsn_cipher) {
            ap.security = NSAPI_SECURITY_WPA2;
        } else {
            ap.security = NSAPI_SECURITY_WPA;
        }

        if (i < count) {
            res[i] = WiFiAccessPoint(ap);
        }

        if (count != 0 && i >= count) {
            break;
        }
    }

    _smutex.unlock();

    return i;
}

nsapi_error_t QCA400x::set_channel(int channel)
{
    return NSAPI_ERROR_UNSUPPORTED;
}

nsapi_error_t QCA400x::dns_lookup(const char *name, char *ip, uint32_t ip_size, nsapi_version_t version)
{
    return NSAPI_ERROR_OK;
}

int8_t QCA400x::rssi()
{
    uint8_t rssi_val;

    _smutex.lock();
    if (qcom_sta_get_rssi(0, &rssi_val) != A_OK) {
        _smutex.unlock();
        return 0;
    }

    _smutex.unlock();

    return rssi_val;
}

void QCA400x::attach(Callback<void()> status_cb)
{
    _conn_stat_cb = status_cb;
}

nsapi_error_t QCA400x::open_socket(int id, nsapi_protocol_t proto)
{
    if (id >= SOCKET_COUNT || _sock_i[id].open) {
        return NSAPI_ERROR_PARAMETER;
    }

    _smutex.lock();

    /* Create socket */
    if (proto == NSAPI_TCP) {
        _sock_i[id].xSocket = qcom_socket(ATH_AF_INET, SOCK_STREAM_TYPE, 0);
    } else {
        _sock_i[id].xSocket = qcom_socket(ATH_AF_INET, SOCK_DGRAM_TYPE, 0);
    }

    if (-1 == _sock_i[id].xSocket) {
        _smutex.unlock();
        return NSAPI_ERROR_DEVICE_ERROR;
    }

    _smutex.unlock();

    _sock_i[id].open = true;
    _sock_i[id].proto = proto;

    return NSAPI_ERROR_OK;
}

nsapi_error_t QCA400x::bind_socket(int id, const uint8_t *addr, int port)
{
    SOCKADDR_T localAddr;
    int status;

    /* Check if socket has been opened */
    if (id >= SOCKET_COUNT || !(_sock_i[id].open)) {
        return NSAPI_ERROR_PARAMETER;
    }

    memset(&localAddr, 0, sizeof(localAddr));
    localAddr.sin_port = port;
    localAddr.sin_family = ATH_AF_INET;
    localAddr.sin_addr.s_addr = CSV_BYTES_TO_UINT32_IPADDR(*addr, *(addr + 1), *(addr + 2), *(addr + 3));

    _smutex.lock();

    status = qcom_bind(_sock_i[id].xSocket, (struct sockaddr *)(&localAddr), sizeof(localAddr));
    if (status != A_OK) {
        _smutex.unlock();
        return NSAPI_ERROR_DEVICE_ERROR;
    }

    _smutex.unlock();

    return NSAPI_ERROR_OK;
}

bool QCA400x::close_socket(int id)
{
    int status;

    /* Check if socket has been opened */
    if (id >= SOCKET_COUNT || !(_sock_i[id].open)) {
        return false;
    }

    _smutex.lock();

    status = qcom_socket_close(_sock_i[id].xSocket);

    if (status != A_OK) {
        _smutex.unlock();
        return false;
    }

    _smutex.unlock();

    _sock_i[id].open = false;

    return true;
}

int QCA400x::connect_socket(int id, const uint8_t *addr, int port)
{
    SOCKADDR_T localAddr;
    int32_t xStatus = 0;

    /* Check if socket has been opened */
    if (id >= SOCKET_COUNT || !(_sock_i[id].open)) {
        return NSAPI_ERROR_PARAMETER;
    }

    _smutex.lock();

    memset(&localAddr, 0, sizeof(localAddr));
    localAddr.sin_port = port;
    localAddr.sin_family = ATH_AF_INET;
    localAddr.sin_addr.s_addr = CSV_BYTES_TO_UINT32_IPADDR(*addr, *(addr + 1), *(addr + 2), *(addr + 3));

    xStatus = qcom_connect(_sock_i[id].xSocket, (struct sockaddr *)&localAddr, sizeof(localAddr));

    if (0 != xStatus) {
        _smutex.unlock();
        return NSAPI_ERROR_DEVICE_ERROR;;
    }

    _smutex.unlock();

    return NSAPI_ERROR_OK;
}

int32_t QCA400x::send(int id, const void *data, uint32_t size)
{
    char *sendBuf;

    _smutex.lock();

    sendBuf = (char *)custom_alloc(size);
    if( sendBuf == NULL ) {
        _smutex.unlock();
        return NSAPI_ERROR_NO_MEMORY;
    }

    memcpy(sendBuf, data, size);
    int ret = qcom_send(_sock_i[id].xSocket, sendBuf, size, 0);
    custom_free( sendBuf );

    if (ret < 0) {
        _smutex.unlock();
        return NSAPI_ERROR_DEVICE_ERROR;
    }
    _smutex.unlock();

    return NSAPI_ERROR_OK;
}

int32_t QCA400x::recv(int id, void *data, uint32_t size)
{
    int xRetVal = 0;

    _smutex.lock();

    xRetVal = qcom_recv(_sock_i[id].xSocket, (char **)&data, size, 0);

    if (xRetVal < 0 ) {
        _smutex.unlock();
        return NSAPI_ERROR_DEVICE_ERROR;
    }

    _smutex.unlock();

    return NSAPI_ERROR_OK;
}

nsapi_connection_status_t QCA400x::connection_status() const
{
    return _conn_status;
}

