/* QCA4002/4004 implementation of NetworkInterfaceAPI
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

#include "QCA_WiFiInterface.h"
#include "events/EventQueue.h"
#include "events/mbed_shared_queues.h"
#include "features/netsocket/nsapi_types.h"
#include "mbed_trace.h"
#include "platform/Callback.h"
#include "platform/mbed_critical.h"
#include "platform/mbed_debug.h"
#include "platform/mbed_wait_api.h"

#define TRACE_GROUP  "QCAI" // ESP8266 Interface

using namespace mbed;

QCA_WiFiInterface::QCA_WiFiInterface():
    _ap_sec(NSAPI_SECURITY_NONE),
    _if_connected(_cmutex),
    _initialized(false),
    _if_blocking(true),
    _connect_retval(NSAPI_ERROR_OK),
    _conn_stat(NSAPI_STATUS_DISCONNECTED),
    _conn_stat_cb(NULL),
    _global_event_queue(mbed_event_queue()), // Needs to be set before attaching event() to SIGIO
    _connect_event_id(0),
    _channel_num(0)
{
    memset(_ssid, 0, sizeof(_ssid));
    memset(ap_pass, 0, sizeof(ap_pass));

    _qca400x.attach(this, &QCA_WiFiInterface::refresh_conn_state_cb);

    for (int i = 0; i < QCA400X_SOCKET_COUNT; i++) {
        _sock_i[i].open = false;
        _sock_i[i].sport = 0;
    }
}

QCA_WiFiInterface::~QCA_WiFiInterface()
{
    _cmutex.lock();
    if (_connect_event_id) {
        _global_event_queue->cancel(_connect_event_id);
    }
    _cmutex.unlock();
}

nsapi_error_t QCA_WiFiInterface::connect(const char *ssid, const char *pass, nsapi_security_t security,
        uint8_t channel)
{
    nsapi_error_t ret_code;

    ret_code = set_credentials(ssid, pass, security);
    if(ret_code) {
        printf ("set_credentials failed with 0x%x\n", ret_code);
        return ret_code;
    }

    _channel_num = channel;

    return connect();
}

void QCA_WiFiInterface::_connect_async()
{
    _cmutex.lock();
    if (!_connect_event_id) {
        printf("_connect_async(): cancelled");
        _cmutex.unlock();
        return;
    }
    _connect_retval = _qca400x.connect(_ssid, ap_pass, _ap_sec, _channel_num);
    int timeleft_ms = QCA400X_INTERFACE_CONNECT_INTERVAL_MS - _conn_timer.read_ms();
    if (_connect_retval == NSAPI_ERROR_OK || ((_if_blocking == true) && (timeleft_ms <= 0))) {
        _connect_event_id = 0;
        _conn_timer.stop();
        if (timeleft_ms <= 0) {
            _connect_retval = NSAPI_ERROR_CONNECTION_TIMEOUT;
        }
        _if_connected.notify_all();
    } else {
        // Postpone to give other stuff time to run
        _connect_event_id = _global_event_queue->call_in(QCA400X_INTERFACE_CONNECT_INTERVAL_MS,
                                                         callback(this, &QCA_WiFiInterface::_connect_async));
        if (!_connect_event_id) {
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_ENOMEM), \
                            "ESP8266Interface::_connect_async(): unable to add event to queue. Increase \"events.shared-eventsize\"\n");
        }
    }
    _cmutex.unlock();
}

nsapi_error_t QCA_WiFiInterface::connect()
{
    nsapi_error_t status = _conn_status_to_error();
    if (status != NSAPI_ERROR_NO_CONNECTION) {
        return status;
    }

    if (strlen(_ssid) == 0) {
        return NSAPI_ERROR_NO_SSID;
    }

    if (_ap_sec != NSAPI_SECURITY_NONE) {
        if (strlen(ap_pass) < QCA400X_PASSPHRASE_MIN_LENGTH) {
            return NSAPI_ERROR_PARAMETER;
        }
    }

    status = _qca400x.initialize();
    if (status != NSAPI_ERROR_OK) {
        return status;
    }

    _cmutex.lock();

    _connect_retval = NSAPI_ERROR_NO_CONNECTION;
    MBED_ASSERT(!_connect_event_id);
    _conn_timer.stop();
    _conn_timer.reset();
    _conn_timer.start();
    _connect_event_id = _global_event_queue->call(callback(this, &QCA_WiFiInterface::_connect_async));

    if (!_connect_event_id) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_ENOMEM), \
                   "connect(): unable to add event to queue. Increase \"events.shared-eventsize\"\n");
    }

    while (_if_blocking && (_conn_status_to_error() != NSAPI_ERROR_IS_CONNECTED)
            && (_connect_retval == NSAPI_ERROR_NO_CONNECTION)) {
        _if_connected.wait();
    }

    _cmutex.unlock();

    if (!_if_blocking) {
        return NSAPI_ERROR_OK;
    } else {
        return _connect_retval;
    }
}

nsapi_error_t QCA_WiFiInterface::set_credentials(const char *ssid, const char *pass,
                                                       nsapi_security_t security)
{
    nsapi_error_t status = _conn_status_to_error();
    if (status != NSAPI_ERROR_NO_CONNECTION) {
        return NSAPI_ERROR_NO_CONNECTION;
    }

    _ap_sec = security;

    if (!ssid) {
        return NSAPI_ERROR_PARAMETER;
    }

    if (!pass) {
        return NSAPI_ERROR_PARAMETER;
    }

    int pass_length = strlen(pass);
    if (pass_length >= QCA400X_PASSPHRASE_MIN_LENGTH &&
        pass_length <= QCA400X_PASSPHRASE_MAX_LENGTH) {
        memset(ap_pass, 0, sizeof(ap_pass));
        strncpy(ap_pass, pass, sizeof(ap_pass));
    } else {
        return NSAPI_ERROR_PARAMETER;
    }

    int ssid_length = strlen(ssid);

    if (ssid_length > 0
            && ssid_length <= QCA400X_SSID_MAX_LENGTH) {
        memset(_ssid, 0, sizeof(_ssid));
        strncpy(_ssid, ssid, sizeof(_ssid));
    } else {
        return NSAPI_ERROR_PARAMETER;
    }

    if (_ap_sec != NSAPI_SECURITY_NONE) {

        if (!pass) {
            return NSAPI_ERROR_PARAMETER;
        }

        int pass_length = strlen(pass);
        if (pass_length >= QCA400X_PASSPHRASE_MIN_LENGTH
                && pass_length <= QCA400X_PASSPHRASE_MAX_LENGTH) {
            memset(ap_pass, 0, sizeof(ap_pass));
            strncpy(ap_pass, pass, sizeof(ap_pass));
        } else {
            return NSAPI_ERROR_PARAMETER;
        }
    } else {
        memset(ap_pass, 0, sizeof(ap_pass));
    }

    return NSAPI_ERROR_OK;
}

nsapi_error_t QCA_WiFiInterface::set_channel(uint8_t channel)
{
    if (_qca400x.set_channel(channel)) {
        return NSAPI_ERROR_OK;
    }

    return NSAPI_ERROR_DEVICE_ERROR;
}

nsapi_error_t QCA_WiFiInterface::disconnect()
{
    _cmutex.lock();
    if (_connect_event_id) {
        _global_event_queue->cancel(_connect_event_id);
        _connect_event_id = 0; // cancel asynchronous connection attempt if one is ongoing
    }
    _cmutex.unlock();
    _initialized = false;

    nsapi_error_t status = _conn_status_to_error();
    if (status == NSAPI_ERROR_NO_CONNECTION) {
        return NSAPI_ERROR_NO_CONNECTION;
    }

    int ret = _qca400x.disconnect() ? NSAPI_ERROR_OK : NSAPI_ERROR_DEVICE_ERROR;

    return ret;
}

const char *QCA_WiFiInterface::get_ip_address()
{
    const char *ip_buff = _qca400x.getIPAddress();
    if(!ip_buff || strcmp(ip_buff, "0.0.0.0") == 0) {
        return NULL;
    }

    return ip_buff;
}

const char *QCA_WiFiInterface::get_mac_address()
{
    return _qca400x.getMACAddress();
}

const char *QCA_WiFiInterface::get_gateway()
{
    return _conn_stat != NSAPI_STATUS_DISCONNECTED ? _qca400x.getGateway() : NULL;
}

const char *QCA_WiFiInterface::get_netmask()
{
    return _conn_stat != NSAPI_STATUS_DISCONNECTED ? _qca400x.getNetmask() : NULL;
}

int8_t QCA_WiFiInterface::get_rssi()
{
    return _qca400x.rssi();
}

int QCA_WiFiInterface::scan(WiFiAccessPoint *res, unsigned count)
{
    nsapi_error_t status;

    status = _qca400x.initialize();
    if (status != NSAPI_ERROR_OK) {
        return status;
    }

    return _qca400x.scan(res, count);
}

struct qca400x_socket {
    int id;
    nsapi_protocol_t proto;
    bool connected;
    SocketAddress addr;
    int keepalive; // TCP
};

nsapi_error_t QCA_WiFiInterface::socket_open(void **handle, nsapi_protocol_t proto)
{
    // Look for an unused socket
    int id = -1;
    nsapi_error_t ret;

    for (int i = 0; i < QCA400X_SOCKET_COUNT; i++) {
        if (!_sock_i[i].open) {
            id = i;
            _sock_i[i].open = true;
            break;
        }
    }

    if (id == -1) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    ret = _qca400x.open_socket(id, proto);
    if (ret != NSAPI_ERROR_OK) {
        _sock_i[id].open = false;
        return NSAPI_ERROR_NO_SOCKET;
    }

    struct qca400x_socket *socket = new struct qca400x_socket;
    if (!socket) {
        _sock_i[id].open = false;
        return NSAPI_ERROR_NO_SOCKET;
    }

    socket->id = id;
    socket->proto = proto;
    socket->connected = false;
    socket->keepalive = 0;
    *handle = socket;
    return 0;
}

nsapi_error_t QCA_WiFiInterface::socket_close(void *handle)
{
    struct qca400x_socket *socket = (struct qca400x_socket *)handle;
    int err = 0;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (socket->connected && !_qca400x.close_socket(socket->id)) {
        err = NSAPI_ERROR_DEVICE_ERROR;
    }

    _cbs[socket->id].callback = NULL;
    _cbs[socket->id].data = NULL;
    core_util_atomic_store_u8(&_cbs[socket->id].deferred, false);

    socket->connected = false;
    _sock_i[socket->id].open = false;
    _sock_i[socket->id].sport = 0;
    delete socket;
    return err;
}

int QCA_WiFiInterface::socket_bind(void *handle, const SocketAddress &address)
{
    struct qca400x_socket *socket = (struct qca400x_socket *)handle;
    nsapi_error_t ret;
    nsapi_addr_t ip_addr;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (socket->connected) {
        return NSAPI_ERROR_PARAMETER;
    }

    if (socket->proto == NSAPI_UDP) {
        if (address.get_addr().version != NSAPI_UNSPEC) {
            return NSAPI_ERROR_UNSUPPORTED;
        }

        for (int id = 0; id < QCA400X_SOCKET_COUNT; id++) {
            if (_sock_i[id].sport == address.get_port() && id != socket->id) { // Port already reserved by another socket
                return NSAPI_ERROR_PARAMETER;
            }
        }

        ip_addr = address.get_addr();
        ret = _qca400x.bind_socket(socket->id, ip_addr.bytes, address.get_port());
        if (ret == NSAPI_ERROR_OK) {
            _sock_i[socket->id].sport = address.get_port();
            return 0;
        }
    }

    return NSAPI_ERROR_UNSUPPORTED;
}

int QCA_WiFiInterface::socket_listen(void *handle, int backlog)
{
    return NSAPI_ERROR_UNSUPPORTED;
}

int QCA_WiFiInterface::socket_connect(void *handle, const SocketAddress &addr)
{
    struct qca400x_socket *socket = (struct qca400x_socket *)handle;
    nsapi_error_t ret;
    nsapi_addr_t ip_addr;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    ip_addr = addr.get_addr();

    ret = _qca400x.connect_socket(socket->id, ip_addr.bytes, addr.get_port());

    socket->connected = (ret == NSAPI_ERROR_OK) ? true : false;

    return ret;

}

int QCA_WiFiInterface::socket_accept(void *handle, void **socket, SocketAddress *address)
{
    return NSAPI_ERROR_UNSUPPORTED;
}

int QCA_WiFiInterface::socket_send(void *handle, const void *data, unsigned size)
{
    nsapi_error_t status;
    struct qca400x_socket *socket = (struct qca400x_socket *)handle;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (!socket->connected) {
        return NSAPI_ERROR_CONNECTION_LOST;
    }

    if (data == NULL) {
        return NSAPI_ERROR_PARAMETER;
    }

    status = _qca400x.send(socket->id, data, size);

    return status != NSAPI_ERROR_OK ? status : size;

}

int QCA_WiFiInterface::socket_recv(void *handle, void *data, unsigned size)
{
    struct qca400x_socket *socket = (struct qca400x_socket *)handle;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (!_sock_i[socket->id].open) {
        return NSAPI_ERROR_CONNECTION_LOST;
    }

    int32_t recv;

    recv = _qca400x.recv(socket->id, data, size);
    if (recv <= 0 && recv != NSAPI_ERROR_WOULD_BLOCK) {
        socket->connected = false;
    }

    return recv;

}

int QCA_WiFiInterface::socket_sendto(void *handle, const SocketAddress &addr, const void *data, unsigned size)
{
    struct qca400x_socket *socket = (struct qca400x_socket *)handle;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if ((strcmp(addr.get_ip_address(), "0.0.0.0") == 0) || !addr.get_port())  {
        return NSAPI_ERROR_DNS_FAILURE;
    }

    if (socket->connected && socket->addr != addr) {
        if (!_qca400x.close_socket(socket->id)) {
            return NSAPI_ERROR_DEVICE_ERROR;
        }
        socket->connected = false;
    }

    if (!socket->connected) {
        int err = socket_connect(socket, addr);
        if (err < 0) {
            return err;
        }
        socket->addr = addr;
    }

    return socket_send(socket, data, size);
}

int QCA_WiFiInterface::socket_recvfrom(void *handle, SocketAddress *addr, void *data, unsigned size)
{
    struct qca400x_socket *socket = (struct qca400x_socket *)handle;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    int ret = socket_recv(socket, data, size);
    if (ret >= 0 && addr) {
        *addr = socket->addr;
    }

    return ret;
}

void QCA_WiFiInterface::socket_attach(void *handle, void (*callback)(void *), void *data)
{
    struct qca400x_socket *socket = (struct qca400x_socket *)handle;
    _cbs[socket->id].callback = callback;
    _cbs[socket->id].data = data;
}

nsapi_error_t QCA_WiFiInterface::setsockopt(nsapi_socket_t handle, int level,
        int optname, const void *optval, unsigned optlen)
{
    struct qca400x_socket *socket = (struct qca400x_socket *)handle;

    if (!optlen) {
        return NSAPI_ERROR_PARAMETER;
    } else if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (level == NSAPI_SOCKET && socket->proto == NSAPI_TCP) {
        switch (optname) {
            case NSAPI_KEEPALIVE: {
                if (socket->connected) { // ESP8266 limitation, keepalive needs to be given before connecting
                    return NSAPI_ERROR_UNSUPPORTED;
                }

                if (optlen == sizeof(int)) {
                    int secs = *(int *)optval;
                    if (secs  >= 0 && secs <= 7200) {
                        socket->keepalive = secs;
                        return NSAPI_ERROR_OK;
                    }
                }
                return NSAPI_ERROR_PARAMETER;
            }
        }
    }

    return NSAPI_ERROR_UNSUPPORTED;

}

nsapi_error_t QCA_WiFiInterface::getsockopt(nsapi_socket_t handle, int level, int optname,
        void *optval, unsigned *optlen)
{
    struct qca400x_socket *socket = (struct qca400x_socket *)handle;

    if (!optval || !optlen) {
        return NSAPI_ERROR_PARAMETER;
    } else if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (level == NSAPI_SOCKET && socket->proto == NSAPI_TCP) {
        switch (optname) {
            case NSAPI_KEEPALIVE: {
                if (*optlen > sizeof(int)) {
                    *optlen = sizeof(int);
                }
                memcpy(optval, &(socket->keepalive), *optlen);
                return NSAPI_ERROR_OK;
            }
        }
    }

    return NSAPI_ERROR_UNSUPPORTED;

}

void QCA_WiFiInterface::attach(Callback<void(nsapi_event_t, intptr_t)> status_cb)
{
    _conn_stat_cb = status_cb;
}

nsapi_connection_status_t QCA_WiFiInterface::get_connection_status() const
{
    return _conn_stat;
}

#if MBED_CONF_QCA400X_PROVIDE_DEFAULT

WiFiInterface *WiFiInterface::get_default_instance()
{
    static QCA_WiFiInterface wifi;
    return &wifi;
}

#endif

void QCA_WiFiInterface::refresh_conn_state_cb()
{
    nsapi_connection_status_t prev_stat = _conn_stat;
    _conn_stat = _qca400x.connection_status();

    switch (_conn_stat) {
        // Doesn't require changes
        case NSAPI_STATUS_CONNECTING:
        case NSAPI_STATUS_GLOBAL_UP:
            break;
        // Start from scratch if connection drops/is dropped
        case NSAPI_STATUS_DISCONNECTED:
            break;
        // Handled on AT layer
        case NSAPI_STATUS_LOCAL_UP:
        case NSAPI_STATUS_ERROR_UNSUPPORTED:
        default:
            _initialized = false;
            _conn_stat = NSAPI_STATUS_DISCONNECTED;
            for (int i = 0; i < QCA400X_SOCKET_COUNT; i++) {
                _sock_i[i].open = false;
                _sock_i[i].sport = 0;
            }
    }

    if (prev_stat == _conn_stat) {
        return;
    }

    tr_debug("refresh_conn_state_cb(): changed to %d", _conn_stat);

    // Inform upper layers
    if (_conn_stat_cb) {
        _conn_stat_cb(NSAPI_EVENT_CONNECTION_STATUS_CHANGE, _conn_stat);
    }
}

nsapi_error_t QCA_WiFiInterface::_conn_status_to_error()
{
    nsapi_error_t ret;

    switch (_conn_stat) {
        case NSAPI_STATUS_DISCONNECTED:
            ret = NSAPI_ERROR_NO_CONNECTION;
            break;
        case NSAPI_STATUS_CONNECTING:
            ret = NSAPI_ERROR_ALREADY;
            break;
        case NSAPI_STATUS_GLOBAL_UP:
            ret = NSAPI_ERROR_IS_CONNECTED;
            break;
        default:
            ret = NSAPI_ERROR_DEVICE_ERROR;
    }

    return ret;
}

nsapi_error_t QCA_WiFiInterface::set_blocking(bool blocking)
{
    _if_blocking = blocking;

    return NSAPI_ERROR_OK;
}

