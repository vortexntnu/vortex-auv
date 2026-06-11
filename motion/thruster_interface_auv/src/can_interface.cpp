#include "can_interface.hpp"
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

can_interface::can_interface() : socket_fd_(-1), is_initialized_(false) {}

can_interface::~can_interface() {
    if (is_initialized_) {
        close(socket_fd_);
    }
}

can_status can_interface::init(const std::string& ifname) {
    interface_name_ = ifname;

    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        return can_status::ERR_SOCKET;
    }

    int enable_canfd = 1;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd,
                   sizeof(enable_canfd)) < 0) {
        close(socket_fd_);
        return can_status::ERR_CANFD_SUPPORT;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, interface_name_.c_str());
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        close(socket_fd_);
        return can_status::ERR_INTERFACE_INDEX;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(socket_fd_);
        return can_status::ERR_BIND;
    }

    is_initialized_ = true;

    return can_status::OK;
}

can_status can_interface::send(uint32_t can_id,
                               const uint8_t* data,
                               uint8_t len,
                               bool use_brs) {
    if (!is_initialized_) {
        return can_status::ERR_NOT_INITIALIZED;
    }

    if (len > 64) {
        return can_status::ERR_DATA_LENGTH;
    }

    struct canfd_frame frame;
    memset(&frame, 0, sizeof(frame));

    frame.can_id = can_id;
    frame.len = len;
    frame.flags = use_brs ? CANFD_BRS : 0;

    if (data != nullptr && len > 0) {
        memcpy(frame.data, data, len);
    }

    if (write(socket_fd_, &frame, sizeof(frame)) != sizeof(frame)) {
        return can_status::ERR_SEND;
    }

    return can_status::OK;
}

can_status can_interface::receive(struct canfd_frame& frame) {
    if (!is_initialized_) {
        return can_status::ERR_NOT_INITIALIZED;
    }

    ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));

    if (nbytes != CANFD_MTU) {
        return can_status::ERR_RECEIVE;
    }

    return can_status::OK;
}

can_status can_interface::receive(struct canfd_frame& frame, int timeout_ms) {
    if (!is_initialized_) {
        return can_status::ERR_NOT_INITIALIZED;
    }

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        return can_status::ERR_SETTING_TIMEOUT;
    }

    ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));

    if (nbytes != CANFD_MTU) {
        return can_status::ERR_RECEIVE;
    }

    return can_status::OK;
}

can_status can_interface::start_async_receive(
    std::function<void(const struct canfd_frame&, can_status)> callback) {
    if (!is_initialized_) {
        return can_status::ERR_NOT_INITIALIZED;
    }

    receiving_ = true;

    receive_thread_ = std::thread([this, callback]() {
        while (receiving_) {
            struct canfd_frame frame;
            can_status status = receive(frame, 1000);
            callback(frame, status);
        }
    });
    return can_status::OK;
}

void can_interface::stop_async_receive() {
    receiving_ = false;
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
}

can_status can_interface::set_filter(uint32_t can_id, uint32_t can_mask) {
    if (!is_initialized_) {
        return can_status::ERR_NOT_INITIALIZED;
    }

    struct can_filter filter;
    filter.can_id = can_id;
    filter.can_mask = can_mask;

    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter,
                   sizeof(filter)) < 0) {
        return can_status::ERR_SETTING_FILTER;
    }

    return can_status::OK;
}

can_status can_interface::set_filters(const struct can_filter* filters,
                                      size_t num_filters) {
    if (!is_initialized_) {
        return can_status::ERR_NOT_INITIALIZED;
    }

    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters,
                   num_filters * sizeof(struct can_filter)) < 0) {
        return can_status::ERR_SETTING_FILTER;
    }

    return can_status::OK;
}

can_status can_interface::clear_filters() {
    if (!is_initialized_) {
        return can_status::ERR_NOT_INITIALIZED;
    }

    struct can_filter filter;
    filter.can_id = 0x0;
    filter.can_mask = 0x0;

    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter,
                   sizeof(filter)) < 0) {
        return can_status::ERR_CLEARING_FILTERS;
    }

    return can_status::OK;
}

bool can_interface::is_initialized() const {
    return is_initialized_;
}

std::string can_interface::get_interface_name() const {
    return interface_name_;
}
