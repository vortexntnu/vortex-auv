#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include <linux/can.h>
#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

enum class can_status {
    OK = 0,
    ERR_ALREADY_INITIALIZED = -1,
    ERR_SOCKET = -2,
    ERR_CANFD_SUPPORT = -3,
    ERR_LOOPBACK = -4,
    ERR_INTERFACE_INDEX = -5,
    ERR_BIND = -6,
    ERR_NOT_INITIALIZED = -7,
    ERR_DATA_LENGTH = -8,
    ERR_SEND = -9,
    ERR_RECEIVE = -10,
    ERR_SETTING_TIMEOUT = -11,
    ERR_SETTING_FILTER = -12,
    ERR_CLEARING_FILTERS = -13
};

class can_interface {
   private:
    int socket_fd_;
    bool is_initialized_;
    std::string interface_name_;

    std::thread receive_thread_;
    std::atomic<bool> receiving_ = false;

   public:
    can_interface();
    ~can_interface();

    can_status init(const std::string& ifname = "can0");
    can_status send(uint32_t can_id,
                    const uint8_t* data,
                    uint8_t len,
                    bool use_brs = true);
    can_status receive(struct canfd_frame& frame);
    can_status receive(struct canfd_frame& frame, int timeout_ms);
    can_status start_async_receive(
        std::function<void(const struct canfd_frame&, can_status)> callback);
    void stop_async_receive();
    can_status set_filter(uint32_t can_id, uint32_t can_mask = 0x7FF);
    can_status set_filters(const struct can_filter* filters,
                           size_t num_filters);
    can_status clear_filters();
    bool is_initialized() const;
    std::string get_interface_name() const;
};

#endif  // CAN_INTERFACE_H
