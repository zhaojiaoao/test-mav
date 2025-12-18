#include "mavlink_udp.h"
#include "mavlink.h"
#include <iostream>
#include <chrono>

MavlinkUdp::MavlinkUdp(asio::io_context& io_context, const Config& config)
    : io_context_(io_context)
    , socket_(io_context)
    , config_(config)
    , recv_buffer_(config.buffer_size)
{
    // 解析远程端点
    std::error_code ec;
    auto remote_address = asio::ip::address::from_string(config.remote_address, ec);
    if (ec) {
        throw std::runtime_error("Invalid remote address: " + config.remote_address);
    }
    remote_endpoint_ = asio::ip::udp::endpoint(remote_address, config.remote_port);
}

MavlinkUdp::~MavlinkUdp() {
    stop();
}

bool MavlinkUdp::start() {
    if (running_) {
        return true;
    }
    
    try {
        // 解析本地地址
        auto local_address = asio::ip::address::from_string(config_.local_address);
        auto local_endpoint = asio::ip::udp::endpoint(local_address, config_.local_port);
        
        // 打开 socket
        socket_.open(local_endpoint.protocol());
        
        // 设置 socket 选项
        socket_.set_option(asio::ip::udp::socket::reuse_address(true));
        socket_.set_option(asio::socket_base::broadcast(true));
        socket_.set_option(asio::socket_base::receive_buffer_size(65536));
        socket_.set_option(asio::socket_base::send_buffer_size(65536));
        
        // 绑定到本地端点
        socket_.bind(local_endpoint);
        
        // 开始接收
        start_receive();
        running_ = true;
        
        std::cout << "MAVLink UDP started on " << config_.local_address 
                  << ":" << config_.local_port << std::endl;
        std::cout << "Remote endpoint: " << config_.remote_address 
                  << ":" << config_.remote_port << std::endl;
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to start MAVLink UDP: " << e.what() << std::endl;
        return false;
    }
}

void MavlinkUdp::stop() {
    if (!running_) return;
    
    running_ = false;
    
    try {
        socket_.close();
    } catch (...) {
        // 忽略关闭异常
    }
    
    std::cout << "MAVLink UDP stopped" << std::endl;
}

void MavlinkUdp::start_receive() {
    if (!running_) return;
    
    socket_.async_receive_from(
        asio::buffer(recv_buffer_),
        sender_endpoint_,
        [this](const std::error_code& ec, size_t bytes_transferred) {
            handle_receive(ec, bytes_transferred);
        }
    );
}

void MavlinkUdp::handle_receive(const std::error_code& ec, size_t bytes_transferred) {
    if (ec) {
        if (ec != asio::error::operation_aborted) {
            if (error_callback_) {
                error_callback_(ec);
            }
            std::cerr << "Receive error: " << ec.message() << std::endl;
        }
        return;
    }
    
    // 更新统计信息
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.packets_received++;
        stats_.bytes_received += bytes_transferred;
    }
    
    // 调用原始数据包回调
    if (packet_callback_) {
        packet_callback_(recv_buffer_.data(), bytes_transferred);
    }
    
    // 解析 MAVLink 消息
    parse_mavlink(recv_buffer_.data(), bytes_transferred);

    // std::cout << "Received " << bytes_transferred << " bytes from "  << sender_endpoint_.address().to_string() << ":"   << sender_endpoint_.port() << std::endl;
    
    // 继续接收
    start_receive();
}

void MavlinkUdp::parse_mavlink(const uint8_t* data, size_t length) {
    static mavlink_status_t status;
    mavlink_message_t msg;
    
    for (size_t i = 0; i < length; i++) {
        uint8_t result = mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status);
        
        if (result == MAVLINK_FRAMING_OK) {
            // 成功解析到一个完整的 MAVLink 消息
            if (message_callback_) {
                message_callback_(msg);
            }
            
            // 打印消息信息
            std::cout << "Received MAVLink message:" << std::endl;
            std::cout << "  System ID: " << static_cast<int>(msg.sysid) << std::endl;
            std::cout << "  Component ID: " << static_cast<int>(msg.compid) << std::endl;
            std::cout << "  Message ID: " << msg.msgid << std::endl;
            std::cout << "  Sequence: " << static_cast<int>(msg.seq) << std::endl;
            std::cout << "  Length: " << static_cast<int>(msg.len) << std::endl;
            
            // 如果是 HEARTBEAT 消息，解析具体内容
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                mavlink_heartbeat_t heartbeat;
                mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                
                std::cout << "  HEARTBEAT details:" << std::endl;
                std::cout << "    Type: " << static_cast<int>(heartbeat.type) << std::endl;
                std::cout << "    Autopilot: " << static_cast<int>(heartbeat.autopilot) << std::endl;
                std::cout << "    Base mode: 0x" << std::hex << static_cast<int>(heartbeat.base_mode) << std::dec << std::endl;
                std::cout << "    Custom mode: " << heartbeat.custom_mode << std::endl;
                std::cout << "    System status: " << static_cast<int>(heartbeat.system_status) << std::endl;
            }
        } else if (result == MAVLINK_FRAMING_BAD_CRC) {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.parse_errors++;
        }
    }
}

void MavlinkUdp::send_raw(const uint8_t* data, size_t length) {
    if (!running_) {
        std::cerr << "send_raw: not running, drop packet of length " << length << std::endl;
        return;
    }
    if (length == 0) {
        std::cerr << "send_raw: called with length==0, nothing to send" << std::endl;
        return;
    }
    
    // 将数据复制到发送队列
    std::vector<uint8_t> packet(data, data + length);
    
    {
        std::lock_guard<std::mutex> lock(send_queue_mutex_);
        send_queue_.push(std::move(packet));
    }
    
    // 开始发送（如果还没有在发送）
    io_context_.post([this]() {
        start_send();
    });
}

void MavlinkUdp::send_message(const mavlink_message_t& msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    std::cout << "send_message: mavlink_msg_to_send_buffer returned len=" << len
              << ", msg.len=" << static_cast<int>(msg.len) << std::endl;

    send_raw(buffer, len);
    
    // 更新统计信息
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.packets_sent++;
    stats_.bytes_sent += len;
}

void MavlinkUdp::start_send() {
    std::lock_guard<std::mutex> lock(send_queue_mutex_);
    
    if (send_queue_.empty()) {
        return;
    }
    
    auto packet = std::move(send_queue_.front());
    send_queue_.pop();

    // Keep packet data alive using shared_ptr so the buffer remains valid
    // for the duration of the async send, independent of capture/move order.
    auto packet_ptr = std::make_shared<std::vector<uint8_t>>(std::move(packet));

    socket_.async_send_to(
        asio::buffer(*packet_ptr),
        remote_endpoint_,
        [this, packet_ptr](const std::error_code& ec, size_t bytes_transferred) {
            handle_send(ec, bytes_transferred);
        }
    );
}

void MavlinkUdp::handle_send(const std::error_code& ec, size_t bytes_transferred) {
    if (ec) {
        if (error_callback_) {
            error_callback_(ec);
        }
        std::cerr << "Send error: " << ec.message() << std::endl;
        return;
    }
    std::cout << "Sent " << bytes_transferred << " bytes to "  << remote_endpoint_.address().to_string() << ":"  << remote_endpoint_.port() << std::endl;
    // 继续发送队列中的下一个数据包
    start_send();
}

MavlinkUdp::Statistics MavlinkUdp::get_statistics() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}