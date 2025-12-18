#pragma once

#include <asio.hpp>
#include <string>
#include <functional>
#include <memory>
#include <vector>
#include <cstdint>
#include <atomic>
#include <queue>
#include <mutex>

// 前向声明 MAVLink 类型
struct __mavlink_message;
typedef __mavlink_message mavlink_message_t;

class MavlinkUdp {
public:
    // 回调函数类型定义
    using PacketCallback = std::function<void(const uint8_t* data, size_t length)>;
    using MessageCallback = std::function<void(const mavlink_message_t& msg)>;
    using ErrorCallback = std::function<void(const std::error_code& ec)>;

    // 配置结构体
    struct Config {
        std::string local_address {"0.0.0.0"};
        uint16_t local_port {14550};
        std::string remote_address  {"127.0.0.1"};
        uint16_t remote_port {14551};
        size_t buffer_size {4096};
    };

    // 构造函数和析构函数
    MavlinkUdp(asio::io_context& io_context, const Config& config = Config());
    ~MavlinkUdp();

    // 启动和停止
    bool start();
    void stop();

    // 发送原始数据
    void send_raw(const uint8_t* data, size_t length);
    
    // 发送 MAVLink 消息
    void send_message(const mavlink_message_t& msg);

    // 设置回调函数
    void set_packet_callback(PacketCallback callback) { packet_callback_ = std::move(callback); }
    void set_message_callback(MessageCallback callback) { message_callback_ = std::move(callback); }
    void set_error_callback(ErrorCallback callback) { error_callback_ = std::move(callback); }

    // 获取统计信息
    struct Statistics {
        size_t packets_received = 0;
        size_t packets_sent = 0;
        size_t bytes_received = 0;
        size_t bytes_sent = 0;
        size_t parse_errors = 0;
    };
    
    Statistics get_statistics() const;

private:
    // 异步接收
    void start_receive();
    void handle_receive(const std::error_code& ec, size_t bytes_transferred);
    
    // 异步发送
    void start_send();
    void handle_send(const std::error_code& ec, size_t bytes_transferred);

    // MAVLink 解析
    void parse_mavlink(const uint8_t* data, size_t length);

    // 成员变量
    asio::io_context& io_context_;
    asio::ip::udp::socket socket_;
    asio::ip::udp::endpoint remote_endpoint_;
    asio::ip::udp::endpoint sender_endpoint_;
    
    Config config_;
    std::vector<uint8_t> recv_buffer_;
    std::queue<std::vector<uint8_t>> send_queue_;
    std::mutex send_queue_mutex_;
    
    // 回调函数
    PacketCallback packet_callback_;
    MessageCallback message_callback_;
    ErrorCallback error_callback_;
    
    // 统计信息
    mutable std::mutex stats_mutex_;
    Statistics stats_;
    
    // 状态标志
    std::atomic<bool> running_{false};
};