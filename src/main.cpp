#include "mavlink_udp.h"
#include "mavlink.h"
#include <asio.hpp>
#include <iostream>
#include <thread>
#include <atomic>
#include <csignal>
#include <functional>

std::atomic<bool> running{true};

void signal_handler(int) {
    running = false;
    std::cout << "\nShutting down..." << std::endl;
}

// 创建并发送 HEARTBEAT 消息
void send_heartbeat(MavlinkUdp& mavlink_udp, uint8_t sysid, uint8_t compid) {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;
    
    heartbeat.type = MAV_TYPE_QUADROTOR;
    heartbeat.autopilot = MAV_AUTOPILOT_PX4;
    heartbeat.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED |  MAV_MODE_FLAG_SAFETY_ARMED;
    heartbeat.custom_mode = 0;
    heartbeat.system_status = MAV_STATE_ACTIVE;
    
    mavlink_msg_heartbeat_encode(sysid, compid, &msg, &heartbeat);
    mavlink_udp.send_message(msg);
    
    std::cout << "Sent HEARTBEAT from sysid=" << static_cast<int>(sysid) 
              << ", compid=" << static_cast<int>(compid) << std::endl;
}

// 创建并发送 COMMAND_LONG 消息
void send_arm_command(MavlinkUdp& mavlink_udp, uint8_t target_sysid, uint8_t target_compid) {
    mavlink_message_t msg;
    
    mavlink_msg_command_long_pack(
        1,  // 发送者系统ID
        1,  // 发送者组件ID
        &msg,
        target_sysid,
        target_compid,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,    // 确认
        0, 0, 0, 0, 0, 0, 0  // 其他参数
    );
    
    mavlink_udp.send_message(msg);
    std::cout << "Sent ARM command to sysid=" << static_cast<int>(target_sysid) 
              << ", compid=" << static_cast<int>(target_compid) << std::endl;
}

int main(int argc, char* argv[]) {
    // 设置信号处理
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    try {
        // 创建 ASIO I/O 上下文
        asio::io_context io_context;
        
        // 配置 MAVLink UDP
        MavlinkUdp::Config config;
        config.local_address = "0.0.0.0";   // 监听所有接口
        config.local_port = 12345;          // QGroundControl 默认端口
        config.remote_address = "127.0.0.1"; // 发送到本地
        config.remote_port = 14550;          // 远程端口
        
        // 创建 MAVLink UDP 实例
        MavlinkUdp mavlink_udp(io_context, config);
        
        // 设置消息回调
        mavlink_udp.set_message_callback([](const mavlink_message_t& msg) {
            std::cout << "Callback: Received message ID=" << msg.msgid 
                      << " from sysid=" << static_cast<int>(msg.sysid) 
                      << ", compid=" << static_cast<int>(msg.compid) << std::endl;
        });
        
        // 设置错误回调
        mavlink_udp.set_error_callback([](const std::error_code& ec) {
            std::cerr << "Error callback: " << ec.message() << std::endl;
        });
        
        // 启动 MAVLink UDP
        if (!mavlink_udp.start()) {
            std::cerr << "Failed to start MAVLink UDP" << std::endl;
            return 1;
        }
        
        // 在后台线程运行 I/O 上下文
        std::thread io_thread([&io_context]() {
            asio::executor_work_guard<asio::io_context::executor_type> work_guard = 
                asio::make_work_guard(io_context);
            io_context.run();
        });
        
        std::cout << "MAVLink UDP Example" << std::endl;
        std::cout << "==================" << std::endl;
        std::cout << "Commands:" << std::endl;
        std::cout << "  h - Send HEARTBEAT" << std::endl;
        std::cout << "  a - Send ARM command" << std::endl;
        std::cout << "  s - Show statistics" << std::endl;
        std::cout << "  q - Quit" << std::endl;
        
        // 主循环
        while (running) {
            std::cout << "\n> ";
            std::string input;
            if (!std::getline(std::cin, input)) {
                break;
            }
            
            if (input.empty()) continue;
            
            switch (input[0]) {
                case 'h':
                case 'H':
                    send_heartbeat(mavlink_udp, 1, 1);
                    break;
                    
                case 'a':
                case 'A':
                    send_arm_command(mavlink_udp, 1, 1);
                    break;
                    
                case 's':
                case 'S': {
                    auto stats = mavlink_udp.get_statistics();
                    std::cout << "\nStatistics:" << std::endl;
                    std::cout << "  Packets received: " << stats.packets_received << std::endl;
                    std::cout << "  Packets sent: " << stats.packets_sent << std::endl;
                    std::cout << "  Bytes received: " << stats.bytes_received << std::endl;
                    std::cout << "  Bytes sent: " << stats.bytes_sent << std::endl;
                    std::cout << "  Parse errors: " << stats.parse_errors << std::endl;
                    break;
                }
                    
                case 'q':
                case 'Q':
                    running = false;
                    break;
                    
                default:
                    std::cout << "Unknown command" << std::endl;
                    break;
            }
        }
        
        // 停止 MAVLink UDP
        mavlink_udp.stop();
        
        // 停止 I/O 上下文并等待线程结束
        io_context.stop();
        if (io_thread.joinable()) {
            io_thread.join();
        }
        
        std::cout << "Program terminated gracefully" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}