#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial_driver/serial_driver.hpp>
#include <vector>
#include <string>

using drivers::serial_driver::SerialDriver;
using drivers::serial_driver::SerialPortConfig;
using drivers::common::IoContext;

class ArduinoSerialNode : public rclcpp::Node {
public:
    ArduinoSerialNode()
        : Node("arduino_serial_node")
    {
        // 创建 IoContext
        io_context_ = std::make_shared<IoContext>(1);

        // 创建 SerialPortConfig
        auto device_config = std::make_shared<SerialPortConfig>(
            9600,
            drivers::serial_driver::FlowControl::NONE,
            drivers::serial_driver::Parity::NONE,
            drivers::serial_driver::StopBits::ONE
        );

        // 创建 SerialDriver
        serial_driver_ = std::make_unique<SerialDriver>(*io_context_);

        // 打开串口
        try {
            serial_driver_->init_port("/dev/ttyACM0", *device_config);
            serial_driver_->port()->open();
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial port: %s", ex.what());
            return;
        }

        // 创建定时器和发布者
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ArduinoSerialNode::timer_callback, this));

        publisher_ = this->create_publisher<std_msgs::msg::String>("arduino_data", 10);
    }

private:
    void timer_callback() {
        std::vector<uint8_t> buffer(256);
        size_t bytes_read = 0;

        try {
            bytes_read = serial_driver_->port()->receive(buffer);
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", ex.what());
            return;
        }

        if (bytes_read > 0) {
            std::string data(buffer.begin(), buffer.begin() + bytes_read);
            process_and_publish_data(data);
        }
    }

    void process_and_publish_data(const std::string& data) {
        static std::string buffer;
        buffer += data;

        size_t pos;
        while ((pos = buffer.find('\n')) != std::string::npos) {
            std::string line = buffer.substr(0, pos);
            buffer.erase(0, pos + 1);

            if (line.find("Arduino ready for communication!") != std::string::npos ||
                line.find("Hello from Arduino!") != std::string::npos ||
                line.find("Arduino uptime:") != std::string::npos ||
                line.find("Blinking LED") != std::string::npos ||
                line.find("Blinking complete!") != std::string::npos ||
                line.find("Invalid choice") != std::string::npos) {
                
                auto message = std_msgs::msg::String();
                message.data = line;
                publisher_->publish(message);
                RCLCPP_INFO(this->get_logger(), "Published data: %s", line.c_str());
            }
        }
    }

    std::shared_ptr<IoContext> io_context_;
    std::unique_ptr<SerialDriver> serial_driver_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoSerialNode>());
    rclcpp::shutdown();
    return 0;
}
