# ROS 2 控制 Arduino 舵机

本文档描述了如何扩展现有的 ROS 2 与 Arduino 通信系统，以实现通过 ROS 2 控制连接到 Arduino 的舵机。

## 功能概述

- 通过 ROS 2 话题发送舵机角度命令
- Arduino 接收命令并控制连接到 9 号引脚的舵机
- ROS 2 节点接收并发布 Arduino 的反馈信息

## Arduino 代码更新

1. 添加 Servo 库支持并初始化舵机：

```cpp:src/main.cpp
#include <Arduino.h>
#include <Servo.h>

Servo myservo;

void printMenu() {
    // ... (保持原有的菜单选项)
    Serial.println("4. Set Servo Angle");
    Serial.println("Enter your choice:");
}

void setup() {
    // ... (保持原有的设置)
    myservo.attach(9); // 将舵机连接到9号引脚
    printMenu();
}

void loop() {
    if (Serial.available() > 0) {
        char choice = Serial.read();
        // Clear the serial buffer
        while (Serial.available() > 0) {
            Serial.read();
        }
        switch (choice) {
            // ... (保持原有的case语句)
            case '4':
                Serial.println("Enter servo angle (0-180):");
                while (!Serial.available()) {} // 等待输入
                int angle = Serial.parseInt();
                if (angle >= 0 && angle <= 180) {
                    myservo.write(angle);
                    Serial.print("Servo angle set to: ");
                    Serial.println(angle);
                } else {
                    Serial.println("Invalid angle. Please enter a value between 0 and 180.");
                }
                break;
            // ... (保持原有的default语句)
        }
        printMenu();
    }
}
```

## ROS 2 节点更新

1. 在 ROS 2 节点中添加一个新的订阅者，用于接收舵机角度命令：

```cpp:ros2_ws/src/arduino_communication/src/arduino_serial_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
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
        serial_driver_ = std::make_unique<SerialDriver>(io_context_);
        // 打开串口
        try {
            serial_driver_->init_port("/dev/ttyACM0", device_config);
            serial_driver_->port()->open();
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial port: %s", ex.what());
            return;
        }
        // 创建定时器和发布者
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ArduinoSerialNode::timer_callback, this)
        );
        publisher_ = this->create_publisher<std_msgs::msg::String>("arduino_data", 10);
        // 创建舵机角度订阅者
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "servo_angle", 10, std::bind(&ArduinoSerialNode::servo_callback, this, std::placeholders::_1)
        );
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
                line.find("Invalid choice") != std::string::npos ||
                line.find("Servo angle set to:") != std::string::npos) {
                auto message = std_msgs::msg::String();
                message.data = line;
                publisher_->publish(message);
                RCLCPP_INFO(this->get_logger(), "Published data: %s", line.c_str());
            }
        }
    }

    void servo_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        int angle = msg->data;
        if (angle >= 0 && angle <= 180) {
            std::string command = "4\n" + std::to_string(angle) + "\n";
            try {
                serial_driver_->port()->send(std::vector<uint8_t>(command.begin(), command.end()));
                RCLCPP_INFO(this->get_logger(), "Sent servo command: %d", angle);
            } catch (const std::exception &ex) {
                RCLCPP_ERROR(this->get_logger(), "Error sending servo command: %s", ex.what());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid servo angle: %d. Must be between 0 and 180.", angle);
        }
    }

    std::shared_ptr<IoContext> io_context_;
    std::unique_ptr<SerialDriver> serial_driver_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoSerialNode>());
    rclcpp::shutdown();
    return 0;
}
```

## 项目配置更新

1. 在 `platformio.ini` 文件中添加 Servo 库的依赖：

    ```ini
    [env:uno]
    platform = atmelavr
    board = uno
    framework = arduino
    test_framework = unity
    monitor_speed = 9600
    lib_deps = 
        arduino-libraries/Servo@^1.1.8
    ```

2. 更新 `CMakeLists.txt` 文件：

    ```cmake
    find_package(std_msgs REQUIRED)

    # ... (其他内容保持不变)

    ament_target_dependencies(arduino_serial_node
      rclcpp
      std_msgs
      serial_driver
    )
    ```

3. 更新 `package.xml` 文件：

    ```xml
    <depend>std_msgs</depend>
    ```

## 使用方法

1. 将舵机连接到 Arduino 的 9 号引脚。
2. 上传更新后的 Arduino 代码。
3. 编译并运行更新后的 ROS 2 节点。
4. 使用以下命令发送舵机角度命令：

    ```sh
    ros2 topic pub /servo_angle std_msgs/msg/Int32 "data: 90"
    ```

    将 90 替换为 0 到 180 之间的任何角度值。

5. 观察 Arduino 的串口输出或 ROS 2 的话题输出，以确认舵机角度设置是否成功。

通过这些更新，我们实现了一个更加灵活的 ROS 2 与 Arduino 通信系统，不仅可以读取 Arduino 的数据，还可以通过 ROS 2 控制 Arduino 上的舵机。这为进一步的机器人开发和控制提供了基础。
