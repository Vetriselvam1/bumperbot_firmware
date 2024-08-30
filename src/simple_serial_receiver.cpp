#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>
#include <chrono>

using namespace std::chrono_literals;

class SimpleSerialReceiver : public rclcpp::Node
{
public:
    SimpleSerialReceiver() : Node("simple_serial_receiver")
    {
        // Declare and get parameter for port
        declare_parameter<std::string>("port", "/dev/ttyACM0");
        port_ = get_parameter("port").as_string();
        
        // Open the serial port
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

        // Create publisher
        pub_ = create_publisher<std_msgs::msg::String>("serial_receiver", 10);

        // Set timer to poll at 100ms (adjust as needed)
        timer_ = create_wall_timer(100ms, std::bind(&SimpleSerialReceiver::timerCallback, this));

        RCLCPP_INFO(get_logger(), "Receiving from serial port %s at baud rate 115200", port_.c_str());
    }

    ~SimpleSerialReceiver()
    {
        // Close the serial port
        arduino_.Close();
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    LibSerial::SerialPort arduino_;
    std::string port_;

    void timerCallback()
    {
        auto message = std_msgs::msg::String();
        
        if (arduino_.IsDataAvailable())
        {
            // Read line from the serial port
            arduino_.ReadLine(message.data);

            // Publish the message
            pub_->publish(message);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSerialReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
