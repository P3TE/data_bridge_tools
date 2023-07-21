#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include <sys/stat.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <std_msgs/msg/byte_multi_array.hpp>

#include "serial_termios_helper_functions.hpp"

#define VERSION "0.0.3"

inline bool file_exists(const std::string& name) {
    struct stat buffer;   
    return (stat (name.c_str(), &buffer) == 0); 
}

class RosToSerialBridge : public rclcpp::Node
{

private:

    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr serialSendDataSubscriber;
    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr serialReceiveDataPublisher;

    std::thread serialPortReadThread;

    const std::string _SerialPortNameParameterId = "serial_port_name";
    std::string serialPortName = "/dev/ttyUSB0";
    const std::string _BaudRateParameterId = "baud_rate";
    int humanReadableBaud = 9600;

    int serialDevice = -1;

    void SerialSendDataTopicCallback(const std_msgs::msg::ByteMultiArray::SharedPtr serialSendDataMsg) {
        RCLCPP_DEBUG(get_logger(), "Writing %d bytes to the serial port", serialSendDataMsg->data.size());
        write(serialDevice, (char*) (serialSendDataMsg->data.data()), serialSendDataMsg->data.size());
    }

    /**
     * Pack the raw bytes into a std_msgs::msg::ByteMultiArray and publish it.
    */
    void PublishDataFromSerial(uint8_t* data, const int dataLength){
        
        std_msgs::msg::ByteMultiArray byteMultiArray;
        byteMultiArray.data.resize(dataLength);
        memcpy(&byteMultiArray.data[0], data, dataLength);
        byteMultiArray.layout.data_offset = 0;
        std_msgs::msg::MultiArrayDimension dim;
        dim.size = dataLength;
        dim.stride = 1;
        dim.label = "data";
        byteMultiArray.layout.dim.push_back(dim);
        serialReceiveDataPublisher->publish(byteMultiArray);

    }

    void SerialReadThread() {
        uint8_t buffer[512];

        while (rclcpp::ok()) {
            int bytesRead = read(serialDevice, buffer, 512);
            if (bytesRead <= 0) {

                // 0 Bytes either means we've not received anything for ~0.5 seconds.
                // Or that the serial port is closed.
                            
                if (!file_exists(serialPortName)) {
                    RCLCPP_FATAL(get_logger(), "Serial Port %s Closed.", serialPortName.c_str());
                    break;
                }

                continue;
            }

            RCLCPP_DEBUG(get_logger(), "Read %i bytes from the serial port", bytesRead);
            
            PublishDataFromSerial(buffer, bytesRead);
        }

        rclcpp::shutdown(nullptr, "Serial port closed.");
    }

public:
    RosToSerialBridge() : Node("ros_to_serial_bridge") {

        RCLCPP_INFO(get_logger(), "Starting %s version %s", get_name(), VERSION);

        if (!has_parameter(_SerialPortNameParameterId)) declare_parameter(_SerialPortNameParameterId);
        bool serialPortNameRetrieved = get_parameter(_SerialPortNameParameterId, serialPortName);
        RCLCPP_INFO(get_logger(), "%s %s for '%s'.", serialPortNameRetrieved ? "Set" : "Defaulted", serialPortName.c_str(), _SerialPortNameParameterId.c_str());

        if (!has_parameter(_BaudRateParameterId)) declare_parameter(_BaudRateParameterId);
        bool baudRateRetrieved = get_parameter(_BaudRateParameterId, humanReadableBaud);
        RCLCPP_INFO(get_logger(), "%s %d for '%s'.", baudRateRetrieved ? "Set" : "Defaulted", humanReadableBaud, _BaudRateParameterId.c_str());
        BaudRate baud = BaudRateIntToSpeed(humanReadableBaud);
        if (!baud.valid) {
            RCLCPP_FATAL(get_logger(), "Unsupported baud rate: %i", baud.humanReadableBaud);
        }

        RCLCPP_INFO(get_logger(), "Opening serial port %s @ %i Baud", serialPortName.c_str(), humanReadableBaud);

        if (!file_exists(serialPortName)) {
            RCLCPP_FATAL(get_logger(), "Serial Port %s does not exist!", serialPortName.c_str());
            rclcpp::shutdown(nullptr, "Failed to open serial port.");
            return;
        }

        if ( ( serialDevice = open( serialPortName.c_str(), O_RDWR | O_NOCTTY ) ) < 0 ) {
            RCLCPP_FATAL(get_logger(), "Failed to open serial port: %s! Check your file permissions. A common fix is to add your user to the dialout group: 'sudo usermod -aG dialout $USER', then restart your computer.", serialPortName.c_str() );
            rclcpp::shutdown(nullptr, "Failed to open serial port.");
            return;
        }

        int setRawModeErrorCode = SetSerialDeviceToRawMode(serialDevice, baud);
        if (setRawModeErrorCode != 0 ) {
            switch (setRawModeErrorCode)
            {
            case TERMIOS_ERROR_INVALID_BAUD_RATE:
                RCLCPP_FATAL(get_logger(), "Unsupported baud rate: %i", humanReadableBaud);
                break;
            case TERMIOS_ERROR_FAILED_TO_SET_DEVICE_OPTION:
                RCLCPP_FATAL(get_logger(), "Failed to set device options for 'Raw Mode'!");
                break;
            default:
                RCLCPP_FATAL(get_logger(), "Failed to set serial in raw mode with error code %i", setRawModeErrorCode);
                break;
            }
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(get_logger(), "Serial port successfully configured in raw mode for port %s @ BAUD %i", serialPortName.c_str(), humanReadableBaud);

        rclcpp::QoS qosProfile(30);
        qosProfile.reliable();
        qosProfile.transient_local();

        serialReceiveDataPublisher = this->create_publisher<std_msgs::msg::ByteMultiArray>("serial/bytes/receive", qosProfile);
        serialSendDataSubscriber = create_subscription<std_msgs::msg::ByteMultiArray>(
            "serial/bytes/send", qosProfile, std::bind(&RosToSerialBridge::SerialSendDataTopicCallback, this, std::placeholders::_1));

        serialPortReadThread = std::thread{std::bind(&RosToSerialBridge::SerialReadThread, this)};
    }

    ~RosToSerialBridge() {
        close(serialDevice);
        serialPortReadThread.join();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosToSerialBridge>());
    rclcpp::shutdown();
    return 0;
}