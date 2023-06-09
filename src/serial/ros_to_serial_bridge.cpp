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
#include <std_msgs/msg/byte_multi_array.hpp>

#define VERSION "0.0.2"

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
    int baudRate = 9600;

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

    /**
     * Initialise the serial port in raw mode (no modification of the data)
     */
    int SetSerialDeviceParams(int fd, int baud) {
        struct termios options;

        fcntl( fd, F_SETFL, 0 );
        tcgetattr( fd, &options );

        // https://man7.org/linux/man-pages/man3/termios.3.html
        // Raw mode
        // cfmakeraw() sets the terminal to something like the "raw" mode of
        // the old Version 7 terminal driver: input is available character
        // by character, echoing is disabled, and all special processing of
        // terminal input and output characters is disabled.  The terminal
        // attributes are set as follows:

        options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                            | INLCR | IGNCR | ICRNL | IXON);
        options.c_oflag &= ~OPOST;
        options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        options.c_cflag &= ~(CSIZE | PARENB);
        options.c_cflag |= CS8;

        // http://unixwiz.net/techtips/termios-vmin-vtime.html
        // Return from read() after any number of bytes (VMIN = 0)
        // Wait for up to 5 tenths of a second without returning anything (VTIME)
        // This does mean after 5 tenths of a second with nothing, it will return from read with 0 bytes
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 5;

        switch ( baud ) {
            case 9600:
                cfsetispeed( &options, B9600 );
                cfsetospeed( &options, B9600 );
                break;
            case 19200:
                cfsetispeed( &options, B19200 );
                cfsetospeed( &options, B19200 );
                break;
            case 38400:
                cfsetispeed( &options, B38400 );
                cfsetospeed( &options, B38400 );
                break;
            case 57600:
                cfsetispeed( &options, B57600 );
                cfsetospeed( &options, B57600 );
                break;
            case 115200:
                cfsetispeed( &options, B115200 );
                cfsetospeed( &options, B115200 );
                break;
            default:
                RCLCPP_FATAL(get_logger(), "INVALID BAUD RATE %d", baud);
                return -1;
        }

        RCLCPP_INFO(get_logger(), "Serial baud rate set to: %d", baud);

        if ( tcsetattr( fd, TCSANOW, &options ) < 0 ) {
            RCLCPP_FATAL(get_logger(), "FAILED TO SET SERIAL DEVICE OPTIONS");
            close( fd );
            return 1;
        }

        RCLCPP_INFO(get_logger(), "Successfully set serial device options");

        // Flush the serial port input and output buffers
        tcflush( fd, TCIOFLUSH );
        return 0;
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

        serialSendDataSubscriber = create_subscription<std_msgs::msg::ByteMultiArray>(
            "serial/bytes/send", 10, std::bind(&RosToSerialBridge::SerialSendDataTopicCallback, this, std::placeholders::_1));
        serialReceiveDataPublisher = this->create_publisher<std_msgs::msg::ByteMultiArray>("serial/bytes/receive", 10);

        if (!has_parameter(_SerialPortNameParameterId)) declare_parameter(_SerialPortNameParameterId);
        bool serialPortNameRetrieved = get_parameter(_SerialPortNameParameterId, serialPortName);
        RCLCPP_INFO(get_logger(), "%s %s for '%s'.", serialPortNameRetrieved ? "Set" : "Defaulted", serialPortName.c_str(), _SerialPortNameParameterId.c_str());

        if (!has_parameter(_BaudRateParameterId)) declare_parameter(_BaudRateParameterId);
        bool baudRateRetrieved = get_parameter(_BaudRateParameterId, baudRate);
        RCLCPP_INFO(get_logger(), "%s %d for '%s'.", baudRateRetrieved ? "Set" : "Defaulted", baudRate, _BaudRateParameterId.c_str());

        RCLCPP_INFO(get_logger(), "Opening serial port %s @ %i Baud", serialPortName.c_str(), baudRate);

        if (!file_exists(serialPortName)) {
            RCLCPP_FATAL(get_logger(), "Serial Port %s does not exist!", serialPortName.c_str());
            rclcpp::shutdown(nullptr, "Failed to open serial port.");
            return;
        }

        if ( ( serialDevice = open( serialPortName.c_str(), O_RDWR | O_NOCTTY ) ) < 0 ) {
            RCLCPP_FATAL(get_logger(), "Failed to open serial port: %s! Check your file permissions. A common fix is 'sudo usermod -aG dialout $USER'", serialPortName.c_str() );
            rclcpp::shutdown(nullptr, "Failed to open serial port.");
            return;
        }

        if (SetSerialDeviceParams(serialDevice, baudRate) != 0 ) {
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(get_logger(), "Serial Port Successfully configured");

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