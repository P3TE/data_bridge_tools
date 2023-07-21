#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include <sys/stat.h>

#include <ros/ros.h>

#include "std_msgs/ByteMultiArray.h"

#define VERSION "0.0.2"

inline bool file_exists(const std::string& name) {
    struct stat buffer;   
    return (stat (name.c_str(), &buffer) == 0); 
}

class RosToSerialBridge
{

private:

    ros::Subscriber serialSendDataSubscriber;
    ros::Publisher serialReceiveDataPublisher;

    std::thread serialPortReadThread;

    const std::string _SerialPortNameParameterId = "serial_port_name";
    std::string serialPortName = "/dev/ttyUSB0";
    const std::string _BaudRateParameterId = "baud_rate";
    int baudRate = 9600;

    int serialDevice = -1;

    void SerialSendDataTopicCallback(const std_msgs::ByteMultiArray::ConstPtr& serialSendDataMsg) {
        ROS_DEBUG("Writing %i bytes to the serial port", (int) serialSendDataMsg->data.size());
        write(serialDevice, (char*) (serialSendDataMsg->data.data()), serialSendDataMsg->data.size());
    }

    /**
     * Pack the raw bytes into a std_msgs::msg::ByteMultiArray and publish it.
    */
    void PublishDataFromSerial(uint8_t* data, const int dataLength){
        
        std_msgs::ByteMultiArray byteMultiArray;
        byteMultiArray.data.resize(dataLength);
        memcpy(&byteMultiArray.data[0], data, dataLength);
        byteMultiArray.layout.data_offset = 0;
        std_msgs::MultiArrayDimension dim;
        dim.size = dataLength;
        dim.stride = 1;
        dim.label = "data";
        byteMultiArray.layout.dim.push_back(dim);
        serialReceiveDataPublisher.publish(byteMultiArray);

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
                ROS_FATAL("INVALID BAUD RATE %d", baud);
                return -1;
        }

        ROS_INFO("Serial baud rate set to: %d", baud);

        if ( tcsetattr( fd, TCSANOW, &options ) < 0 ) {
            ROS_FATAL("FAILED TO SET SERIAL DEVICE OPTIONS");
            close( fd );
            return 1;
        }

        ROS_INFO("Successfully set serial device options");

        // Flush the serial port input and output buffers
        tcflush( fd, TCIOFLUSH );
        return 0;
    }

    void SerialReadThread() {
        uint8_t buffer[512];

        while (ros::ok()) {
            int bytesRead = read(serialDevice, buffer, 512);
            if (bytesRead <= 0) {

                // 0 Bytes either means we've not received anything for ~0.5 seconds.
                // Or that the serial port is closed.
                            
                if (!file_exists(serialPortName)) {
                    ROS_FATAL("Serial Port %s Closed.", serialPortName.c_str());
                    break;
                }

                continue;
            }

            ROS_DEBUG("Read %i bytes from the serial port", bytesRead);
            
            PublishDataFromSerial(buffer, bytesRead);
        }

        ros::shutdown();
    }

public:
    RosToSerialBridge(ros::NodeHandle& n) {

        bool serialPortNameRetreived = n.getParam(_SerialPortNameParameterId, serialPortName);
        ROS_INFO("%s %s for '%s'.", serialPortNameRetreived ? "Set" : "Defaulted", serialPortName.c_str(), _SerialPortNameParameterId.c_str());

        bool baudRateRetreived = n.getParam(_BaudRateParameterId, baudRate);
        ROS_INFO("%s %i for '%s'.", baudRateRetreived ? "Set" : "Defaulted", baudRate, _BaudRateParameterId.c_str());

        ROS_INFO("Opening serial port %s @ %i Baud", serialPortName.c_str(), baudRate);

        if (!file_exists(serialPortName)) {
            ROS_FATAL("Serial Port %s does not exist!", serialPortName.c_str());
            ros::shutdown();
            return;
        }

        if ( ( serialDevice = open( serialPortName.c_str(), O_RDWR | O_NOCTTY ) ) < 0 ) {
            ROS_FATAL("Failed to open serial port: %s! Check your file permissions. A common fix is to add your user to the dialout group: 'sudo usermod -aG dialout $USER', then restart your computer.", serialPortName.c_str() );
            ros::shutdown();
            return;
        }

        if (SetSerialDeviceParams(serialDevice, baudRate) != 0 ) {
            ros::shutdown();
            return;
        }

        ROS_INFO("Serial Port Successfully configured");

        serialSendDataSubscriber = n.subscribe<std_msgs::ByteMultiArray>("serial/bytes/send", 10,
            std::bind(&RosToSerialBridge::SerialSendDataTopicCallback, this, std::placeholders::_1));
        serialReceiveDataPublisher = n.advertise<std_msgs::ByteMultiArray>("serial/bytes/receive", 10);

        serialPortReadThread = std::thread{std::bind(&RosToSerialBridge::SerialReadThread, this)};
    }

    ~RosToSerialBridge() {
        close(serialDevice);
        serialPortReadThread.join();
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "ros_to_serial_bridge");
    ros::NodeHandle n;

    ROS_INFO("Starting %s v%s", ros::this_node::getName().c_str(), VERSION);
    std::shared_ptr<RosToSerialBridge> rosToSerialBridge = std::make_shared<RosToSerialBridge>(n);
    ros::spin();

    ros::shutdown();
    return 0;
}