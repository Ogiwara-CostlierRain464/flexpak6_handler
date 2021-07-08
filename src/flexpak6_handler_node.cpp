
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <sensor_msgs/NavSatFix.h>

#include <unistd.h>
#include <sstream>
#include <experimental/optional>

#include "../include/SerialPort.h"

struct Solution
{
    double latitude;
    double longitude;
    double height;
    Solution() : latitude(0.0), longitude(0.0), height(0.0){};
    Solution(double lat, double lon, double h) : latitude(lat), longitude(lon), height(h){};
};

class Flexpak6Handler
{
public:
    Flexpak6Handler(std::string path) : mLastConnected(false)
    {
        ros::NodeHandle nh("~");

        // Initialize ros objects
        mSerialSub = nh.subscribe("Serial_out", 1, &Flexpak6Handler::serial_callback, this);
        mTimer = nh.createTimer(ros::Duration(0.01), &Flexpak6Handler::timer_callback, this);
        mGNSSPub = nh.advertise<sensor_msgs::NavSatFix>("nmea_msg", 1);

        // Initialize diagnostics objects
        mUpdater.setHardwareID("SerialPort");
        mUpdater.add("Connect", boost::bind(&Flexpak6Handler::diagnostic0, this, _1));

        // Get and set device path of serial port
        mDevicePath = path;
        nh.getParam("device_name", mDevicePath);
        // Open serial port
        open_port();

        mLastConnected = mSerial.IsOpen();

        // Receiver Initialization command
        send_command("unlogall true");
        send_command("log gpgga ontime 0.5");
        //send_command("log gpgll ontime 1");
    }
    ~Flexpak6Handler()
    {
        send_command("unlogall true"); // Stop output from receiver
        mSerial.Close();
    }

    void mainLoop()
    {
        ros::spin();
    }

private:
    std::string mDevicePath;
    LibSerial::SerialPort mSerial;

    ros::Subscriber mSerialSub;
    ros::Publisher mGNSSPub;
    ros::Timer mTimer;
    diagnostic_updater::Updater mUpdater;

    std::string mBuffer;

    bool mLastConnected;

    void open_port()
    {
        try
        {
            mSerial.Open(mDevicePath.c_str());
            ROS_INFO("Opend serial port: %s", mDevicePath.c_str());
        }
        catch (LibSerial::OpenFailed &)
        {
            ROS_ERROR("Serial Fail: cound not open %s", mDevicePath.c_str());
            throw std::runtime_error("");
        }
        // Set the baud rates.
        mSerial.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
        // Set the number of data bits.
        mSerial.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        // Turn off hardware flow control.
        mSerial.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        // Disable parity.
        mSerial.SetParity(LibSerial::Parity::PARITY_NONE);
        // Set the number of stop bits.
        mSerial.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    }

    void send_command(const std::string c)
    {
        std::string command = c + '\r' + '\n';
        mSerial.Write(command);
        mSerial.DrainWriteBuffer();
    }

    void serial_callback(const std_msgs::String &serial_msg)
    {
        ROS_INFO("command : %s\n", serial_msg.data.c_str());
        send_command(serial_msg.data);
        ROS_INFO("sent.");
    }

    void diagnostic0(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        bool serial_c = mSerial.IsOpen();
        bool serial_s = true;
        if (serial_c && serial_s)
            stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Active.");
        else if (serial_c && !serial_s)
            stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "No Recieve.");
        else
            stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "No Connection.");
    }

    std::experimental::optional<sensor_msgs::NavSatFix> parseNMEA(std::string line)
    {
        std::stringstream ss(line);
        std::string seg;
        getline(ss, seg, ',');
        if (seg[0] != '$')
            return std::experimental::nullopt;
        if (seg != "$GPGGA")
            return std::experimental::nullopt;

        sensor_msgs::NavSatFix ret;
        ret.header.stamp = ros::Time::now();
        ret.header.frame_id = "gnss";

        getline(ss, seg, ','); // utc time
        getline(ss, seg, ','); // lat DDmm.mm
        if (seg.size())
            ret.latitude = stod(seg);
        getline(ss, seg, ','); // lat dir
        getline(ss, seg, ','); // lon DDmm.mm
        if (seg.size())
            ret.longitude = stod(seg);
        getline(ss, seg, ','); // lon dir
        getline(ss, seg, ','); // quality
        getline(ss, seg, ','); // sats
        getline(ss, seg, ','); // hdop
        getline(ss, seg, ','); // altitude
        if (seg.size())
            ret.altitude = stod(seg);

        while (getline(ss, seg, ','))
        {
            //ROS_INFO("  %s", seg.c_str());
        }

        return ret;
    }

    void timer_callback(const ros::TimerEvent &)
    {
        if (not mSerial.IsOpen())
        {
            // Retry to connect
            open_port();
        }
        else
        {
            std::string recv_data;
            try
            {
                mSerial.Read(recv_data, 0, 10);
                if (recv_data.size())
                {
                    mBuffer += recv_data;
                    ROS_INFO("str (%d): %s", int(mBuffer.size()), mBuffer.c_str());

                }
            }
            catch (const LibSerial::ReadTimeout &)
            {
                if (recv_data.size()) {
                    //ROS_INFO("truncate: %d : %s", int(recv_data.size()), recv_data.c_str());
                    mBuffer += recv_data;
                }
            }
            std::stringstream ssBuf(mBuffer);
            std::string line;
            int cnt = 0;
            while (true)
            {
                getline(ssBuf, line);
                if (ssBuf.eof() or ssBuf.fail())
                    break;
                cnt++;
            /*
                if (line.size() == 0)
                    continue;
                if (line[0] == '<')
                    continue; // Command Result
                if (line[0] == '[')
                    continue; // Port info
            */
                std::cout << line.c_str() << std::endl;
                auto ret = parseNMEA(line);
                if (ret)
                {
                    auto rv = ret.value();
                    ROS_INFO("parsed as : %f %f %f", rv.latitude, rv.longitude, rv.altitude);
                    mGNSSPub.publish(rv);
                }
            }
            mBuffer = line;
        }
        mLastConnected = mSerial.IsOpen();
        mUpdater.update();
        ros::spinOnce();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flexpak6_handler");

    std::string path;
    ros::NodeHandle nh("~");
    if (nh.getParam("path", path))
    {
        ROS_INFO("%s", path.c_str());
        Flexpak6Handler fh(path);
        fh.mainLoop();
    }
    else
    {
        throw std::runtime_error("Specify device path using rosparam");
    }

    return 0;
}