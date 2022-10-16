#include <unistd.h>
#include <math.h>
#include <map>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/executor.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "RTProtocol.h"
#include "RTPacket.h"

using namespace std::chrono_literals;

class QualisysNode : public rclcpp::Node
{

private:
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pub_pose;
    std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> pub_odom;
    std::map<std::string, rclcpp::Time> pub_stamps;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::string server = "192.168.1.2"; // mocap server ip
    double rate_limit = 100;  // hz
    std::string parent_frame = "map";
    int slow_count = 0;

    CRTProtocol rtProtocol;

    const unsigned short basePort = 22222;
    const int majorVersion = 1;
    const int minorVersion = 19;
    const bool bigEndian = false;

    bool dataAvailable = false;
    bool streamFrames = false;
    unsigned short udpPort = 6734;

    const int queue_size = 1;
    
public:
    QualisysNode()
    : Node ("qualisys_node")
    {       
        RCLCPP_INFO(this->get_logger(), "qualisys node is created");
        auto interval = std::chrono::duration<double>(1/rate_limit);
        RCLCPP_INFO(this->get_logger(), "Interval set at: %g", interval.count());
        // tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

        timer_ = this->create_wall_timer(interval, std::bind(&QualisysNode::run, this));
    }

    // Setting rate_limit and server ip address
    QualisysNode(double rate_limit, std::string server_str)
    : Node("qualisys_node")
    {
        server = server_str;
        RCLCPP_INFO(this->get_logger(), "qualisys node is created");
        auto interval = std::chrono::duration<double>(1/rate_limit);
        RCLCPP_INFO(this->get_logger(), "Interval set at: %g Hz | %g seconds", rate_limit, interval.count());
        // tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

        timer_ = this->create_wall_timer(interval, std::bind(&QualisysNode::run, this));
    }

    ~QualisysNode()
    {
        RCLCPP_INFO(this->get_logger(), "qualisys node is shutting down");
        rtProtocol.StreamFramesStop();
        rtProtocol.Disconnect();
    }

    void run()
    {
        try
        {
            if (!rtProtocol.Connected())
            {                
                if (!rtProtocol.Connect(server.c_str(), basePort, &udpPort, majorVersion, minorVersion, bigEndian))
                {
                    RCLCPP_WARN(this->get_logger(),"rtProtocol.Connect: %s\n\n", rtProtocol.GetErrorString());
                    sleep(1);
                    // continue;
                }
            }

            if (!dataAvailable)
            {
                if (!rtProtocol.Read6DOFSettings(dataAvailable))
                {
                    RCLCPP_WARN(this->get_logger(),"rtProtocol.Read6DOFSettings: %s\n\n", rtProtocol.GetErrorString());
                    sleep(1);
                    // continue;
                }
            }

            if (!streamFrames)
            {
                if (!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort, NULL, CRTProtocol::cComponent6d))
                {
                    RCLCPP_WARN(this->get_logger(),"rtProtocol.StreamFrames: %s\n\n", rtProtocol.GetErrorString());
                    sleep(1);
                    // continue;
                }
                streamFrames = true;

                RCLCPP_INFO(this->get_logger(),"Starting to streaming 6DOF data");
            }

            CRTPacket::EPacketType packetType;

            if (rtProtocol.ReceiveRTPacket(packetType, true) > 0)
            {
                if (packetType == CRTPacket::PacketData)
                {
                    // auto now = rclcpp::Clock::now;
                    rclcpp::Time now = this->get_clock()->now();

                    float fX, fY, fZ;
                    float rotationMatrix[9];

                    CRTPacket *rtPacket = rtProtocol.GetRTPacket();

                    //RCLCPP_WARN(this->get_logger(),"Frame %d\n", rtPacket->GetFrameNumber());

                    for (unsigned int i = 0; i < rtPacket->Get6DOFBodyCount(); i++)
                    {

                        if (rtPacket->Get6DOFBody(i, fX, fY, fZ, rotationMatrix))
                        {
                            std::string name(rtProtocol.Get6DOFBodyName(i));
                            //RCLCPP_WARN(this->get_logger(),"data received for rigid body %s", name.c_str());

                            if (!isfinite(fX) || !isfinite(fY) || !isfinite(fZ)) {
                                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "rigid body %s tracking lost", name.c_str());
                                continue;
                            }

                            for (int i=0; i<9; i++) {
                                if (!isfinite(rotationMatrix[i])) {
                                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "rigid body %s tracking lost", name.c_str());
                                    continue;
                                }
                            }

                            // convert to quaternion
                            tf2::Matrix3x3 R(
                              rotationMatrix[0], rotationMatrix[3], rotationMatrix[6],
                              rotationMatrix[1], rotationMatrix[4], rotationMatrix[7],
                              rotationMatrix[2], rotationMatrix[5], rotationMatrix[8]);
                            tf2::Quaternion q;
                            R.getRotation(q);

                            // scale position to meters from mm
                            double x = fX/1.0e3;
                            double y = fY/1.0e3;
                            double z = fZ/1.0e3;
                            double elapsed = 0;

                            // publish data if rate limit met
                            if (pub_stamps.count(name) == 0) {
                                elapsed = 0;
                            } else {
                              elapsed = (now - pub_stamps[name]).seconds();
                              if (elapsed < 0.99/rate_limit) {
                                // wait
                                continue;
                              }
                            }
                            pub_stamps[name] = now;

                            // warning if slow
                            if (elapsed > 3.0/rate_limit) {
                                slow_count += 1;
                                if (slow_count > 10) {
                                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,"publication rate low: %10.4f Hz", 1.0/elapsed);
                                    slow_count = 0;
                                }
                            }
                            // RCLCPP_INFO(this->get_logger(), "RB: %s x: %g y: %g z:%g", name.c_str(), x, y, z);

                            // publish transform
                            {
                                geometry_msgs::msg::TransformStamped transformStamped;
                                transformStamped.header.stamp = now;
                                transformStamped.header.frame_id = parent_frame;
                                transformStamped.child_frame_id = name;
                                transformStamped.transform.translation.x = x;
                                transformStamped.transform.translation.y = y;
                                transformStamped.transform.translation.z = z;
                                transformStamped.transform.rotation.x = q.x();
                                transformStamped.transform.rotation.y = q.y();
                                transformStamped.transform.rotation.z = q.z();
                                transformStamped.transform.rotation.w = q.w();
                                // br->sendTransform(transformStamped);
                            }

                            // publish pose stamped message
                            {
                                if (pub_pose.find(name) == pub_pose.end()) {
                                    RCLCPP_INFO(this->get_logger(), "rigid body %s pose added", name.c_str());
                                    pub_pose[name] = this->create_publisher<geometry_msgs::msg::PoseStamped>(name + "/pose", queue_size);
                                }
                                geometry_msgs::msg::PoseStamped msg;
                                msg.header.frame_id= parent_frame;
                                msg.header.stamp = now;
                                msg.pose.position.x = x;
                                msg.pose.position.y = y;
                                msg.pose.position.z = z;
                                msg.pose.orientation.x = q.x();
                                msg.pose.orientation.y = q.y();
                                msg.pose.orientation.z = q.z();
                                msg.pose.orientation.w = q.w();
                                pub_pose[name]->publish(msg);
                            }
                        }
                    }
                }
            }
        }
    
        catch (std::exception &e)
        {
            printf("%s\n", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    double rate_limit;
    std::string server;

    for (int i = 0; i < argc; i++)
    {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", argv[i]);
        if (std::string(argv[i]) == std::string("--rate"))
        {
            rate_limit = std::atof(argv[i+1]);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "rate set at %g", rate_limit);
            i++;
        }
        else if (std::string(argv[i]) == "--server")
        {
            server = std::string(argv[i+1]);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "server ip set at: %s", server.c_str());
            i++;
        }
        
    }

    auto qualisys_node = std::make_shared<QualisysNode>(rate_limit, server);
    rclcpp::spin(qualisys_node);
    return 0;
}   
