#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include "mqtt/async_client.h"

using namespace std;

// for publish data with MQTT
//const string DFLT_SERVER_ADDRESS	{ "tcp://192.168.1.127:1883" };
const string DFLT_SERVER_ADDRESS { "tcp://localhost" };
const string TOPIC { "odom" };
const int  QOS = 1;
const auto TIMEOUT = std::chrono::seconds(10);
mqtt::async_client client(DFLT_SERVER_ADDRESS, "");
mqtt::topic top(client, TOPIC, QOS);
mqtt::token_ptr tok;

// for listen ROS Data
void listenRosDataCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    //char* buffer;
    //sprintf(buffer, "linear.x=%f\r\nlinear.y=%f\r\nlinear.z=%f\r\nangular.x=%f\r\nangular.y=%f\r\nangular.z=%f\r\n\r\n", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
    //sprintf(buffer, "linear.x=%g\r\n", msg->linear.x);
    string buffer = "linear.x = ";
    buffer.append(to_string(msg->linear.x));
    buffer.append("\tlinear.y = ");
    buffer.append(to_string(msg->linear.y));
    buffer.append("\tlinear.z = ");
    buffer.append(to_string(msg->linear.z));
    buffer.append("\r\nangular.x = ");
    buffer.append(to_string(msg->angular.x));
    buffer.append("\tangular.y = ");
    buffer.append(to_string(msg->angular.y));
    buffer.append("\tangular.z = ");
    buffer.append(to_string(msg->angular.z));
    buffer.append("\r\n");
    tok = top.publish(buffer); 
    //ROS_INFO("received msg: [%f]", msg->linear.x);
    //ROS_INFO("received msg: [%f]", msg->linear.y);
    //ROS_INFO("received msg: [%f]", msg->linear.z);
    //ROS_INFO("received msg: [%f]", msg->angular.x);
    //ROS_INFO("received msg: [%f]", msg->angular.y);
    //ROS_INFO("received msg: [%f]", msg->angular.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros2mqtt_pub");
  
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, listenRosDataCallback);
  
    client.connect()->wait();
    ros::spin();

    tok->wait();
    client.disconnect()->wait();

  
    return 0;
}
