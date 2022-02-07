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
#include "json/json.h"

using namespace std;

const string ROBOT_ID { "spu-robot-0" };

const string DFLT_SERVER_ADDRESS { "tcp://192.168.1.242:1883" };
const string CLIENT_ID { "spu-robot-0" };
const string TOPIC { "cmd_vel" };
const int QOS = 1;
const int N_RETRY_ATTEMPTS = 5;

mqtt::async_client client(DFLT_SERVER_ADDRESS, "");
mqtt::topic top(client, TOPIC, QOS);
mqtt::token_ptr tok;

string json_payload;
bool cmdReceivedFlag = false;

/////////////////////////////////////////////////////////////////////////////

// Callbacks for the success or failures of requested actions.
// This could be used to initiate further action, but here we just log the
// results to the console.

class action_listener : public virtual mqtt::iaction_listener
{
	string name_;

	void on_failure(const mqtt::token& tok) override {
		cout << name_ << " failure";
		if (tok.get_message_id() != 0)
			cout << " for token: [" << tok.get_message_id() << "]" << endl;
		cout << endl;
	}

	void on_success(const mqtt::token& tok) override {
		cout << name_ << " success";
		if (tok.get_message_id() != 0)
			cout << " for token: [" << tok.get_message_id() << "]" << endl;
		auto top = tok.get_topics();
		if (top && !top->empty())
			cout << "\ttoken topic: '" << (*top)[0] << "', ..." << endl;
		cout << endl;
	}

public:
	action_listener(const string& name) : name_(name) {}
};

/////////////////////////////////////////////////////////////////////////////

/**
 * Local callback & listener class for use with the client connection.
 * This is primarily intended to receive messages, but it will also monitor
 * the connection to the broker. If the connection is lost, it will attempt
 * to restore the connection and re-subscribe to the topic.
 */
class callback : public virtual mqtt::callback,
					public virtual mqtt::iaction_listener

{
	// Counter for the number of connection retries
    int nretry_;
	// The MQTT client
	mqtt::async_client& cli_;
	// Options to use if we need to reconnect
	mqtt::connect_options& connOpts_;
	// An action listener to display the result of actions.
	action_listener subListener_;

	// This deomonstrates manually reconnecting to the broker by calling
	// connect() again. This is a possibility for an application that keeps
	// a copy of it's original connect_options, or if the app wants to
	// reconnect with different options.
	// Another way this can be done manually, if using the same options, is
	// to just call the async_client::reconnect() method.
	void reconnect() {
		this_thread::sleep_for(chrono::milliseconds(2500));
		try {
			cli_.connect(connOpts_, nullptr, *this);
		}
		catch (const mqtt::exception& exc) {
			cerr << "Error: " << exc.what() << endl;
			exit(1);
		}
	}

	// Re-connection failure
	void on_failure(const mqtt::token& tok) override {
		cout << "Connection attempt failed" << endl;
		if (++nretry_ > N_RETRY_ATTEMPTS)
			exit(1);
		reconnect();
	}

	// (Re)connection success
	// Either this or connected() can be used for callbacks.
	void on_success(const mqtt::token& tok) override {}

	// (Re)connection success
	void connected(const string& cause) override {
		cout << "\nConnection success" << endl;
		cout << "\nSubscribing to topic '" << TOPIC << "'\n"
			<< "\tfor client " << CLIENT_ID
			<< " using QoS" << QOS << "\n"
			<< "\nPress Q<Enter> to quit\n" << endl;
		cli_.subscribe(TOPIC, QOS, nullptr, subListener_);
	}

	// Callback for when the connection is lost.
	// This will initiate the attempt to manually reconnect.
	void connection_lost(const string& cause) override {
		cout << "\nConnection lost" << endl;
		if (!cause.empty())
			cout << "\tcause: " << cause << endl;

		cout << "Reconnecting..." << endl;
		nretry_ = 0;
		reconnect();
	}

	// Callback for when a message arrives.
	void message_arrived(mqtt::const_message_ptr msg) override {
		cout << "Message arrived" << endl;
		cout << "\ttopic: '" << msg->get_topic() << "'" << endl;
		cout << "\tpayload: '" << msg->to_string() << "'\n" << endl;
        json_payload = msg->to_string();
        cmdReceivedFlag = true;
	}

	void delivery_complete(mqtt::delivery_token_ptr token) override {}

public:
	callback(mqtt::async_client& cli, mqtt::connect_options& connOpts)
				: nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription") {}
};

/////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros2mqtt_sub");

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Rate loop_rate(100);
    
    Json::Reader reader;
    Json::Value root;

    // A subscriber often wants the server to remember its messages when its
	// disconnected. In that case, it needs a unique ClientID and a
	// non-clean session.
    mqtt::async_client client(DFLT_SERVER_ADDRESS, CLIENT_ID);
    mqtt::connect_options connOpts;
    connOpts.set_clean_session(false);

    // Install the callback(s) before connecting.
    callback cb(client, connOpts);
    client.set_callback(cb);

    // Start the connection.
	// When completed, the callback will subscribe to topic.
    try {
		cout << "Connecting to the MQTT server..." << flush;
		client.connect(connOpts, nullptr, cb);
	}
	catch (const mqtt::exception& exc) {
		cerr << "\nERROR: Unable to connect to MQTT server: '"
			<< DFLT_SERVER_ADDRESS << "'" << exc << endl;
		return 1;
	}

    while (ros::ok()) {
        geometry_msgs::Twist mqtt_msg;
        
        if (cmdReceivedFlag) {
            cmdReceivedFlag = false;
            if (reader.parse(json_payload, root) == false) {
                cerr << "\nERROR: Unable to parse JSON stream" << endl;
                return 1;
            }
            switch (root[ROBOT_ID]["move"].asInt()) {
                case 0:
                    mqtt_msg.linear.x = 0;
                    mqtt_msg.linear.y = 0;
                    mqtt_msg.linear.z = 0;
                    mqtt_msg.angular.x = 0;
                    mqtt_msg.angular.y = 0;
                    mqtt_msg.angular.z = 0; break;
                case 1: mqtt_msg.linear.x = 0.3; break;    // go straight
                case 2: mqtt_msg.linear.x = -0.3; break;   // go back
                case 3: mqtt_msg.angular.z = 0.6; break;   // turn left
                case 4: mqtt_msg.angular.z = -0.6; break;  // turn right
                default: 
                    mqtt_msg.linear.x = 0;
                    mqtt_msg.linear.y = 0;
                    mqtt_msg.linear.z = 0;
                    mqtt_msg.angular.x = 0;
                    mqtt_msg.angular.y = 0;
                    mqtt_msg.angular.z = 0; break;
            }
            pub.publish(mqtt_msg);
            ros::spinOnce();
        }
        loop_rate.sleep();
    }

    // Disconnect
	try {
		cout << "\nDisconnecting from the MQTT server..." << flush;
		client.disconnect()->wait();
		cout << "OK" << endl;
	}
	catch (const mqtt::exception& exc) {
		cerr << exc << endl;
		return 1;
	}
  
    return 0;
}
