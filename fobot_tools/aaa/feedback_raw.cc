#include <string>
#include <ros/ros.h>
#include <time.h>
#include "scservo/SCServo.h"
#include "std_msgs/Float64MultiArray.h"
#include "Motor_parameter.h"

#define MSG_FREQ 10

int main(int argc, char **argv)
{
    SMSBL s;
    std_msgs::Float64MultiArray msg;
    std_msgs::Float64MultiArray calculated_msg;
    for (size_t i = 0; i < IDN; i++)
    {
        msg.data.push_back(0);
        msg.data.push_back(0);
        calculated_msg.data.push_back(0);
    }
    ros::init(argc, argv, "fobot_raw_joint_position_feedback");
    ros::NodeHandle n;
    ros::Publisher pos_pub = n.advertise<std_msgs::Float64MultiArray>("/fobot_raw_joint_position_publisher", 100);
    ros::Publisher position_pub = n.advertise<std_msgs::Float64MultiArray>("/fobot_joint_position_publisher", 100);
    ros::Rate loop_rate(MSG_FREQ);
    if (s.begin(1000000, "/dev/ttyUSB0"))
    {
        ROS_INFO("Port \"dev/ttyUSB0\" is opened!");
    }
    else
    {
        ROS_ERROR("Cannot open the port!");
        return 1;
    }

    size_t loop = 0;

    while (ros::ok())
    {
        loop++;
        ROS_INFO_STREAM_ONCE("Feedback raw: Loop " << loop);
        for (int i = 0; i < IDN; i++)
        {
            if (s.FeedBack(i) != -1)
            {
                msg.data.at(i*2) = s.ReadPos(-1);
                msg.data.at(i*2+1) = s.ReadSpeed(-1);
            }
            else
            {
                ROS_ERROR_STREAM_ONCE("FEEDBACK_READ_ERROR: Loop " << loop << ", Joint " << i);
            }
        }

        for (size_t i = 0; i < 6; i++)
        {
            calculated_msg.data.at(i) = ((msg.data.at(i) - BiasPos[i]) / SMSPOSTRANS / Dirction[i]);
        }
        calculated_msg.data.at(2) = ((msg.data.at(1) + msg.data.at(2) - BiasPos[2]) / SMSPOSTRANS);

        pos_pub.publish(msg);
        position_pub.publish(calculated_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
