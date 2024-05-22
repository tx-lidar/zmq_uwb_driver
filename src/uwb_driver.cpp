#include <ros/ros.h>
#include <zmq.h>
#include <zmq.hpp>
#include <time.h>
#include <string>
#include <assert.h>
#include <jsoncpp/json/json.h>
#include "zmq_uwb_driver/uwb.h"

zmq_uwb_driver::uwb uwb_msg;

int main(int argc, char** argv)
{
    //初始化ros节点
    ros::init(argc, argv, "uwb_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Publisher uwb_pub = nh.advertise<zmq_uwb_driver::uwb>("uwb_pose", 10);

    //初始化上下文
    zmq::context_t context(1); 

    //创建订阅者套接字
    zmq::socket_t subscriber(context, ZMQ_SUB);
    
    //套接字选项，正确关闭zmq
    int linger = 0;
    subscriber.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));

    //连接tcp端口
    subscriber.connect("tcp://localhost:5581");

    //订阅关键字
    std::string TOPIC = "filter_imu_uwb_info";
    //设置订阅选项，订阅名为TOPIC的主题
    subscriber.setsockopt(ZMQ_SUBSCRIBE, TOPIC.c_str(), TOPIC.length());

    //设置循环频率
    ros::Rate loop_rate(10);
    while (ros::ok()) 
    {
        bool printed = false;
        if (!printed)
        {
            std::cout << "uwb_node: Received UWB message" << std::endl;
            printed = true;
        }
        //捕获异常
        try
        {
            //接收消息
            zmq::message_t message;
            int rc = subscriber.recv(&message);
            std::string message_str(static_cast<char*>(message.data()), message.size());

            //调用reader解析meeeage数据，将解析后的数据储存在root中
            Json::Reader reader;
            Json::Value root;
            //调用reader解析报文数据，解析成功返回true
            if (reader.parse(message_str, root))
            {
                //从root获取数据
                double timestamp = root["timestamp"].asDouble();
                //巷道层数
                double tunnel_layer = root["tunnel_layer"].asDouble();
                //坐标
                double x = root["location"]["x"].asDouble();
                double y = root["location"]["y"].asDouble();
                double z = root["location"]["z"].asDouble();
                //yaw
                double yaw = root["location"]["yaw"].asDouble();
                //定位质量等级
                int locate_quality = root["location"]["locate_quality"].asInt();
                //速度
                double speed = root["location"]["speed"].asDouble();
                // ROS_INFO("Received UWB message: %s", message);

                uwb_msg.header.frame_id = "uwb";
                uwb_msg.header.stamp= ros::Time().fromSec(timestamp);
                uwb_msg.tunnel_layer = tunnel_layer;
                uwb_msg.x = x;
                uwb_msg.y = y;
                uwb_msg.z = z;
                uwb_msg.yaw = yaw;
                uwb_msg.locate_quality = locate_quality;
                uwb_msg.speed = speed;
                uwb_pub.publish(uwb_msg);
            } 
        }
        catch (const zmq::error_t& e) 
        {
            std::cerr << "ZMQ Error: " << e.what() << std::endl;
            continue;
        }
        //延时
        loop_rate.sleep();
    }
    //关闭套接字，上下文
    subscriber.close();
    context.close();
    return 0;
}