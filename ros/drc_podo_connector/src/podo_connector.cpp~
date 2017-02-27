#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/JointState.h>

// for socket client
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "ROSLANData.h"


#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <drc_podo_connector/DRC_HEAD_CMD.h>
#include <drc_podo_connector/SendPos.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


#define PODO_ADDR       "10.12.3.30"
//#define PODO_ADDR       "127.0.0.1"
#define PODO_PORT       5000

const float     D2Rf = 0.0174533;
const float     R2Df = 57.2957802;
char ip[20];

int sock = 0;
struct sockaddr_in  server;

pthread_t LANTHREAD_t;
int threadWorking = false;
int connectionStatus = false;


int     CreateSocket(const char *addr, int port);
int     Connect2Server();
void*   LANThread(void*);
void    NewRXData();

LAN_PODO2ROS    RXData;
LAN_ROS2PODO    TXData;

int     RXDataSize;
int     TXDataSize;
void*   RXBuffer;
void*   TXBuffer;


sensor_msgs::JointState joint_state;
ros::Publisher joint_pub;
ros::Publisher head_cmd_pub;
ros::Publisher odom_pub;


tf::TransformBroadcaster *podom_broadcaster;
tf::TransformBroadcaster *pfootprint_broadcaster;

ros::Subscriber base_move_sub;
ros::Subscriber obj_pos_sub;


void base_move_callback(const geometry_msgs::Twist::ConstPtr &msg){
    if(connectionStatus){
        TXData.vx = msg->linear.x;
        TXData.vth = -msg->angular.z;

        write(sock, &TXData, TXDataSize);
        //ROS_ERROR("%f, %f", msg->linear.x, msg->angular.z);
    }
}

void send_pos_callback(const drc_podo_connector::SendPos::ConstPtr &msg){
    std::cout << "send_pos_callback" << std::endl;
    if(connectionStatus){
        tf::TransformListener listener;
        tf::StampedTransform streaming;
        std::string b_stream = "Sensor_streaming";
        std::string b_tor = "Body_TORSO";
        ros::Duration timeout(0.05);
        try{
            listener.waitForTransform(b_tor, b_stream, ros::Time(0), ros::Duration(0.05));
            listener.lookupTransform(b_tor, b_stream, ros::Time(0), streaming);

            tf::Transform tempTF;
            tf::Transform objTF;
            objTF.setIdentity();
            objTF.setOrigin(tf::Vector3(msg->x, msg->y, msg->z));
            tempTF.mult(streaming, objTF);

            TXData.pos[0] = tempTF.getOrigin().x();
            TXData.pos[1] = tempTF.getOrigin().y();
            TXData.pos[2] = tempTF.getOrigin().z();

            write(sock, &TXData, TXDataSize);
            std::cout << TXData.pos[0] << ", " << TXData.pos[1] << ", " << TXData.pos[2] << std::endl;
        }catch(tf::TransformException &ex){
            //ROS_WARN("%s",ex.what());
        }
    }
}


int main(int argc, char **argv)
{
    std::cout << "\033[1;32m===================================" << std::endl;
    std::cout << "   Now Start the PODOConnetor..!!" << std::endl << std::endl;
    std::cout << "   Developer: Jeongsoo Lim" << std::endl;
    std::cout << "   E-mail   : yjs0497@kaist.ac.kr" << std::endl;
    std::cout << "===================================\033[0m" << std::endl;

    ros::init(argc, argv, "podo_connector");
    ros::NodeHandle n;
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    head_cmd_pub = n.advertise<drc_podo_connector::DRC_HEAD_CMD>("drc_head_cmd", 1);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 20);

    base_move_sub = n.subscribe("/cmd_vel", 10, base_move_callback);
    obj_pos_sub = n.subscribe("send_pos_topic", 10, send_pos_callback);

    tf::TransformBroadcaster odom_broadcaster;
    podom_broadcaster = &odom_broadcaster;
    tf::TransformBroadcaster footprint_broadcaster;
    pfootprint_broadcaster = &footprint_broadcaster;


    joint_state.name.resize(NO_OF_JOINTS+9*2+4+1+4);
    joint_state.position.resize(NO_OF_JOINTS+9*2+4+1+4);
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(i == RHAND || i == LHAND)
            continue;
        joint_state.name[i] = JointNameList[i];
    }
    joint_state.name[RHAND] = "RFWH"; // use for foot passive wheel
    joint_state.name[LHAND] = "LFWH"; // use for foot passive wheel

    joint_state.name[NO_OF_JOINTS + 0] = "LHAND_a1";
    joint_state.name[NO_OF_JOINTS + 1] = "LHAND_a2";
    joint_state.name[NO_OF_JOINTS + 2] = "LHAND_a3";
    joint_state.name[NO_OF_JOINTS + 3] = "LHAND_b1";
    joint_state.name[NO_OF_JOINTS + 4] = "LHAND_b2";
    joint_state.name[NO_OF_JOINTS + 5] = "LHAND_b3";
    joint_state.name[NO_OF_JOINTS + 6] = "LHAND_c1";
    joint_state.name[NO_OF_JOINTS + 7] = "LHAND_c2";
    joint_state.name[NO_OF_JOINTS + 8] = "LHAND_c3";

    joint_state.name[NO_OF_JOINTS + 9 + 0] = "RHAND_a1";
    joint_state.name[NO_OF_JOINTS + 9 + 1] = "RHAND_a2";
    joint_state.name[NO_OF_JOINTS + 9 + 2] = "RHAND_a3";
    joint_state.name[NO_OF_JOINTS + 9 + 3] = "RHAND_b1";
    joint_state.name[NO_OF_JOINTS + 9 + 4] = "RHAND_b2";
    joint_state.name[NO_OF_JOINTS + 9 + 5] = "RHAND_b3";
    joint_state.name[NO_OF_JOINTS + 9 + 6] = "RHAND_c1";
    joint_state.name[NO_OF_JOINTS + 9 + 7] = "RHAND_c2";
    joint_state.name[NO_OF_JOINTS + 9 + 8] = "RHAND_c3";

    joint_state.name[NO_OF_JOINTS + 18 + 0] = "RAFT";
    joint_state.name[NO_OF_JOINTS + 18 + 1] = "LAFT";
    joint_state.name[NO_OF_JOINTS + 18 + 2] = "RWFT";
    joint_state.name[NO_OF_JOINTS + 18 + 3] = "LWFT";

    joint_state.name[NO_OF_JOINTS + 18 + 4] = "head_joint";

    joint_state.name[NO_OF_JOINTS + 18 + 5] = "RFWH";
    joint_state.name[NO_OF_JOINTS + 18 + 6] = "LFWH";
    joint_state.name[NO_OF_JOINTS + 18 + 7] = "RFUNI";
    joint_state.name[NO_OF_JOINTS + 18 + 8] = "LFUNI";


    ros::Rate loop_rate(50);

    // Create Socket ---------------------
    FILE *fpNet = NULL;
    fpNet = fopen("/home/rainbow/catkin_ws/src/drc_hubo/ros/settings/network.txt", "r");
    if(fpNet == NULL){
        std::cout << ">>> Network File Open Error..!!" << std::endl;
        sprintf(ip, PODO_ADDR);
    }else{
        std::cout << ">>> Network File Open Success..!!" << std::endl;
        fscanf(fpNet, "%s", ip);
        fclose(fpNet);
    }

    if(CreateSocket(ip, PODO_PORT)){
        ROS_INFO("Created Socket..");

        RXDataSize = sizeof(LAN_PODO2ROS);
        TXDataSize = sizeof(LAN_ROS2PODO);
        RXBuffer = (void*)malloc(RXDataSize);
        TXBuffer = (void*)malloc(TXDataSize);
        int threadID = pthread_create(&LANTHREAD_t, NULL, &LANThread, NULL);
        if(threadID < 0){
            ROS_ERROR("Create Thread Error..");
            return 0;
        }
    }else{
        ROS_ERROR("Create Socket Error..");
        ROS_ERROR("Terminate Node..");
        return 0;
    }



    tf::TransformListener listener;
    tf::StampedTransform left;
    tf::StampedTransform right;
    ros::Time tzero(0);

    while(ros::ok()){
        ros::spinOnce();

        // Wheel Mode
        if(RXData.JointEncoder[RKN] > 145.0 && RXData.JointEncoder[LKN] > 145.0){
            try{
                listener.lookupTransform("Body_TORSO", "Body_LWH", tzero, left);
                listener.lookupTransform("Body_TORSO", "Body_RWH", tzero, right);

                float x = (left.getOrigin().x() + right.getOrigin().x()) / 2.0;
                float y = (left.getOrigin().y() + right.getOrigin().y()) / 2.0;
                float z = (left.getOrigin().z() + right.getOrigin().z()) / 2.0 - 0.06;
                pfootprint_broadcaster->sendTransform(
                            tf::StampedTransform(
                                tf::Transform(
                                    tf::Quaternion(0,0,0,1),
                                    tf::Vector3(-x,-y,-z)),
                            ros::Time::now(),
                            "base_footprint",
                            "Body_TORSO"));
//                std::cout << "Wheel Mode: " << x << ", " << y << ", " << z << std::endl;
            }catch(tf::TransformException &ex){
                //ROS_WARN("%s",ex.what());
            }
        }
        // Walking Mode
        else{
            try{
                listener.lookupTransform("Body_RAFT", "Body_TORSO", tzero, right);
                listener.lookupTransform("Body_LAFT", "Body_TORSO", tzero, left);

                float x, y, z;
                if(fabs(left.getOrigin().z()-right.getOrigin().z()) < 0.03){
                    x = (left.getOrigin().x() + right.getOrigin().x()) / 2.0;
                    y = (left.getOrigin().y() + right.getOrigin().y()) / 2.0;
                    z = (left.getOrigin().z() + right.getOrigin().z()) / 2.0 + 0.05;

                    pfootprint_broadcaster->sendTransform(
                                tf::StampedTransform(
                                    tf::Transform(
//                                        tf::Quaternion(q.x, q.y, q.z, q.w),
                                        tf::Quaternion(0,0,0,1),
                                        tf::Vector3(x, y, z)),
                                    ros::Time::now(),
                                    "base_footprint",
                                    "Body_TORSO"));
                }else if(right.getOrigin().z() > left.getOrigin().z()){
                    x = right.getOrigin().x();
                    y = right.getOrigin().y();
                    z = right.getOrigin().z() + 0.05;

                    pfootprint_broadcaster->sendTransform(
                                tf::StampedTransform(
                                    tf::Transform(
//                                        tf::Quaternion(q.x, q.y, q.z, q.w),
                                        tf::Quaternion(0,0,0,1),
                                        tf::Vector3(x, y, z)),
                                    ros::Time::now(),
                                    "base_footprint",
                                    "Body_TORSO"));
                }else if(left.getOrigin().z() > right.getOrigin().z()){
                    x = left.getOrigin().x();
                    y = left.getOrigin().y();
                    z = left.getOrigin().z() + 0.05;

                    pfootprint_broadcaster->sendTransform(
                                tf::StampedTransform(
                                    tf::Transform(
//                                        tf::Quaternion(q.x, q.y, q.z, q.w),
                                        tf::Quaternion(0,0,0,1),
                                        tf::Vector3(x, y, z)),
                                    ros::Time::now(),
                                    "base_footprint",
                                    "Body_TORSO"));
                }

            }catch(tf::TransformException &ex){
                //ROS_WARN("%s",ex.what());
            }
        }

        loop_rate.sleep();
    }
    return 0;
}


int CreateSocket(const char *addr, int port){
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock == -1){
        return false;
    }
    server.sin_addr.s_addr = inet_addr(addr);
    server.sin_family = AF_INET;
    server.sin_port = htons(port);

    int optval = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

    return true;
}


int Connect2Server(){
    if(connect(sock, (struct sockaddr*)&server, sizeof(server)) < 0){
        return false;
    }
    std::cout << "Client connect to server!! (PODO_CONNECTOR)" << std::endl;
    return true;
}

void NewRXData(){
    joint_state.header.stamp = ros::Time::now();
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(i == RHAND || i == LHAND){
            joint_state.position[i] = 0.0;
            continue;
        }
        joint_state.position[i] = RXData.JointEncoder[i] * D2Rf;
    }
    joint_state.position[LEB] += -20.0 * D2Rf;
    joint_state.position[REB] += -20.0 * D2Rf;
    joint_state.position[RSR] += -15.0 * D2Rf;
    joint_state.position[LSR] += 15.0 * D2Rf;

    for(int i=0; i<9; i++){
        joint_state.position[NO_OF_JOINTS + i] = RXData.JointEncoder[LHAND] * D2Rf;
        joint_state.position[NO_OF_JOINTS + 9 + i] = RXData.JointEncoder[RHAND] *D2Rf;
    }
    joint_state.position[NO_OF_JOINTS + 18 + 0] = 0.0;
    joint_state.position[NO_OF_JOINTS + 18 + 1] = 0.0;
    joint_state.position[NO_OF_JOINTS + 18 + 2] = 0.0;
    joint_state.position[NO_OF_JOINTS + 18 + 3] = 0.0;

    joint_state.position[NO_OF_JOINTS + 18 + 4] = 0.0;

    joint_state.position[NO_OF_JOINTS + 18 + 5] = 0.0;
    joint_state.position[NO_OF_JOINTS + 18 + 6] = 0.0;
    joint_state.position[NO_OF_JOINTS + 18 + 7] = 0.0;
    joint_state.position[NO_OF_JOINTS + 18 + 8] = 0.0;

    joint_pub.publish(joint_state);


    //--------------------------

    // Wheel Mode
//    if(RXData.JointEncoder[RKN] > 145.0 && RXData.JointEncoder[LKN] > 145.0){
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(RXData.odom_theta);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";   // "wheel_center";

        odom_trans.transform.translation.x = RXData.odom_x;
        odom_trans.transform.translation.y = RXData.odom_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        podom_broadcaster->sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = RXData.odom_x;
        odom.pose.pose.position.y = RXData.odom_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = RXData.vx;
        odom.twist.twist.linear.y = RXData.vy;
        odom.twist.twist.angular.z = RXData.vth;

        //publish the message
        odom_pub.publish(odom);
//    }

    drc_podo_connector::DRC_HEAD_CMD cmd;
    switch(RXData.cmd.cmd){
    case 1:
        cmd.cmd = 1;
        cmd.start_pos = RXData.cmd.param_f[0];
        cmd.end_pos = RXData.cmd.param_f[1];
        cmd.timeMs = RXData.cmd.param_f[2];
        head_cmd_pub.publish(cmd);
        break;
    case 2:
        cmd.cmd = 2;
        cmd.end_pos = RXData.cmd.param_f[0];
        head_cmd_pub.publish(cmd);
        break;
    }
}

void* LANThread(void *){
    threadWorking = true;

    unsigned int tcp_status = 0x00;
    int tcp_size = 0;
    int connectCnt = 0;

    while(threadWorking){
        usleep(100);
        if(tcp_status == 0x00){
            // If client was not connected
            if(sock == 0){
                CreateSocket(ip, PODO_PORT);
            }
            if(Connect2Server()){
                tcp_status = 0x01;
                connectionStatus = true;
                connectCnt = 0;
            }else{
                if(connectCnt%10 == 0)
                    std::cout << "Connect to Server Failed.." << std::endl;
                connectCnt++;
            }
            usleep(1000*1000);
        }
        if(tcp_status == 0x01){
            // If client was connected
            tcp_size = read(sock, RXBuffer, RXDataSize);
            if(tcp_size == RXDataSize){
                memcpy(&RXData, RXBuffer, RXDataSize);
                NewRXData();
            }

            if(tcp_size == 0){
                tcp_status = 0x00;
                connectionStatus = false;
                close(sock);
                sock = 0;
                std::cout << "Socket Disconnected.." << std::endl;
            }
        }
    }
    return NULL;
}
