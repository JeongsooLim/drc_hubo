
#include "head_plugin.h"


#define PODO_ADDR       "10.12.3.30"
#define PODO_PORT       8888

const float     D2Rf = 0.0174533;
const float     R2Df = 57.2957802;

namespace gazebo
{

void HeadPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    std::cout << "\033[1;32m===================================" << std::endl;
    std::cout << "   Now Start the HeadPlugin..!!" << std::endl << std::endl;
    std::cout << "   Developer: Jeongsoo Lim" << std::endl;
    std::cout << "   E-mail   : yjs0497@kaist.ac.kr" << std::endl;
    std::cout << "===================================\033[0m" << std::endl;

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()){
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }
    ros::NodeHandle n;
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
    cmd_sub = n.subscribe("drc_head_cmd", 1, &HeadPlugin::head_cmd_callback, this);
    scan_sub = n.subscribe("scan", 2, &HeadPlugin::scan_callback, this);

    cloud.height = 1;
    cloud.fields.resize (3);
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[0].count = 1;
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[1].count = 1;
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[2].count = 1;
    cloud.point_step = 12;

    model = _model;
    JCon = new physics::JointController(model);

    std::cout << "Model Name : " << model->GetName() << std::endl;

    joint_state.name.resize(1);
    joint_state.position.resize(1);


    JPtr_Hokuyo = model->GetJoint("DRC_head::Head_Sensor");
    if(JPtr_Hokuyo == NULL){
        JPtr_Hokuyo = model->GetJoint("DRC_head_kinect::Head_Sensor");
    }

    joint_state.name[0] = "Head_Sensor";
    ref_Hokuyo = 0.0;

    toCamera.setOrigin(tf::Vector3(0.05, -0.043, -0.042));
    toCamera.setRotation(tf::Quaternion(0,0,0,1));
    toHokuyo.setOrigin(tf::Vector3(0, -0.042, 0.036));
    toHokuyo.setRotation(tf::Quaternion(0,0,0,1));
    toStreaming.setOrigin(tf::Vector3(0.082, -0.063, 0.253));
    toStreaming.setRotation(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.1746, 0.0)));


    // Listen to the update event. This event is broadcast every simulation iteration.
    UpdateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&HeadPlugin::OnUpdate, this, _1));
}

void HeadPlugin::scan_callback(const sensor_msgs::LaserScanPtr &scan){
    if(jointData.mode != SCAN_MODE_SCAN)
        return;

    cloud.data.resize(cloud.point_step * (cloud_accum_cnt + scan->ranges.size()));
    for(int i=0; i<scan->ranges.size(); i++){
        if(scan->ranges[i] <= scan->range_min || scan->ranges[i] >= scan->range_max)
            continue;

        float *pstep = (float*)&cloud.data[cloud_accum_cnt * cloud.point_step];

        pstep[0] = scan->ranges[i] * cos(scan->angle_min + (float)i * scan->angle_increment) * cos(ref_Hokuyo) + 0.036 * sin(ref_Hokuyo) + 0.085;
        pstep[1] = scan->ranges[i] * sin(scan->angle_min + (float)i * scan->angle_increment);
        pstep[2] = scan->ranges[i] * cos(scan->angle_min + (float)i * scan->angle_increment) * (-sin(ref_Hokuyo)) + 0.036 * cos(ref_Hokuyo) + 0.165;

        cloud_accum_cnt++;
    }
    cloud.width = cloud_accum_cnt;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step);
}

void HeadPlugin::head_cmd_callback(const drc_plugin::DRC_HEAD_CMD::ConstPtr &cmd){
    std::cout << "Got Head Command..!!" << std::endl;
    switch(cmd->cmd){
    case 1: // scan from A to B
        scanData.target = cmd->end_pos;
        scanData.timeMs = cmd->timeMs;
        setMoveJoint(cmd->start_pos, 500, SCAN_MODE_GOTO_START_POS);
        break;
    case 2: // go to target pos
        setMoveJoint(cmd->end_pos, 500, SCAN_MODE_JUST_MOVE);
        break;
    }
}


void HeadPlugin::OnUpdate(const common::UpdateInfo &){
    static int cnt = 0;

    moveJoint();
    ref_Hokuyo = jointData.current;
    JPtr_Hokuyo->SetPosition(0, ref_Hokuyo);

    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = ref_Hokuyo;
    joint_pub.publish(joint_state);

    if(cnt%10 == 0){
        broadcaster.sendTransform(
                    tf::StampedTransform(
                        toHokuyo,
                        ros::Time::now(),
                        "Body_Hokuyo",
                        "Sensor_scan"));
        broadcaster.sendTransform(
                    tf::StampedTransform(
                        toCamera,
                        ros::Time::now(),
                        "Body_Hokuyo",
                        "Sensor_camera"));
        broadcaster.sendTransform(
                    tf::StampedTransform(
                        toStreaming,
                        ros::Time::now(),
                        "Body_Head",
                        "Sensor_streaming"));
    }

    cnt++;
}

void HeadPlugin::setMoveJoint(float target, long msTime, int mode){
    jointData.flag = false;
    jointData.togo = target;
    jointData.init = jointData.current;
    jointData.delta = jointData.togo - jointData.current;
    jointData.curCnt = 0;
    jointData.goalCnt = msTime;
    jointData.flag = true;
    jointData.mode = mode;
}

void HeadPlugin::moveJoint(){
    if(jointData.flag == true){
        jointData.curCnt++;
        if(jointData.goalCnt <= jointData.curCnt){
            jointData.goalCnt = jointData.curCnt = 0;
            jointData.current = jointData.togo;
            jointData.flag = false;
            if(jointData.mode == SCAN_MODE_GOTO_START_POS){
                cloud_accum_cnt = 0;
                setMoveJoint(scanData.target, scanData.timeMs, SCAN_MODE_SCAN);
            }else if(jointData.mode == SCAN_MODE_SCAN){
                jointData.mode = SCAN_MODE_NONE;

                cloud.header.frame_id = "Body_Head";
                cloud.header.stamp = ros::Time::now();
                cloud_pub.publish(cloud);
            }else{
                jointData.mode = SCAN_MODE_NONE;
            }
        }else{
            jointData.current = jointData.init + jointData.delta*jointData.curCnt/jointData.goalCnt;
        }
    }
}


GZ_REGISTER_MODEL_PLUGIN(HeadPlugin)
}

