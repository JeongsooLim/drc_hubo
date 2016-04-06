#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/math/Vector3.hh>
#include <stdio.h>

////////////////////////////

#include "GazeboLANData.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/physics/JointWrench.hh>
#include <gazebo/physics/Model.hh>


// for socket client
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>


#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <drc_plugin/DRC_HEAD_CMD.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

namespace gazebo{

typedef struct _JOINT_DATA_{
    float   delta;
    float   current;
    float   togo;
    float   init;
    long    goalCnt;
    long    curCnt;
    int     flag;
    int     mode;
}JOINT_DATA;

typedef struct _SCAN_DATA_{
    float   target;
    long    timeMs;
}SCAN_DATA;

enum SCAN_MODE{
    SCAN_MODE_NONE = 0,
    SCAN_MODE_JUST_MOVE,
    SCAN_MODE_GOTO_START_POS,
    SCAN_MODE_SCAN
};

class HeadPlugin : public ModelPlugin
{
public:
    void    Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void    OnUpdate(const common::UpdateInfo &);



private:
    ros::Publisher  joint_pub;
    ros::Publisher  cloud_pub;
    ros::Subscriber cmd_sub;
    ros::Subscriber scan_sub;

    sensor_msgs::JointState joint_state;
    tf::TransformBroadcaster broadcaster;
    tf::Transform    toHokuyo;
    tf::Transform    toCamera;
    tf::Transform    toStreaming;

    sensor_msgs::PointCloud2 cloud;
    int     cloud_accum_cnt;

    void    head_cmd_callback(const drc_plugin::DRC_HEAD_CMD::ConstPtr &cmd);
    void    scan_callback(const sensor_msgs::LaserScanPtr &scan);

    // Pointer to the model
    physics::ModelPtr   model;
    sdf::ElementPtr     element;

    // Pointer to the update event connection
    event::ConnectionPtr UpdateConnection;


    //Joint controller
    physics::JointController    *JCon;
    physics::JointPtr           JPtr_Hokuyo;

    // reference ====
    double  ref_Hokuyo;

    JOINT_DATA  jointData;
    SCAN_DATA   scanData;

    void    setMoveJoint(float target, long msTime, int mode);
    void    moveJoint();
};



}// namespace gazebo
