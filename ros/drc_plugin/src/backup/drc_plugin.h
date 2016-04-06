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


// for socket client
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>


#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

namespace gazebo{


typedef struct _GO_DATA_{
    float   delta;
    float   current;
    float   togo;
    float   init;
    long    goalCnt;
    long    curCnt;
    int     flag;
}GO_DATA;

class DRCPlugin : public ModelPlugin
{
public:
    void    Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void    OnUpdate(const common::UpdateInfo &);
    void    OnLAFTUpdate();
    void    OnRAFTUpdate();
    void    OnLWFTUpdate();
    void    OnRWFTUpdate();
    void    OnIMUUpdate();

private:
    ros::Publisher joint_pub;
    tf::TransformBroadcaster broadcaster;
    sensor_msgs::JointState joint_state;

    // Pointer to the model
    physics::ModelPtr   model;
    sdf::ElementPtr     element;

    // Pointer to the update event connection
    event::ConnectionPtr UpdateConnection;
    event::ConnectionPtr LAFTupdateConnection;
    event::ConnectionPtr RAFTupdateConnection;
//    event::ConnectionPtr LWFTupdateConnection;
//    event::ConnectionPtr RWFTupdateConnection;
    event::ConnectionPtr IMUupdateConnection;
//    event::ConnectionPtr RGBDupdateConnection;
//    event::ConnectionPtr DepthupdateConnection;

    //Joint controller
    physics::JointController    *JCon;
    physics::JointPtr           JPtrs[NO_OF_JOINTS];
    physics::JointPtr           JPtr_LHAND[9];
    physics::JointPtr           JPtr_RHAND[9];
    physics::JointPtr           JPtr_RAFT, JPtr_LAFT;
    physics::JointPtr           JPtr_RWFT, JPtr_LWFT;

//    physics::JointPtr           JPtr_RFWH, JPtr_LFWH;
//    physics::JointPtr           JPtr_RFUNI, JPtr_LFUNI;
    physics::JointPtr           JPtr_Head;
    float                       PIDGains[NO_OF_JOINTS][3];
    GO_DATA                     GainOverride[NO_OF_JOINTS];

    void    setGainOverride(int joint, float gain, long msTime);
    void    moveGainOverride(int joint);
    void    adjustAllGain();

    void    findHome(int jNum);

    //common::Time UpdateTime, UpdatePeriod;

    gazebo::sensors::ForceTorqueSensorPtr   LAFT, RAFT;
    sensors::ImuSensorPtr                   IMU;
//    gazebo::sensors::ForceTorqueSensorPtr   LWFT, RWFT;
//    sensors::ContactSensorPtr               LWFT, RWFT;
//    msgs::Contacts LFcontact, RFcontact;
//    msgs::Contacts LWcontact, RWcontact;

    math::Vector3   filt_LAF;
    math::Vector3   filt_LAT;
    math::Vector3   filt_RAF;
    math::Vector3   filt_RAT;
    math::Vector3   filt_LWF;
    math::Vector3   filt_LWT;
    math::Vector3   filt_RWF;
    math::Vector3   filt_RWT;
    double          filt_alpha;

    // reference ====
    double  refs[NO_OF_JOINTS];
    double  ref_RHAND, ref_LHAND;
    double  ref_RWH, ref_LWH;

    double  home_ref_RWH, home_ref_LWH;

    double  prev_refs[NO_OF_JOINTS];
    double  inc_refs[NO_OF_JOINTS];
    int     new_ref_cnt;


    // socket ==========
    int sock;
    struct sockaddr_in  client;
    pthread_t   LANTHREAD_t;

    int     threadWorking;
    int     connectionStatus;

    DRC_GAZEBO_JOINT    RXJointData;
    DRC_GAZEBO_SENSOR   TXSensorData;
    int     RXDataSize;
    int     TXDataSize;
    void*   RXBuffer;
    void*   TXBuffer;

    DRC_GAZEBO_GO_CMD   GainOverrideCMD;
    int     HomeJoint;

    int     CreateSocket(const char *addr, int port);
    int     Connect2Server();
    void    NewRXData();
    static void    *LANThread(void *_arg);

};



}// namespace gazebo
