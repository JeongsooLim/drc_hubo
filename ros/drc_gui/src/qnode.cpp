/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/drc_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace drc_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
    init_argv(argv),
    mapFrameId("/map"),
    objectFramePrefix("object")
{
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.

    ros::NodeHandle pnh("~");
    pnh.param("map_frame_id", mapFrameId, mapFrameId);
    pnh.param("object_prefix", objectFramePrefix, objectFramePrefix);

    ros::NodeHandle n;
    sub_object_tf = n.subscribe("objectsStamped", 1, &QNode::callback_object_tf, this);
    pub_object_pos = n.advertise<drc_gui::SendPos>("send_pos_topic", 2);

    ac = new MoveBaseClient("/move_base", true);
    if(!ac->waitForServer(ros::Duration(0,0))){
        ROS_ERROR("Waiting for the move_base action server to come up");
        return false;
    }else{
        ROS_INFO("Success to connect move_base action server");
    }

	start();
	return true;
}

void QNode::publishPos(){
    drc_gui::SendPos msg;

    msg.x = camera_pos[0];
    msg.y = camera_pos[1];
    msg.z = camera_pos[2];
    pub_object_pos.publish(msg);
}

void QNode::callback_object_tf(const ObjectsStampedConstPtr &msg){
    if(msg->objects.data.size()){
        for(unsigned int i=0; i<msg->objects.data.size(); i+=12)
        {
            // get data
            int id = (int)msg->objects.data[i];
            std::string objectFrameId = QString("%1_%2").arg(objectFramePrefix.c_str()).arg(id).toStdString(); // "object_1", "object_2"

            tf::StampedTransform pose;
            tf::StampedTransform poseCam;
            try
            {
                // Get transformation from "object_#" frame to target frame "map"
                // The timestamp matches the one sent over TF
                tfListener.lookupTransform(mapFrameId, objectFrameId, msg->header.stamp, pose);
                tfListener.lookupTransform(msg->header.frame_id, objectFrameId, msg->header.stamp, poseCam);
            }
            catch(tf::TransformException & ex)
            {
                ROS_WARN("%s",ex.what());
                continue;
            }

            // Here "pose" is the position of the object "id" in "/map" frame.
            ROS_INFO("Object_%d [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
                    id, mapFrameId.c_str(),
                    pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(),
                    pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w());
            ROS_INFO("Object_%d [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
                    id, msg->header.frame_id.c_str(),
                    poseCam.getOrigin().x(), poseCam.getOrigin().y(), poseCam.getOrigin().z(),
                    poseCam.getRotation().x(), poseCam.getRotation().y(), poseCam.getRotation().z(), poseCam.getRotation().w());

            global_pos[0] = pose.getOrigin().x();
            global_pos[1] = pose.getOrigin().y();
            global_pos[2] = pose.getOrigin().z();

            global_ori[0] = pose.getRotation().x();
            global_ori[1] = pose.getRotation().y();
            global_ori[2] = pose.getRotation().z();
            global_ori[3] = pose.getRotation().w();

            camera_pos[0] = poseCam.getOrigin().x();
            camera_pos[1] = poseCam.getOrigin().y();
            camera_pos[2] = poseCam.getOrigin().z();

            camera_ori[0] = poseCam.getRotation().x();
            camera_ori[1] = poseCam.getRotation().y();
            camera_ori[2] = poseCam.getRotation().z();
            camera_ori[3] = poseCam.getRotation().w();
        }
    }
    Q_EMIT sig_new_object_tf();
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


}  // namespace drc_gui
