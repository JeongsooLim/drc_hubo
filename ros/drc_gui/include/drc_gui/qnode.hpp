/**
 * @file /include/drc_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef drc_gui_QNODE_HPP_
#define drc_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include <tf/transform_listener.h>
#include <drc_gui/ObjectsStamped.h>
#include <drc_gui/SendPos.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace drc_gui {

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    void run();

    MoveBaseClient  *ac;


    float   global_ori[4];
    float   global_pos[3];
    float   camera_ori[4];
    float   camera_pos[3];


    std::string     mapFrameId;
    std::string     objectFramePrefix;

    tf::TransformListener   tfListener;

    ros::Subscriber sub_object_tf;
    ros::Publisher  pub_object_pos;

    void    publishPos();
    void    callback_object_tf(const ObjectsStampedConstPtr &msg);


Q_SIGNALS:
    void rosShutdown();
    void sig_new_object_tf();

private:
    int init_argc;
    char** init_argv;
    ros::Publisher chatter_publisher;
};

}  // namespace drc_gui

#endif /* drc_gui_QNODE_HPP_ */
