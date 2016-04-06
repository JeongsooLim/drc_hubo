/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/drc_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace drc_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    ui.setupUi(this);
    setWindowIcon(QIcon(":/images/icon.png"));

    qnode.init();
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(sig_new_object_tf()), this, SLOT(slot_new_object_tf()));

    QObject::connect(ui.BTN_SEND_POS, SIGNAL(clicked()), this, SLOT(on_BTN_SEND_POS()));
    QObject::connect(ui.BTN_GOTO_GOAL, SIGNAL(clicked()), this, SLOT(on_BTN_GOTO_GOAL()));
    QObject::connect(ui.BTN_GOTO_ORIGIN, SIGNAL(clicked()), this, SLOT(on_BTN_GOTO_ORIGIN()));

}

MainWindow::~MainWindow() {}

void MainWindow::on_BTN_SEND_POS(){
    qnode.publishPos();
}

void MainWindow::on_BTN_GOTO_GOAL(){
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = qnode.mapFrameId;
    goal.target_pose.header.stamp = ros::Time::now();



    double goal_yaw = 3.0516;
    double goal_x = 4.1018;
    double goal_y = 1.8656;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(goal_yaw);
    goal.target_pose.pose.orientation.x = goal_quat.x;
    goal.target_pose.pose.orientation.y = goal_quat.y;
    goal.target_pose.pose.orientation.z = goal_quat.z;
    goal.target_pose.pose.orientation.w = goal_quat.w;

    goal.target_pose.pose.position.x = goal_x;
    goal.target_pose.pose.position.y = goal_y;

    ROS_INFO("Sending Goal");
    qnode.ac->sendGoal(goal);

//    qnode.ac->waitForResult(ros::Duration(5,0));
//    if(qnode.ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//        ROS_INFO("Goto Goal Success");
//    else
//        ROS_ERROR("Goto Goal Fail");
}

void MainWindow::on_BTN_GOTO_ORIGIN(){
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = qnode.mapFrameId;
    goal.target_pose.header.stamp = ros::Time::now();


    double goal_yaw = 0.0;
    double goal_x = 0.0;
    double goal_y = 0.0;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(goal_yaw);
    goal.target_pose.pose.orientation.x = goal_quat.x;
    goal.target_pose.pose.orientation.y = goal_quat.y;
    goal.target_pose.pose.orientation.z = goal_quat.z;
    goal.target_pose.pose.orientation.w = goal_quat.w;

    goal.target_pose.pose.position.x = goal_x;
    goal.target_pose.pose.position.y = goal_y;

    ROS_INFO("Sending Origin");
    qnode.ac->sendGoal(goal);

//    qnode.ac->waitForResult(ros::Duration(5,0));
//    if(qnode.ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//        ROS_INFO("Goto Origin Success");
//    else
//        ROS_ERROR("Goto Origin Fail");
}


void MainWindow::slot_new_object_tf(){
    ui.LE_GLOBAL_POS_X->setText(QString().sprintf("%.3f", qnode.global_pos[0]));
    ui.LE_GLOBAL_POS_Y->setText(QString().sprintf("%.3f", qnode.global_pos[1]));
    ui.LE_GLOBAL_POS_Z->setText(QString().sprintf("%.3f", qnode.global_pos[2]));

    ui.LE_GLOBAL_ORI_X->setText(QString().sprintf("%.3f", qnode.global_ori[0]));
    ui.LE_GLOBAL_ORI_Y->setText(QString().sprintf("%.3f", qnode.global_ori[1]));
    ui.LE_GLOBAL_ORI_Z->setText(QString().sprintf("%.3f", qnode.global_ori[2]));
    ui.LE_GLOBAL_ORI_W->setText(QString().sprintf("%.3f", qnode.global_ori[3]));

    ui.LE_CAMERA_POS_X->setText(QString().sprintf("%.3f", qnode.camera_pos[0]));
    ui.LE_CAMERA_POS_Y->setText(QString().sprintf("%.3f", qnode.camera_pos[1]));
    ui.LE_CAMERA_POS_Z->setText(QString().sprintf("%.3f", qnode.camera_pos[2]));

    ui.LE_CAMERA_ORI_X->setText(QString().sprintf("%.3f", qnode.camera_ori[0]));
    ui.LE_CAMERA_ORI_Y->setText(QString().sprintf("%.3f", qnode.camera_ori[1]));
    ui.LE_CAMERA_ORI_Z->setText(QString().sprintf("%.3f", qnode.camera_ori[2]));
    ui.LE_CAMERA_ORI_W->setText(QString().sprintf("%.3f", qnode.camera_ori[3]));

    on_BTN_SEND_POS();
}



}  // namespace drc_gui

